#!/usr/bin/env python3
"""m20_mujoco_sim — minimal MuJoCo sim of the M20 on the ROS 2 bus (drdds interface).

Faithful stand-in for the vendor low-level sim (ADR-002/015): publishes
JointsData (/JOINTS_DATA) + ImuData (/IMU_DATA) at 200 Hz, subscribes JointsDataCmd
(/JOINTS_CMD). Per joint it applies the vendor control law
    tau = kp*(pos* - q) + kd*(vel* - qd) + tau_ff       (clamped to actuator range)
Wheels (joint index %4==3) use kp=0 -> velocity/torque control, legs hold position.

Env:  M20_MJCF=<path to M20.xml>   M20_SIM_GUI=1|0
"""
import os, math, time
import numpy as np
import mujoco
import rclpy
from rclpy.node import Node
from drdds.msg import JointsData, JointsDataCmd, ImuData
from nav_msgs.msg import Odometry            # ground-truth pose ("perfect estimator")
from sensor_msgs.msg import PointCloud2, PointField   # real LiDAR interface (/LIDAR/POINTS)
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

# --- LiDAR: the REAL M20 interface (see docs/LIDAR_RESEARCH.md) ---------------------
# The real robot's dual RoboSense 96-line units are merged by the vendor driver into
# ONE PointCloud2 on /LIDAR/POINTS, expressed in a single body frame (lidar_link),
# 360x90 deg hemispherical FOV, 0.5 m blind radius. We model a reduced-beam version of
# that: 16 elevation rings x 120 azimuths = 1920 rays at ~10 Hz (real: ~860k pts/s).
# A 2D /scan for slam_toolbox/Nav2 is DERIVED by the real pointcloud_to_laserscan node,
# exactly as it will be on the robot. Elevations are densest near horizon-down where
# ground pipes live (sensor ~0.56 m above ground; a 0.30 m pipe top at 5 m is ~ -3 deg).
LIDAR_ELEVS_DEG = [15, 10, 5, 2, 0, -2, -4, -6, -8, -10, -13, -16, -20, -25, -32, -45]
LIDAR_AZIMS = 120                    # 3 deg azimuth spacing (180 was marginal under live load)
LIDAR_BLIND, LIDAR_MAX = 0.5, 12.0   # blind radius per vendor config; our range budget
LIDAR_OFFSET = 0.10                  # lidar_link height above base_link (ASSUMED site,
                                     # top of body — verify on hardware day 1)
# Rays test geom GROUP 0 ONLY = floor + field obstacles. The robot's own geoms live in
# groups 1 (collision primitives) and 2 (visual meshes); masking them out (a) matches
# the real vendor driver, which self-filters the robot body from the merged cloud, and
# (b) is 40x faster (0.8 ms vs 31 ms per full-sweep cast — the 17 visual meshes were the
# cost). NOTE: new sim scenes must keep world obstacles in group 0 (the default).
LIDAR_GEOMGROUP = np.array([1, 0, 0, 0, 0, 0], np.uint8)

# --- LiDAR realism (sensor-realism update; closes the "gaps" table in LIDAR_RESEARCH.md)
# XYZIRT points (rslidar field convention), progressive sector casting (real motion
# skew + per-point timestamps an LIO can deskew with), range noise + dropout, and an
# optional dust mode. M20_LIDAR_REALISM=0 reverts to the clean instantaneous xyz cloud.
# Deliberately NOT modeled (documented): multipath/ghost returns off reflective tanks —
# that needs EM-level simulation; a bad fake is worse than a declared gap.
LIDAR_REALISM = os.environ.get("M20_LIDAR_REALISM", "1") == "1"
WEATHER = os.environ.get("M20_WEATHER", "clear")   # clear | dust
LIDAR_SECTORS = 20          # one sector cast per 200 Hz tick -> 20 sectors = 100 ms sweep
RANGE_SIGMA = 0.015         # gaussian range noise, m (typical +-2 cm class spec)
DROP_BASE, DROP_SLOPE = 0.02, 0.04    # dropout prob = base + slope*(d/LIDAR_MAX)
DUST_PHANTOM_P = 0.03       # dust: fraction of rays returning a 1-3 m phantom "wall"
DUST_FAR_DROP = 0.30        # dust: extra dropout for returns beyond 8 m
# intensity: base reflectivity by geom-name keyword (0-255 scale), ~50% linear falloff
# across full range; ground/unnamed geoms get the default.
REFLECTIVITY = (("tank", 200.0), ("pipe", 120.0), ("rack", 120.0), ("well", 120.0),
                ("skid", 80.0))
REFL_DEFAULT = 30.0

# Wheel-odometry kinematics. TRACK_B matches the bridge. ODOM_R is the CALIBRATED
# effective rolling radius (measured ~0.072 m from test drives) — the bridge's nominal
# 0.10 m over-commands wheel speed, so odometry must use the calibrated value or it
# over-reads distance ~38%. Real robots calibrate this exact number from test runs.
ODOM_R, TRACK_B = 0.072, 0.40
# Honest dead-reckoned odometry, exactly like a real skid-steer robot:
#   * FORWARD speed from wheel encoders — wheels slip (spin faster than the body moves),
#     so integrated distance OVER-reads => position drifts.
#   * HEADING from the IMU GYRO, NOT wheel differential. Skid-steer wheels slip too much
#     laterally for wheel-diff yaw to be usable (we measured it giving the wrong sign);
#     real skid-steer platforms integrate the gyro for heading. A small fixed gyro bias +
#     noise => heading drifts slowly.
# Net: the odom->base_link TF DRIFTS realistically and slam_toolbox must correct it.
# Ground truth is published on /odom_true for EVAL ONLY (never consumed by SLAM/Nav2).
GYRO_NOISE = 0.0005    # per-sample heading noise (rad) -> small yaw random walk

NJ = 16
WHEELS = (3, 7, 11, 15)
# Standing pose (found empirically; body ~0.46 m, level). Per leg [hipx, hipy, knee, wheel].
# NOTE: the vendor JOINT_INIT (knee≈2.76≈limit) is a FOLDED start pose, not standing.
STANCE = np.array([
    0.0, -0.7,  1.4, 0.0,   # FL
    0.0, -0.7,  1.4, 0.0,   # FR
    0.0,  0.7, -1.4, 0.0,   # HL
    0.0,  0.7, -1.4, 0.0])  # HR
MJCF = os.environ["M20_MJCF"]
GUI = os.environ.get("M20_SIM_GUI", "1") == "1"
# M20_START_POSE=folded spawns in the low resting crouch so the commander-driven
# standup behavior is REAL (rises on camera), not a robot that spawned standing.
# Values mirror m20_behaviors/behaviors/standup.py FOLDED — keep in sync.
START_POSE = os.environ.get("M20_START_POSE", "stand")
NO_ODOM_TF = os.environ.get("M20_NO_ODOM_TF", "0") == "1"
# Demo-only anti-tip stabilizer (see Sim._upright_assist). Default OFF.
UPRIGHT_ASSIST = os.environ.get("M20_UPRIGHT_ASSIST", "0") == "1"
UPRIGHT_DEADBAND = math.radians(5.0)
# Demo-only ground-truth odometry (odom frame = truth). Default OFF: tests keep drifting odom.
GT_ODOM = os.environ.get("M20_GT_ODOM", "0") == "1"
FOLDED = np.array([
    0.0, -1.0,  2.3, 0.0,   # FL
    0.0, -1.0,  2.3, 0.0,   # FR
    0.0,  1.0, -2.3, 0.0,   # HL
    0.0,  1.0, -2.3, 0.0])  # HR
SIM_HZ = 500.0
PUB_HZ = 200.0
CAM_W, CAM_H, CAM_HZ = 320, 240, 5.0   # M20_CAMERA=1 (monitoring-grade, OQ-14)


def quat_to_rpy(w, x, y, z):
    sinr, cosr = 2 * (w * x + y * z), 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny, cosy = 2 * (w * z + x * y), 1 - 2 * (y * y + z * z)
    return roll, pitch, math.atan2(siny, cosy)


class Sim(Node):
    def __init__(self):
        super().__init__("m20_mujoco_sim")
        self.m = mujoco.MjModel.from_xml_path(MJCF)
        self.m.opt.timestep = 1.0 / SIM_HZ
        # Wheel motor rotor inertia: real motors/gearboxes have it; without it the
        # velocity controller chatters (±20 rad/s) and shakes the legs. Damps that.
        for w in WHEELS:
            self.m.dof_armature[6 + w] = 0.03
        self.d = mujoco.MjData(self.m)
        # Spawn pose (stand default; folded for the standup-behavior path), auto-dropped
        # so the lowest geom rests ~on the ground. qpos: [xyz, quat(wxyz), 16 joints].
        spawn = FOLDED if START_POSE == "folded" else STANCE
        self.d.qpos[:] = 0.0
        self.d.qpos[3] = 1.0                      # unit quaternion (w=1)
        self.d.qpos[7:7 + NJ] = spawn
        self.d.qpos[2] = 1.0                      # lift high, then measure
        mujoco.mj_forward(self.m, self.d)
        self.d.qpos[2] = 1.0 - float(self.d.geom_xpos[:, 2].min()) + 0.03
        mujoco.mj_forward(self.m, self.d)
        self.lo = self.m.actuator_ctrlrange[:, 0].copy()
        self.hi = self.m.actuator_ctrlrange[:, 1].copy()
        # default command = hold the SPAWN pose (legs stiff PD, wheels free) until the
        # first /JOINTS_CMD arrives — a folded robot must rest folded, not self-stand.
        self.kp = np.full(NJ, 200.0)
        self.kd = np.full(NJ, 4.0)
        self.pos = spawn.copy()
        self.vel = np.zeros(NJ)
        self.tau = np.zeros(NJ)
        for w in WHEELS:
            self.kp[w] = 0.0
        self.jd_pub = self.create_publisher(JointsData, "/JOINTS_DATA", 10)
        self.imu_pub = self.create_publisher(ImuData, "/IMU_DATA", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.gps_pub = self.create_publisher(Odometry, "/gps", 10)   # noisy absolute pos (~5 Hz)
        self.cloud_pub = self.create_publisher(PointCloud2, "/LIDAR/POINTS", 10)  # real interface (~10 Hz)
        self.true_pub = self.create_publisher(Odometry, "/odom_true", 10)  # GROUND TRUTH (eval only)
        self.tf_bc = TransformBroadcaster(self)                        # odom->base_link
        # static TF base_link -> lidar_link (the frame /LIDAR/POINTS is expressed in,
        # matching the real robot where the merged cloud arrives in one body frame)
        self.static_bc = StaticTransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = "base_link"
        st.child_frame_id = "lidar_link"
        st.transform.translation.z = LIDAR_OFFSET
        st.transform.rotation.w = 1.0
        self.static_bc.sendTransform(st)
        # precompute the hemispherical ray table in the SENSOR frame (unit vectors),
        # AZIMUTH-MAJOR so one spinning "sector" (all 16 rings at 6 consecutive
        # azimuths) is a contiguous slice: sector s = rays [s*96, (s+1)*96). The table
        # is rotated per cast by the body rotation (sensor is body-fixed).
        dirs, rings = [], []
        for k in range(LIDAR_AZIMS):
            az = 2.0 * math.pi * k / LIDAR_AZIMS
            for ri, ed in enumerate(LIDAR_ELEVS_DEG):
                el = math.radians(ed)
                dirs.append([math.cos(el) * math.cos(az),
                             math.cos(el) * math.sin(az),
                             math.sin(el)])
                rings.append(ri)
        self._ray_dirs = np.array(dirs)                       # (NRAY, 3) sensor frame
        self._ray_ring = np.array(rings, np.uint16)
        self._nray = len(dirs)
        self._ray_geomid = np.full(self._nray, -1, np.int32)  # mj_multiRay outputs
        self._ray_dist = np.zeros(self._nray, np.float64)
        self._sector = 0                                      # next sector to cast
        self._sector_n = self._nray // LIDAR_SECTORS          # rays per sector (96)
        self._sweep = []                                      # per-sector XYZIRT chunks
        self._slow_scans = 0                                   # perf-guard counter
        # per-geom base reflectivity for the intensity channel (name-keyword lookup)
        refl = np.full(self.m.ngeom, REFL_DEFAULT, np.float32)
        for gid in range(self.m.ngeom):
            gname = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
            for kw, v in REFLECTIVITY:
                if kw in gname:
                    refl[gid] = v
                    break
        self._geom_refl = refl
        # dead-reckoned wheel-odometry pose (drifts) — seeds the odom frame at the true start
        _, _, y0 = quat_to_rpy(*self.d.qpos[3:7])
        self.odom_x = float(self.d.qpos[0])
        self.odom_y = float(self.d.qpos[1])
        self.odom_th = float(y0)
        self._prev_yaw = float(y0)             # last true yaw, for gyro turn-rate tracking
        self.odt = 1.0 / PUB_HZ
        self._gyro_bias = 0.002                # gyro bias (rad/s, ~0.1 deg/s) -> slow heading drift
        self._tick = 0
        self._rng = np.random.default_rng(0)
        self.create_subscription(JointsDataCmd, "/JOINTS_CMD", self.on_cmd, 10)
        # M20_CAMERA=1: forward-looking body camera (#11). No vendor-model edit —
        # a free camera is posed from the true base pose each frame (eye ~0.15 m
        # above base, looking along heading). Offscreen EGL render, 320x240 @5 Hz,
        # monitoring-grade (OQ-14 owns the low-latency teleop path). Opt-in flag:
        # rendering costs CPU and this box is contention-sensitive (gate batches
        # run without it).
        self.cam_on = os.environ.get("M20_CAMERA", "0") == "1"
        if self.cam_on:
            # render OFF the tick thread: software-GL frames cost ~250 ms and
            # inline rendering starved the sim to 81 Hz (measured 2026-07-20) —
            # the exact slow-robot class from the 07-17 forensics. GL contexts are
            # THREAD-AFFINE, so the Renderer must be constructed inside the render
            # thread too (constructing it here published zero frames, silently).
            # Snapshot via unlocked mj_copyData: a torn frame is cosmetic
            # (monitoring-grade only, OQ-14).
            from sensor_msgs.msg import Image
            self._ImageMsg = Image
            self.cam_pub = self.create_publisher(Image, "/camera/image_raw", 3)
            self._cam_snap = mujoco.MjData(self.m)
            import threading
            threading.Thread(target=self._camera_loop, daemon=True).start()
            self.get_logger().info(f"camera thread starting: /camera/image_raw {CAM_W}x{CAM_H} @{CAM_HZ} Hz")
        self.viewer = None
        if GUI:
            from mujoco import viewer as mj_viewer   # 'from' avoids shadowing `mujoco`
            self.viewer = mj_viewer.launch_passive(self.m, self.d)
        self.substeps = max(1, round(SIM_HZ / PUB_HZ))
        if UPRIGHT_ASSIST:
            # robot root = body of the free joint; gains scaled from its mass so a full
            # tipping moment (m*g*h at ~45 deg) is beaten with margin
            self._base_body = int(self.m.jnt_bodyid[0])
            mass = float(self.m.body_subtreemass[self._base_body])
            self._up_kp = 8.0 * mass * 9.81 * 0.5          # N*m per rad beyond the deadband
            self._up_kd = 0.8 * mass                         # N*m*s per rad
            self._up_max = 3.0 * mass * 9.81 * 0.5           # torque cap
            self.get_logger().warn(f"UPRIGHT ASSIST ON (demo only): mass {mass:.1f} kg, "
                                   f"kp {self._up_kp:.0f}, kd {self._up_kd:.0f}, cap {self._up_max:.0f} N*m")
        self.create_timer(1.0 / PUB_HZ, self.tick)
        self.get_logger().info(
            f"M20 MuJoCo sim up (GUI={GUI}, nq={self.m.nq}, substeps={self.substeps})")

    def _camera_loop(self):
        period = 1.0 / CAM_HZ
        try:
            renderer = mujoco.Renderer(self.m, CAM_H, CAM_W)   # GL context lives HERE
            cam = mujoco.MjvCamera()
        except Exception as e:  # noqa: BLE001 — camera is optional; sim must not die
            self.get_logger().warn(f"camera disabled (offscreen GL unavailable: {e})")
            return
        errs = 0
        while rclpy.ok():
            t0 = time.time()
            try:
                mujoco.mj_copyData(self._cam_snap, self.m, self.d)
                s = self._cam_snap
                x, y, z = s.qpos[0], s.qpos[1], s.qpos[2]
                _, _, yaw = quat_to_rpy(*s.qpos[3:7])
                # FIRST-PERSON camera (2026-07-27): the earlier chase cam kept the
                # robot in its own frame — and the vendor model has one pure-red
                # geom, so camera_scan locked onto the robot's own body and
                # "centered" it at any yaw (caught by the exit test's ground-truth
                # yaw check). Eye at the body front looking forward, matching the
                # real M20's front cameras; the robot is never in frame.
                cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                eye_x = x + 0.30 * math.cos(yaw)
                eye_y = y + 0.30 * math.sin(yaw)
                cam.lookat[:] = [eye_x + 0.5 * math.cos(yaw),
                                 eye_y + 0.5 * math.sin(yaw), z + 0.05]
                cam.distance = 0.5
                # az = yaw (NOT +180): measured 2026-07-27 — with +180 the mask saw
                # 0 wellhead pixels at yaw 0; the feed had only ever been verified
                # by Hz, never by content. Verify cameras by what they SEE.
                cam.azimuth = math.degrees(yaw)
                cam.elevation = 0.0
                renderer.update_scene(s, camera=cam)
                rgb = renderer.render()
                msg = self._ImageMsg()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "base_link"
                msg.height, msg.width = rgb.shape[0], rgb.shape[1]
                msg.encoding = "rgb8"
                msg.step = rgb.shape[1] * 3
                msg.data = rgb.tobytes()
                self.cam_pub.publish(msg)
            except Exception as e:  # noqa: BLE001 — log the first few, never die
                errs += 1
                if errs <= 3:
                    self.get_logger().warn(f"camera frame failed: {e}")
            time.sleep(max(0.0, period - (time.time() - t0)))

    def on_cmd(self, msg: JointsDataCmd):
        for i in range(NJ):
            j = msg.data.joints_data[i]
            self.pos[i], self.vel[i] = j.position, j.velocity
            self.kp[i], self.kd[i], self.tau[i] = j.kp, j.kd, j.torque

    def _upright_assist(self):
        """DEMO-ONLY virtual stabilizer (M20_UPRIGHT_ASSIST=1): beyond a small tilt deadband,
        torque the base back toward level (PD on roll/pitch, yaw untouched), so the robot
        cannot tip no matter what the planner commands. NOT physical — keep it off for any
        test that should catch falls (gates, sim-to-real evidence)."""
        R = np.empty(9)
        mujoco.mju_quat2Mat(R, self.d.qpos[3:7])
        R = R.reshape(3, 3)
        zb = R[:, 2]                                    # body up-axis in world
        axis = np.cross(zb, [0.0, 0.0, 1.0])            # rotate body-up toward world-up
        s = float(np.linalg.norm(axis))
        tilt = math.asin(min(1.0, s))
        if zb[2] < 0:
            tilt = math.pi - tilt
        w = R @ self.d.qvel[3:6]                        # free-joint ang. vel is body-frame
        w_tilt = w - np.array([0.0, 0.0, w[2]])         # leave yaw free
        tau = np.zeros(3)
        if tilt > UPRIGHT_DEADBAND and s > 1e-9:
            tau += self._up_kp * (tilt - UPRIGHT_DEADBAND) * axis / s
        if tilt > UPRIGHT_DEADBAND / 2:
            tau -= self._up_kd * w_tilt
        n = float(np.linalg.norm(tau))
        if n > self._up_max:
            tau *= self._up_max / n
        self.d.xfrc_applied[self._base_body, 3:6] = tau

    def tick(self):
        for _ in range(self.substeps):
            q = self.d.qpos[7:7 + NJ]
            qd = self.d.qvel[6:6 + NJ]
            t = self.kp * (self.pos - q) + self.kd * (self.vel - qd) + self.tau
            self.d.ctrl[:] = np.clip(t, self.lo, self.hi)
            if UPRIGHT_ASSIST:
                self._upright_assist()
            mujoco.mj_step(self.m, self.d)
        if self.viewer is not None:
            if not self.viewer.is_running():
                raise KeyboardInterrupt
            # sync the window at ~25 Hz, not 200 Hz — full-rate sync costs ~25%
            # real-time under WSLg software GL and the eye can't tell the difference
            if self._tick % 8 == 0:
                self.viewer.sync()
        self.publish()

    def publish(self):
        now = self.get_clock().now().to_msg()
        q = self.d.qpos[7:7 + NJ]
        qd = self.d.qvel[6:6 + NJ]
        ctrl = self.d.ctrl
        jd = JointsData()
        jd.header.stamp = now
        for i in range(NJ):
            e = jd.data.joints_data[i]
            e.position, e.velocity, e.torque = float(q[i]), float(qd[i]), float(ctrl[i])
        self.jd_pub.publish(jd)
        imu = ImuData()
        imu.header.stamp = now
        r, p, y = quat_to_rpy(*(self.d.qpos[3:7]))
        v = imu.data
        v.roll, v.pitch, v.yaw = float(r), float(p), float(y)
        v.omega_x, v.omega_y, v.omega_z = (float(x) for x in self.d.qvel[3:6])
        v.acc_x, v.acc_y, v.acc_z = 0.0, 0.0, 9.81   # placeholder linear accel
        self.imu_pub.publish(imu)
        # --- honest dead-reckoned WHEEL odometry (drifts; slam_toolbox corrects it) ---
        # recover body vx, wz from the four wheel encoder speeds (skid-steer kinematics,
        # inverse of the bridge's mixer). Left = FL(3)+HL(11), right = FR(7)+HR(15).
        wl = (qd[3] + qd[11]) * 0.5
        wr = (qd[7] + qd[15]) * 0.5
        vx = -ODOM_R * (wl + wr) * 0.5                        # forward speed from wheel encoders
        vx *= (1.0 + self._rng.normal(0.0, 0.02))            # encoder/slip noise (=> distance drift)
        # heading from the IMU gyro: tracks the TRUE turn rate (delta true-yaw) but the
        # gyro accumulates a slow bias + per-sample noise => heading drifts slowly. (Using
        # the true-yaw delta avoids the body-frame-tilt artifact of raw qvel during pivots.)
        dyaw = math.atan2(math.sin(y - self._prev_yaw), math.cos(y - self._prev_yaw))
        self._prev_yaw = y
        wz = dyaw / self.odt                                  # measured turn rate (for /odom twist)
        self.odom_th += dyaw + self._gyro_bias * self.odt + self._rng.normal(0.0, GYRO_NOISE)
        self.odom_th = math.atan2(math.sin(self.odom_th), math.cos(self.odom_th))
        self.odom_x += vx * math.cos(self.odom_th) * self.odt
        self.odom_y += vx * math.sin(self.odom_th) * self.odt
        if GT_ODOM:
            # DEMO-ONLY perfect odometry (M20_GT_ODOM=1): the odom frame IS ground truth,
            # so with a fixed map->odom (bringup M20_GT_LOC=1) Nav2 never mislocalizes.
            self.odom_x, self.odom_y, self.odom_th = float(self.d.qpos[0]), float(self.d.qpos[1]), float(y)
        oq_z, oq_w = math.sin(self.odom_th / 2), math.cos(self.odom_th / 2)
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = self.odom_x
        od.pose.pose.position.y = self.odom_y
        od.pose.pose.orientation.z = oq_z
        od.pose.pose.orientation.w = oq_w
        od.twist.twist.linear.x = float(vx)
        od.twist.twist.angular.z = float(wz)
        self.odom_pub.publish(od)
        # odom->base_link TF from the DRIFTING wheel odom; slam_toolbox adds map->odom
        # to fix it. M20_NO_ODOM_TF=1 hands this edge to an external estimator (the
        # GPS-EKF during map-building) — ONE publisher per TF edge, never two.
        if not NO_ODOM_TF:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = "odom"
            tf.child_frame_id = "base_link"
            tf.transform.translation.x = self.odom_x
            tf.transform.translation.y = self.odom_y
            tf.transform.rotation.z = oq_z
            tf.transform.rotation.w = oq_w
            self.tf_bc.sendTransform(tf)
        # ground-truth pose on /odom_true — EVAL ONLY (never consumed by SLAM/Nav2)
        tpx, tpy, tpz = self.d.qpos[0:3]
        tw, tx_, ty_, tz = self.d.qpos[3:7]
        ot = Odometry()
        ot.header.stamp = now
        ot.header.frame_id = "map"
        ot.child_frame_id = "base_link_true"
        ot.pose.pose.position.x = float(tpx)
        ot.pose.pose.position.y = float(tpy)
        ot.pose.pose.position.z = float(tpz)
        ot.pose.pose.orientation.x = float(tx_)
        ot.pose.pose.orientation.y = float(ty_)
        ot.pose.pose.orientation.z = float(tz)
        ot.pose.pose.orientation.w = float(tw)
        self.true_pub.publish(ot)
        # simulated GPS: noisy absolute position at ~5 Hz (sensor the real M20 has)
        self._tick += 1
        if self._tick % 40 == 0:
            g = Odometry()
            g.header.stamp = now
            g.header.frame_id = "map"
            g.pose.pose.position.x = float(self.d.qpos[0] + self._rng.normal(0, 0.8))
            g.pose.pose.position.y = float(self.d.qpos[1] + self._rng.normal(0, 0.8))
            self.gps_pub.publish(g)
        if LIDAR_REALISM:
            self._cast_sector(now)                   # 1 sector/tick -> 10 Hz sweeps
        elif self._tick % 20 == 0:                   # legacy clean instantaneous cloud
            self._publish_cloud(now)

    # ---- realistic LiDAR: progressive sector casting (motion skew + XYZIRT) --------
    _CLOUD_DTYPE = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "timestamp"],
                             "formats": [np.float32, np.float32, np.float32, np.float32,
                                         np.uint16, np.float64],
                             "offsets": [0, 4, 8, 12, 16, 24], "itemsize": 32})

    def _cast_sector(self, now):
        """Cast ONE azimuth sector (96 rays) from the robot's CURRENT pose; 20 sectors
        assemble into a 100 ms sweep, so the published cloud carries the same motion
        skew a real spinning unit does, with per-point capture timestamps (5 ms steps)
        an LIO can deskew with. Assumption (documented in LIDAR_RESEARCH.md): a single
        mechanical 10 Hz rotation — the real dual-unit scan pattern is unpublished, so
        we model the representative worst case for skew."""
        t0 = time.perf_counter()
        s = self._sector
        lo, hi = s * self._sector_n, (s + 1) * self._sector_n
        R = np.empty(9)
        mujoco.mju_quat2Mat(R, self.d.qpos[3:7])
        R = R.reshape(3, 3)
        origin = self.d.qpos[0:3] + R @ np.array([0.0, 0.0, LIDAR_OFFSET])
        dirs = self._ray_dirs[lo:hi]
        vecs = (dirs @ R.T).reshape(-1)
        geomid = self._ray_geomid[lo:hi]
        dist = self._ray_dist[lo:hi]
        mujoco.mj_multiRay(self.m, self.d, origin.astype(np.float64), vecs,
                           LIDAR_GEOMGROUP, True, -1, geomid, dist,
                           None, self._sector_n, LIDAR_MAX)
        d_ = dist.copy()
        valid = (d_ > LIDAR_BLIND) & (d_ < LIDAR_MAX) & (geomid >= 0)
        # range noise, then distance-dependent dropout (absorption / grazing losses)
        d_[valid] += self._rng.normal(0.0, RANGE_SIGMA, int(valid.sum()))
        drop = self._rng.random(self._sector_n) < (DROP_BASE + DROP_SLOPE * d_ / LIDAR_MAX)
        if WEATHER == "dust":
            drop |= (d_ > 8.0) & (self._rng.random(self._sector_n) < DUST_FAR_DROP)
        valid &= ~drop
        inten = self._geom_refl[np.clip(geomid, 0, None)] * (1.0 - 0.5 * d_ / LIDAR_MAX)
        if WEATHER == "dust":
            # phantom near-range "dust wall" returns on a fraction of ALL rays
            ph = self._rng.random(self._sector_n) < DUST_PHANTOM_P
            d_[ph] = self._rng.uniform(1.0, 3.0, int(ph.sum()))
            inten[ph] = 12.0
            valid |= ph
        n = int(valid.sum())
        chunk = np.zeros(n, self._CLOUD_DTYPE)
        pts = (dirs[valid] * d_[valid, None]).astype(np.float32)
        chunk["x"], chunk["y"], chunk["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
        chunk["intensity"] = np.clip(inten[valid] + self._rng.normal(0, 5.0, n), 1, 255)
        chunk["ring"] = self._ray_ring[lo:hi][valid]
        chunk["timestamp"] = now.sec + now.nanosec * 1e-9   # sector capture time
        self._sweep.append(chunk)
        if time.perf_counter() - t0 > 0.002:
            self._slow_scans += 1
            if self._slow_scans in (1, 10, 100):
                self.get_logger().warn(f"LiDAR sector cast >2ms (#{self._slow_scans})")
        self._sector += 1
        if self._sector == LIDAR_SECTORS:
            self._publish_sweep(now)
            self._sector = 0
            self._sweep = []

    def _publish_sweep(self, now):
        """Assemble the 20 sector chunks into one XYZIRT PointCloud2 (rslidar field
        layout, 32-byte points) on /LIDAR/POINTS. Header stamp = sweep END (freshest
        TF for the projection node; per-point timestamps carry true capture times)."""
        data = np.concatenate(self._sweep)
        msg = PointCloud2()
        msg.header.stamp = now
        msg.header.frame_id = "lidar_link"
        msg.height = 1
        msg.width = int(data.shape[0])
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
            PointField(name="timestamp", offset=24, datatype=PointField.FLOAT64, count=1)]
        msg.is_bigendian = False
        msg.point_step = 32
        msg.row_step = 32 * msg.width
        msg.is_dense = True
        msg.data = data.tobytes()
        self.cloud_pub.publish(msg)

    def _publish_cloud(self, now):
        """Cast the hemispherical ray pattern and publish /LIDAR/POINTS (PointCloud2,
        sensor frame lidar_link) — the real M20 interface. One mj_multiRay C call."""
        t0 = time.perf_counter()
        # body rotation matrix from the base quaternion (sensor is body-fixed: tilts too)
        R = np.empty(9)
        mujoco.mju_quat2Mat(R, self.d.qpos[3:7])
        R = R.reshape(3, 3)
        origin = self.d.qpos[0:3] + R @ np.array([0.0, 0.0, LIDAR_OFFSET])
        vecs = (self._ray_dirs @ R.T).reshape(-1)     # world-frame directions, flat
        # bodyexclude=-1: exclude nothing by body (the geomgroup mask already removes
        # the robot; 0 would wrongly exclude the WORLD body = floor + field obstacles)
        mujoco.mj_multiRay(self.m, self.d, origin.astype(np.float64), vecs,
                           LIDAR_GEOMGROUP, True, -1, self._ray_geomid, self._ray_dist,
                           None, self._nray, LIDAR_MAX)
        dist = self._ray_dist
        valid = (dist > LIDAR_BLIND) & (dist < LIDAR_MAX) & (self._ray_geomid >= 0)
        # points in the SENSOR frame (= what the real merged cloud is expressed in)
        pts = (self._ray_dirs[valid] * dist[valid, None]).astype(np.float32)
        msg = PointCloud2()
        msg.header.stamp = now
        msg.header.frame_id = "lidar_link"
        msg.height = 1
        msg.width = int(pts.shape[0])
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.is_dense = True
        msg.data = pts.tobytes()
        self.cloud_pub.publish(msg)
        # perf guard: ~4-5 ms per cloud every 20th tick costs ~2% real-time (measured
        # 9.78 Hz vs 10) — acceptable. Warn only if a cast goes genuinely bad (>8 ms);
        # fallback ladder then: azimuths 120->90, or drop elevation rings.
        if time.perf_counter() - t0 > 0.008:
            self._slow_scans += 1
            if self._slow_scans in (1, 10, 100):
                self.get_logger().warn(
                    f"LiDAR cast >8ms ({(time.perf_counter()-t0)*1e3:.1f} ms, "
                    f"#{self._slow_scans}) — consider ray fallback ladder")


def main():
    rclpy.init()
    node = Sim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
