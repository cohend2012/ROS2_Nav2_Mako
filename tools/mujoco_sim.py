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
SIM_HZ = 500.0
PUB_HZ = 200.0


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
        # Spawn at the vendor standing stance, auto-dropped so the lowest geom rests ~on
        # the ground (avoids a hard fall on start). qpos: [xyz, quat(wxyz), 16 joints].
        self.d.qpos[:] = 0.0
        self.d.qpos[3] = 1.0                      # unit quaternion (w=1)
        self.d.qpos[7:7 + NJ] = STANCE
        self.d.qpos[2] = 1.0                      # lift high, then measure
        mujoco.mj_forward(self.m, self.d)
        self.d.qpos[2] = 1.0 - float(self.d.geom_xpos[:, 2].min()) + 0.03
        mujoco.mj_forward(self.m, self.d)
        self.lo = self.m.actuator_ctrlrange[:, 0].copy()
        self.hi = self.m.actuator_ctrlrange[:, 1].copy()
        # default command = hold the standing stance (legs stiff PD, wheels free).
        self.kp = np.full(NJ, 200.0)
        self.kd = np.full(NJ, 4.0)
        self.pos = STANCE.copy()
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
        # precompute the hemispherical ray table in the SENSOR frame (unit vectors);
        # per scan the table is rotated by the body rotation (sensor is body-fixed).
        dirs = []
        for ed in LIDAR_ELEVS_DEG:
            el = math.radians(ed)
            for k in range(LIDAR_AZIMS):
                az = 2.0 * math.pi * k / LIDAR_AZIMS
                dirs.append([math.cos(el) * math.cos(az),
                             math.cos(el) * math.sin(az),
                             math.sin(el)])
        self._ray_dirs = np.array(dirs)                       # (NRAY, 3) sensor frame
        self._nray = len(dirs)
        self._ray_geomid = np.full(self._nray, -1, np.int32)  # mj_multiRay outputs
        self._ray_dist = np.zeros(self._nray, np.float64)
        self._slow_scans = 0                                   # perf-guard counter
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
        self.viewer = None
        if GUI:
            from mujoco import viewer as mj_viewer   # 'from' avoids shadowing `mujoco`
            self.viewer = mj_viewer.launch_passive(self.m, self.d)
        self.substeps = max(1, round(SIM_HZ / PUB_HZ))
        self.create_timer(1.0 / PUB_HZ, self.tick)
        self.get_logger().info(
            f"M20 MuJoCo sim up (GUI={GUI}, nq={self.m.nq}, substeps={self.substeps})")

    def on_cmd(self, msg: JointsDataCmd):
        for i in range(NJ):
            j = msg.data.joints_data[i]
            self.pos[i], self.vel[i] = j.position, j.velocity
            self.kp[i], self.kd[i], self.tau[i] = j.kp, j.kd, j.torque

    def tick(self):
        for _ in range(self.substeps):
            q = self.d.qpos[7:7 + NJ]
            qd = self.d.qvel[6:6 + NJ]
            t = self.kp * (self.pos - q) + self.kd * (self.vel - qd) + self.tau
            self.d.ctrl[:] = np.clip(t, self.lo, self.hi)
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
        # odom->base_link TF from the DRIFTING wheel odom; slam_toolbox adds map->odom to fix it
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
        if self._tick % 20 == 0:                     # ~10 Hz LiDAR
            self._publish_cloud(now)

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
