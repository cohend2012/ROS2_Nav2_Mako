#!/usr/bin/env python3
"""Live photoreal view of a splat-world sim, in the browser (viser + gsplat).

Free camera over the Gaussian splat (world frame = sim world, via the scene yaml), with
the robot drawn at its ground-truth pose (/odom_true), the robot's own photoreal camera
(/camera/image_raw from splat_camera_node.py) in the side panel, a chase-cam toggle,
and arm + go-to-goal controls (same RunMission action as tools/dev/m20).

Robot drawing (GUI "Robot" dropdown):
  MuJoCo composite (default) - the SIMULATOR'S OWN M20 model (MJCF, mounted at /model),
      posed from /odom_true + /JOINTS_DATA, rendered offscreen by MuJoCo from exactly the
      viewer camera (same pose + pinhole), then depth-composited per pixel into the gsplat
      render (robot wins where it is nearer). "Physics sim + splat renderer", GaussGym-style.
  URDF mesh - the vendor URDF drawn as viser scene meshes (the old way, for comparison).

Open http://localhost:8080 (WSL forwards the port to Windows).
Env: SPLAT_PLY, SPLAT_SCENE (as splat_camera_node.py), VIEW_PORT (8080), VIEW_W (960),
     M20_VIEW_MJCF (/model/m20_mjcf/mjcf/M20.xml), MUJOCO_GL (egl),
     VIEW_ROBOT (mujoco | urdf: initial robot drawing).

Headless check (no browser; live robot state with --live, else STANCE at the origin):
  python3 splat_viewer.py --snapshot X Y Z  LX LY LZ  OUT.png [--live]
  -> OUT.png (composite), OUT_robot.png (MuJoCo layer only) + alignment numbers on stdout.
  SNAP_BASE="X Y YAW_DEG" places the static robot elsewhere (e.g. behind furniture).
"""
import math, os, sys, threading, time
import numpy as np
import torch
from gsplat import rasterization

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from splat_camera_node import PLY, SCENE, load_splat, quat_to_mat, write_png   # one splat loader

PORT = int(os.environ.get("VIEW_PORT", 8080))
VIEW_W = int(os.environ.get("VIEW_W", 960))
AUTO_EXPLORE = os.environ.get("VIEW_AUTO_EXPLORE", "0") == "1"   # start the tour once the sim is seen
PASS_RADIUS = float(os.environ.get("VIEW_PASS_RADIUS", 1.0))   # explore: move on this close to a stop [m]
PASS_MAX_TURN = math.radians(float(os.environ.get("VIEW_PASS_MAX_TURN_DEG", 45)))  # ...only if the turn is gentler
LOC_ERR_MAX = float(os.environ.get("VIEW_LOC_ERR_MAX", 2.0))     # explore watchdog: halt above this [m] (live SLAM runs ~0.5 m; failures were 4-17 m)
LOC_ERR_START = float(os.environ.get("VIEW_LOC_ERR_START", 1.0))  # ...and refuse to start a lap above this
STOP_RETRIES = int(os.environ.get("VIEW_STOP_RETRIES", 2))        # explore: re-send a failed stop this many times
STALL_RADIUS = float(os.environ.get("VIEW_STALL_RADIUS", 1.2))    # explore: this close to a stop...
STALL_SECS = float(os.environ.get("VIEW_STALL_SECS", 5.0))        # ...and no 10 cm of progress this long = reached
LOG_PATH = os.environ.get("VIEW_LOG", "/tmp/explore_log.csv")      # black box: t,truth xy yaw roll pitch z,est xy,cmd vx wz
LAPS_DIR = os.environ.get("VIEW_LAPS_DIR", "/laps")                # recorded laps for replay (mounted host dir)
# Vendor M20 URDF (DeepRoboticsLab/URDF_model, mounted at runtime). Its 16 actuated joints
# share names + order with the sim's /JOINTS_DATA (fl, fr, hl, hr x hipx, hipy, knee, wheel).
URDF = os.environ.get("M20_URDF", "/urdf/M20.urdf")
SIM_JOINTS = [f"{leg}_{j}_joint" for leg in ("fl", "fr", "hl", "hr") for j in ("hipx", "hipy", "knee", "wheel")]
BODY = (0.82, 0.43, 0.25)          # fallback marker box if the URDF is missing
# The sim's own MJCF (robot-only M20.xml; its floor plane is hidden by geom group below).
MJCF = os.environ.get("M20_VIEW_MJCF", "/model/m20_mjcf/mjcf/M20.xml")
# Standing pose, same as tools/mujoco_sim.py STANCE (used until /JOINTS_DATA arrives).
STANCE = np.array([0.0, -0.7, 1.4, 0.0] * 2 + [0.0, 0.7, -1.4, 0.0] * 2)
NEAR, FAR = 0.05, 200.0            # shared by the splat and the MuJoCo projection
NO_DEPTH = 160.0                   # "nothing here" depth for viser (its max is ~167.7 m)
ROBOT_MODES = ("MuJoCo composite", "URDF mesh")


class MjRobot:
    """Offscreen MuJoCo render of the robot alone, from an OpenCV-convention camera.

    The scene camera (mjvGLCamera) is overwritten with the viewer's exact c2w and a
    symmetric frustum matching the gsplat pinhole K (fy = fx = (h/2)/tan(fov_y/2),
    cx = w/2, cy = h/2), so both layers share one projection pixel-for-pixel.
    """
    BUF = 2048                     # offscreen framebuffer side; larger views are rendered scaled

    def __init__(self, path):
        os.environ.setdefault("MUJOCO_GL", "egl")
        import mujoco
        self.mj = mujoco
        m = mujoco.MjModel.from_xml_path(path)
        if m.nq != 7 + 16:
            raise RuntimeError(f"{path}: nq={m.nq}, expected 23 (free base + 16 joints)")
        m.vis.global_.offwidth = m.vis.global_.offheight = self.BUF
        self.m, self.d = m, mujoco.MjData(m)
        self.gl = mujoco.GLContext(self.BUF, self.BUF)
        self.gl.make_current()
        self.con = mujoco.MjrContext(m, mujoco.mjtFontScale.mjFONTSCALE_100)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.con)
        self.scn = mujoco.MjvScene(m, maxgeom=500)
        self.scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0       # no floor to shadow; ~3x faster
        self.scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
        self.opt = mujoco.MjvOption()
        self.opt.geomgroup[:] = 0
        self.opt.geomgroup[2] = 1          # robot visual meshes only (floor = 0, collision = 1)
        self.opt.sitegroup[:] = 0
        self.cam = mujoco.MjvCamera()
        self.vis_geoms = np.flatnonzero(m.geom_group == 2)
        from OpenGL import GL
        self.gl_name = GL.glGetString(GL.GL_RENDERER).decode()
        self.set_state(None, None, None)
        self._probe_depth_convention()

    def set_state(self, p, R, q):
        d = self.d
        d.qpos[:] = 0.0
        d.qpos[2] = 0.46
        d.qpos[3] = 1.0
        if p is not None:
            d.qpos[0:3] = p
            d.qpos[3:7] = mat_to_wxyz(R)
        d.qpos[7:23] = STANCE if q is None else q
        self.mj.mj_kinematics(self.m, d)
        self.mj.mj_camlight(self.m, d)

    def render(self, c2w, fov_y, w, h):
        """-> (rgb uint8, z float32, (y0, x0)): the robot's bounding-box crop of a w x h view.
        z = metres along the view axis (same as gsplat "ED"), inf where there is no robot.
        Only the robot's projected bounds are read back and processed, so the per-frame cost
        scales with the robot's size on screen, not the view size."""
        s = min(1.0, self.BUF / max(w, h))
        rw, rh = max(1, int(w * s)), max(1, int(h * s))
        if (rw, rh) == (w, h):
            rect = self._screen_rect(c2w, fov_y, w, h)
            if rect is None:                                 # robot entirely off-screen
                return np.zeros((0, 0, 3), np.uint8), np.zeros((0, 0), np.float32), (0, 0)
            rgb, buf = self._raw(c2w, fov_y, rw, rh, rect)
            oy, ox = rect[1], rect[0]
        else:                                                # huge view: render scaled, upsample
            rgb, buf = self._raw(c2w, fov_y, rw, rh, (0, 0, rw, rh))
            iy = (np.arange(h) * rh // h)[:, None]
            ix = (np.arange(w) * rw // w)[None, :]
            rgb, buf = rgb[iy, ix], buf[iy, ix]
            oy = ox = 0
        hit = buf > 0.0 if self.reversed_z else buf < 1.0    # anything drawn (cleared = far)
        rows, cols = np.flatnonzero(hit.any(1)), np.flatnonzero(hit.any(0))
        if rows.size == 0:
            return np.zeros((0, 0, 3), np.uint8), np.zeros((0, 0), np.float32), (0, 0)
        y0, y1, x0, x1 = rows[0], rows[-1] + 1, cols[0], cols[-1] + 1
        z = self._linearize(buf[y0:y1, x0:x1].astype(np.float64))
        z = np.where(hit[y0:y1, x0:x1], z, np.inf).astype(np.float32)
        return np.ascontiguousarray(rgb[y0:y1, x0:x1]), z, (int(oy + y0), int(ox + x0))

    def _screen_rect(self, c2w, fov_y, w, h):
        """Pixel rect (x0, y0, x1, y1) bounding the robot's geoms (bounding spheres) in the view;
        the whole view if the robot straddles the camera plane; None if it is off-screen."""
        m, d = self.m, self.d
        g = self.vis_geoms
        r = m.geom_rbound[g][:, None]
        lo, hi = (d.geom_xpos[g] - r).min(0), (d.geom_xpos[g] + r).max(0)
        corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        w2c = np.linalg.inv(c2w)
        pc = corners @ w2c[:3, :3].T + w2c[:3, 3]
        if (pc[:, 2] < NEAR).any():
            return None if (pc[:, 2] < NEAR).all() else (0, 0, w, h)
        f = (h / 2) / math.tan(fov_y / 2)
        u, v = f * pc[:, 0] / pc[:, 2] + w / 2, f * pc[:, 1] / pc[:, 2] + h / 2
        x0, x1 = max(0, int(u.min()) - 2), min(w, int(math.ceil(u.max())) + 2)
        y0, y1 = max(0, int(v.min()) - 2), min(h, int(math.ceil(v.max())) + 2)
        return None if x0 >= x1 or y0 >= y1 else (x0, y0, x1, y1)

    @staticmethod
    def full(layer, w, h):
        """Crop from render() -> full-frame (rgb, z) (z inf off-robot); for checks/debug."""
        mrgb, mz, (y0, x0) = layer
        rgb, z = np.zeros((h, w, 3), np.uint8), np.full((h, w), np.inf, np.float32)
        rgb[y0:y0 + mz.shape[0], x0:x0 + mz.shape[1]] = mrgb
        z[y0:y0 + mz.shape[0], x0:x0 + mz.shape[1]] = mz
        return rgb, z

    def _raw(self, c2w, fov_y, rw, rh, rect):
        """Render at rw x rh, read back rect (x0, y0, x1, y1; image coords, top-left origin)
        -> rgb uint8, raw GL depth buffer, both top row first."""
        mj = self.mj
        mj.mjv_updateScene(self.m, self.d, self.opt, None, self.cam, mj.mjtCatBit.mjCAT_ALL, self.scn)
        t = NEAR * math.tan(fov_y / 2)
        for c in (self.scn.camera[0], self.scn.camera[1]):
            c.pos[:] = c2w[:3, 3]
            c.forward[:] = c2w[:3, 2]           # OpenCV +z (view dir)  = GL -z
            c.up[:] = -c2w[:3, 1]               # OpenCV +y (image down) = GL -y
            c.orthographic = 0
            c.frustum_near, c.frustum_far = NEAR, FAR
            c.frustum_bottom, c.frustum_top = -t, t
            c.frustum_center = 0.0
            c.frustum_width = t * rw / rh       # half-width; square pixels (fx = fy)
        mj.mjr_render(mj.MjrRect(0, 0, rw, rh), self.scn, self.con)
        x0, y0, x1, y1 = rect
        rgb = np.empty((y1 - y0, x1 - x0, 3), np.uint8)
        buf = np.empty((y1 - y0, x1 - x0), np.float32)
        mj.mjr_readPixels(rgb, buf, mj.MjrRect(x0, rh - y1, x1 - x0, y1 - y0), self.con)
        return rgb[::-1], buf[::-1]                          # GL rows are bottom-up

    def _linearize(self, buf):
        # Invert the glFrustum projection for OUR near/far. Some MuJoCo builds render with a
        # reversed-Z buffer (far = 0, near = 1; mujoco.Renderer handles both): probed at init.
        n, f = np.float32(NEAR), np.float32(FAR)
        c = -(f + n) / (f - n)
        dd = -(np.float32(2) * f * n) / (f - n)
        if self.reversed_z:
            c = np.float32(-0.5) * c - np.float32(0.5)
            dd = np.float32(-0.5) * dd
            return dd / (buf + c)
        ndc = 2 * buf - 1                                    # classic [0,1] depth buffer
        return dd / (ndc + c)

    def _probe_depth_convention(self):
        """Render an empty view: the cleared depth is 0 with reversed-Z, 1 with classic Z."""
        far_away = np.eye(4)
        far_away[:3, 3] = (0.0, 0.0, -1000.0)                 # robot 1000 m ahead = past FAR
        _, buf = self._raw(far_away, 1.0, 8, 8, (0, 0, 8, 8))
        self.reversed_z = float(buf.mean()) < 0.5

    def mesh_points(self):
        """World-frame vertices of the rendered (group-2 mesh) geoms, for alignment checks."""
        m, d, mj = self.m, self.d, self.mj
        pts = []
        for g in range(m.ngeom):
            if m.geom_group[g] != 2 or m.geom_type[g] != mj.mjtGeom.mjGEOM_MESH:
                continue
            k = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[k]:m.mesh_vertadr[k] + m.mesh_vertnum[k]]
            pts.append(v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])
        return np.concatenate(pts)


def composite(rgb, sz, alpha, layer):
    """Per pixel: robot where its z-depth is nearer than the splat's (splat alpha<0.5 = empty).
    rgb is modified in place. Returns (rgb, depth for viser, robot-visible mask)."""
    depth = np.where(alpha > 0.5, sz, NO_DEPTH).astype(np.float32)
    mask = np.zeros(depth.shape, bool)
    mrgb, mz, (y0, x0) = layer
    if mz.size:
        win = (slice(y0, y0 + mz.shape[0]), slice(x0, x0 + mz.shape[1]))
        m = mz < depth[win]
        rgb[win][m] = mrgb[m]
        depth[win][m] = mz[m]
        mask[win] = m
    return rgb, depth, mask


def look_at_c2w(eye, target):
    """OpenCV-convention camera-to-world (x right, y down, z forward), world +z up."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    f = target - eye
    f /= np.linalg.norm(f)
    r = np.cross(f, [0.0, 0.0, 1.0])
    r /= np.linalg.norm(r)
    c2w = np.eye(4)
    c2w[:3, 0], c2w[:3, 1], c2w[:3, 2], c2w[:3, 3] = r, np.cross(f, r), f, eye
    return c2w


def mat_to_wxyz(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return np.array([w, x, y, z])


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.pose = None          # (p, R) world<-base
        self.joints = None        # 16 joint angles, SIM_JOINTS order
        self.cam_img = None
        self.det_img, self.det_t = None, 0.0   # /detections/image (annotated camera) + arrival time
        self.objects = []                      # confirmed object map from detector_node.py
        self.explore = None       # {"tour": [...], "i": int, "skipped": int, "stop": bool, "gh": goal handle}
        self.est = None           # Nav2's own belief: map->base_link (x, y)
        self.loc_err = None       # |truth - belief| (sim only: truth exists here)
        self.cmd = (0.0, 0.0)     # last /cmd_vel (vx, wz)
        self.status = "waiting for /odom_true (is the sim running?)"


def start_ros(state):
    """rclpy in a background thread: robot pose + camera in, arm/goal out."""
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    from m20_msgs.srv import SetMode
    from m20_msgs.action import RunMission
    from m20_msgs.msg import MissionItem
    from drdds.msg import JointsData

    rclpy.init()
    node = Node("m20_splat_viewer")

    def joints(msg):
        q = np.array([msg.data.joints_data[i].position for i in range(16)])
        with state.lock:
            state.joints = q

    node.create_subscription(JointsData, "/JOINTS_DATA", joints, 10)

    def odom(msg):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        with state.lock:
            state.pose = (np.array([p.x, p.y, p.z]), quat_to_mat(o.x, o.y, o.z, o.w))

    def image(msg):
        if msg.encoding == "rgb8":
            with state.lock:
                state.cam_img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3)

    node.create_subscription(Odometry, "/odom_true", odom, qos_profile_sensor_data)
    node.create_subscription(Image, "/camera/image_raw", image, qos_profile_sensor_data)

    def det_image(msg):
        if msg.encoding == "rgb8":
            with state.lock:
                state.det_img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3)
                state.det_t = time.time()

    def det_objects(msg):
        import json
        try:
            state.objects = json.loads(msg.data).get("objects", [])
        except ValueError:
            pass

    from std_msgs.msg import String
    node.create_subscription(Image, "/detections/image", det_image, qos_profile_sensor_data)
    node.create_subscription(String, "/detections/objects", det_objects, 10)
    from geometry_msgs.msg import Twist
    node.create_subscription(Twist, "/cmd_vel", lambda m: setattr(state, "cmd", (m.linear.x, m.angular.z)), 10)

    # Nav2's own pose belief (map->base_link). The explorer steers by THIS, not by sim
    # truth, so it can never silently disagree with Nav2; truth is used only to WATCH
    # localization (sim-only safety net) and for the black-box log.
    from tf2_ros import Buffer, TransformListener
    tf_buf = Buffer()
    node._tf_listener = TransformListener(tf_buf, node)
    rec = {"f": None}

    def localize():
        try:
            tr = tf_buf.lookup_transform("map", "base_link", rclpy.time.Time()).transform.translation
        except Exception:
            return
        with state.lock:
            state.est = (tr.x, tr.y)
            pose = state.pose
        if pose is not None:
            state.loc_err = math.dist(pose[0][:2], state.est)
        # black box while exploring: truth, belief, command, attitude
        if state.explore is not None and pose is not None:
            if rec["f"] is None:
                rec["f"] = open(LOG_PATH, "a")
            R = pose[1]
            roll, pitch = math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1.0, min(1.0, R[2, 0])))
            rec["f"].write(f"{time.time():.2f},{pose[0][0]:.3f},{pose[0][1]:.3f},{math.atan2(R[1, 0], R[0, 0]):.3f},"
                           f"{roll:.3f},{pitch:.3f},{pose[0][2]:.3f},{tr.x:.3f},{tr.y:.3f},"
                           f"{state.cmd[0]:.3f},{state.cmd[1]:.3f}\n")
            rec["f"].flush()

    node.create_timer(0.1, localize)
    set_mode = node.create_client(SetMode, "/m20/set_mode")
    mission = ActionClient(node, RunMission, "/m20/mission/run")

    def arm(on_armed=None):
        """ASSISTED (2). The commander refuses ESTOP -> ASSISTED (e.g. latched after a
        tip-over); like bringup's standup path, go through IDLE (0) first and retry.
        on_armed(ok) fires once the outcome is known."""
        if not set_mode.wait_for_service(timeout_sec=2.0):
            state.status = "arm failed: /m20/set_mode not available"
            on_armed and on_armed(False); return

        def call(mode, cb):
            req = SetMode.Request(); req.mode = mode; req.requester = "splat_viewer"
            set_mode.call_async(req).add_done_callback(lambda f: cb(f.result()))

        def done(r):
            state.status = f"arm: accepted={r.accepted} ({r.reason})"
            on_armed and on_armed(r.accepted)

        def first(r):
            if r.accepted:
                done(r)
            else:   # e.g. latched ESTOP: IDLE, then ASSISTED again
                node.get_logger().warn(f"arm: ASSISTED refused ({r.reason}); going via IDLE")
                call(0, lambda _r: call(2, done))
        call(2, first)

    def send(x, y, label, on_done, yaw=0.0):
        """One-item RunMission (same action as tools/dev/m20). on_done(success, detail)."""
        if not mission.wait_for_server(timeout_sec=2.0):
            on_done(False, "mission server not running"); return
        goal = RunMission.Goal()
        it = MissionItem(); it.type = MissionItem.TYPE_GOTO; it.label = label
        it.goal.header.frame_id = "map"
        it.goal.pose.position.x, it.goal.pose.position.y = float(x), float(y)
        it.goal.pose.orientation.z, it.goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        goal.items.append(it)

        def accepted(f):
            gh = f.result()
            if not gh.accepted:
                on_done(False, "REJECTED — press Arm first"); return
            if state.explore is not None:
                state.explore["gh"] = gh
            gh.get_result_async().add_done_callback(
                lambda r: on_done(r.result().result.success, r.result().result.detail))
        mission.send_goal_async(goal).add_done_callback(accepted)

    def goto(x, y):
        state.status = f"navigating to ({x:.1f}, {y:.1f})"
        send(x, y, "viewer_goal", lambda ok, detail: setattr(
            state, "status", f"goal ({x:.1f}, {y:.1f}): success={ok} — {detail}"))

    def explore(tour):
        """Drive the scene's exploration tour one goal at a time; a stop the planner can't
        reach is skipped (logged), not fatal — the tour keeps going. Failures that come back
        instantly mean something else is fighting for Nav2 (another explorer/goal sender
        preempting us): 3 in a row halts the tour instead of burning through every stop."""
        if state.explore is not None:
            state.status = "already exploring — 'Stop exploring' first"; return
        if state.loc_err is None or state.loc_err > LOC_ERR_START:
            err = "unknown" if state.loc_err is None else f"{state.loc_err:.2f} m"
            state.status = (f"not starting: localization error {err} (> {LOC_ERR_START} m) — "
                            "restart the demo for a fresh map")
            return
        state.explore = e0 = {"tour": tour, "i": 0, "skipped": 0, "stop": False, "gh": None,
                              "sent": 0.0, "fast_fail": 0, "leg": 0, "target": None}
        lock = threading.Lock()

        def step(ok=None, detail="", leg=None):
            with lock:
                _step(ok, detail, leg)

        def _step(ok, detail, leg):
            e = state.explore
            if e is None or (leg is not None and leg != e["leg"]):
                return          # result of a leg we already passed through — ignore
            e["leg"] += 1       # invalidate the current leg's late result
            if ok is False:
                e["skipped"] += 1
                node.get_logger().warn(f"explore: stop {e['i']} {tuple(e['tour'][e['i'] - 1])} skipped: {detail}")
                e["fast_fail"] = e["fast_fail"] + 1 if time.time() - e["sent"] < 2.0 else 0
                if e["fast_fail"] >= 3:
                    state.status = ("explore HALTED: Nav2 rejected 3 goals instantly — is another "
                                    "client sending goals? (only one explorer at a time)")
                    state.explore = None
                    return
                # complete the PLANNED path: retry a failed stop before giving up on it
                if not e["stop"] and e.get("tries", 0) < STOP_RETRIES and e.get("cur"):
                    e["tries"] = e.get("tries", 0) + 1
                    e["skipped"] -= 1
                    x, y, yaw = e["cur"]
                    state.status = (f"exploring: retrying stop {e['i']}/{len(e['tour'])} "
                                    f"(attempt {e['tries'] + 1}/{STOP_RETRIES + 1})")
                    node.get_logger().warn(state.status)
                    e["sent"] = time.time()
                    e["target"] = (x, y)
                    leg = e["leg"]
                    send(x, y, f"explore_{e['i']}_retry{e['tries']}",
                         lambda ok_, d_, leg=leg: step(ok_, d_, leg), yaw)
                    return
            elif ok:
                e["fast_fail"] = 0
            if e["stop"] or e["i"] >= len(e["tour"]):
                state.status = (f"explore {'stopped' if e['stop'] else 'DONE'}: "
                                f"{e['i'] - e['skipped']}/{len(e['tour'])} stops reached, {e['skipped']} skipped")
                node.get_logger().info(state.status)
                state.explore = None
                return
            x, y = e["tour"][e["i"]]
            # arrive already facing the NEXT stop: a fixed goal yaw made the robot spin in
            # place at every stop (turn to goal yaw, then turn back toward the next leg)
            nx, ny = e["tour"][e["i"] + 1] if e["i"] + 1 < len(e["tour"]) else (x + 1.0, y)
            yaw = math.atan2(ny - y, nx - x)
            e["i"] += 1
            state.status = f"exploring: stop {e['i']}/{len(e['tour'])} -> ({x:.1f}, {y:.1f}), {e['skipped']} skipped"
            node.get_logger().info(state.status)
            e["sent"] = time.time()
            e["target"] = (x, y)
            e["cur"], e["tries"] = (x, y, yaw), 0
            leg = e["leg"]
            send(x, y, f"explore_{e['i']}", lambda ok_, d_, leg=leg: step(ok_, d_, leg), yaw)

        def turn_at(i):
            """Heading change at tour stop i (0-based), incoming vs outgoing leg, radians."""
            t_ = e0["tour"]
            prev = t_[i - 1] if i > 0 else (0.0, 0.0)
            a_in = math.atan2(t_[i][1] - prev[1], t_[i][0] - prev[0])
            a_out = math.atan2(t_[i + 1][1] - t_[i][1], t_[i + 1][0] - t_[i][0])
            return abs(math.remainder(a_out - a_in, 2 * math.pi))

        def pass_through():
            """Don't brake at intermediate stops: once within PASS_RADIUS of the current stop,
            preempt with the next goal (the final stop, home, is driven to normally).
            ONLY for gentle turns: a sharp turn taken at speed is turning-while-driving —
            the M20's known tip mode (it flipped at a wall corner doing exactly that) — so
            sharp stops are driven to normally and DWB turns in place before the next leg."""
            bad_since = None
            best_d, best_t, best_tgt = None, time.time(), None
            while state.explore is e0:
                est, err = state.est, state.loc_err
                # STALL RESCUE (progress-based): near a stop, DWB can park just outside goal
                # tolerance (70-98 s dead-stops) or wiggle in place by a wall pocket (56 s of
                # +-wz flips) — measured 2026-10-05. Tour stops are waypoints, not docking
                # targets: within STALL_RADIUS and no >=10 cm of progress toward the stop for
                # STALL_SECS -> count it as reached and move on.
                tgt_ = e0["target"]
                if est is not None and tgt_ is not None:
                    d_ = math.dist(est, tgt_)
                    if best_tgt != tgt_ or best_d is None or d_ < best_d - 0.10:
                        best_d, best_t, best_tgt = d_, time.time(), tgt_
                    elif d_ < STALL_RADIUS and time.time() - best_t > STALL_SECS:
                        node.get_logger().warn(f"explore: no progress {d_:.2f} m from stop "
                                               f"{e0['i']} {tgt_} for {STALL_SECS:.0f} s — counting it as reached")
                        e0["target"] = None
                        best_tgt = None
                        step(True, "close enough (stall rescue)", e0["leg"])
                        continue
                # WATCHDOG: Nav2 driving on a wrong pose is how the robot fell (17 m off after
                # an hour of laps). Sustained disagreement with sim truth -> halt, don't drive blind.
                if err is not None and err > LOC_ERR_MAX:
                    bad_since = bad_since or time.time()
                    if time.time() - bad_since > 2.0:
                        node.get_logger().error(f"explore HALTED: localization error {err:.2f} m")
                        e0["stop"] = True
                        if e0.get("gh") is not None:
                            e0["gh"].cancel_goal_async()
                        state.explore = None
                        state.status = (f"HALTED: localization lost ({err:.1f} m off) — robot stopped. "
                                        "Restart the demo for a fresh map.")
                        return
                else:
                    bad_since = None
                tgt = e0["target"]
                cur = e0["i"] - 1          # index of the stop currently being driven to
                if est is not None and tgt is not None and e0["i"] < len(e0["tour"]) \
                        and turn_at(cur) < PASS_MAX_TURN \
                        and math.dist(est, tgt) < PASS_RADIUS:
                    node.get_logger().info(f"explore: passed stop {e0['i']} {tgt}")
                    e0["target"] = None
                    step(True, "passed", e0["leg"])
                time.sleep(0.05)
        def armed(ok):
            if ok:
                step()
                threading.Thread(target=pass_through, daemon=True).start()
            else:
                state.status = f"explore not started — could not arm ({state.status})"
                state.explore = None
        arm(armed)   # start the tour only once ASSISTED is confirmed

    def stop():
        e = state.explore
        if e is not None:
            e["stop"] = True
            if e.get("gh") is not None:
                e["gh"].cancel_goal_async()
            state.status = "stopping explore..."

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    return arm, goto, explore, stop


def splat_renderer(g, deg, dev):
    def render(c2w, fov_y, w, h):
        """gsplat pinhole render -> rgb uint8, expected z-depth float32, alpha float32."""
        view = torch.tensor(np.linalg.inv(c2w), dtype=torch.float32, device=dev)[None]
        fy = (h / 2) / math.tan(fov_y / 2)
        K = torch.tensor([[fy, 0, w / 2], [0, fy, h / 2], [0, 0, 1]], dtype=torch.float32, device=dev)[None]
        with torch.no_grad():
            out, alpha, _ = rasterization(g["means"], g["quats"], g["scales"], g["opacities"], g["colors"],
                                          view, K, w, h, sh_degree=deg, near_plane=NEAR, far_plane=FAR,
                                          render_mode="RGB+ED")
        rgb = (out[0, ..., :3].clamp(0, 1) * 255).to(torch.uint8).cpu().numpy()
        depth = out[0, ..., 3].cpu().numpy()
        return rgb, depth, alpha[0, ..., 0].cpu().numpy()
    return render


def load_world():
    import yaml
    dev = torch.device("cuda")
    with open(SCENE) as f:
        scene = yaml.safe_load(f)
    g, deg = load_splat(PLY, scene["splat_to_world"], dev)
    return scene, g, splat_renderer(g, deg, dev)


def load_mj_robot():
    if not os.path.exists(MJCF):
        print(f"[splat_viewer] {MJCF} not found — MuJoCo composite off (URDF mesh only)", flush=True)
        return None
    try:
        mjr = MjRobot(MJCF)
    except Exception as e:   # no EGL / GL: keep the viewer usable with the URDF mesh
        print(f"[splat_viewer] MuJoCo offscreen renderer unavailable ({type(e).__name__}: {e}) "
              f"— URDF mesh only", flush=True)
        return None
    print(f"[splat_viewer] MuJoCo composite: {MJCF} on '{mjr.gl_name}' "
          f"(MUJOCO_GL={os.environ.get('MUJOCO_GL')}, reversed-Z={mjr.reversed_z})", flush=True)
    return mjr


def _slerp(q0, q1, a):
    """Shortest-arc slerp between unit quaternions (wxyz)."""
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + a * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(min(1.0, d))
    return (math.sin((1 - a) * th) * q0 + math.sin(a * th) * q1) / math.sin(th)


class LapRecording:
    """One explore lap, recorded for replay: robot pose + joints at 20 Hz (interpolated on
    playback: lerp position/joints, slerp orientation -> smooth at any speed), the camera frame
    the panel showed (JPEG, 5 Hz), and the object map / tour progress whenever they change.
    Every stream is timestamped, so replay shows exactly what was on screen at lap time tau.
    Saved as a pickle in LAPS_DIR so a recorded lap survives viewer restarts."""
    POSE_DT, IMG_DT = 0.05, 0.2

    def __init__(self):
        self.t, self.p, self.quat, self.q = [], [], [], []
        self.img_t, self.img = [], []
        self.obj_t, self.obj = [], []
        self.cur_t, self.cur = [], []
        self.meta = {}

    def add(self, t, pose, q, shown, objects, cur, cv2):
        if not self.t or t - self.t[-1] >= self.POSE_DT:
            self.t.append(t)
            self.p.append(np.asarray(pose[0], float).copy())
            self.quat.append(np.asarray(mat_to_wxyz(pose[1]), float))
            self.q.append(np.zeros(16) if q is None else np.asarray(q, float).copy())
        if shown is not None and (not self.img_t or t - self.img_t[-1] >= self.IMG_DT):
            ok, buf = cv2.imencode(".jpg", cv2.cvtColor(shown, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                self.img_t.append(t); self.img.append(buf.tobytes())
        if not self.obj or objects is not self.obj[-1]:
            self.obj_t.append(t); self.obj.append(objects)
        if not self.cur or cur != self.cur[-1]:
            self.cur_t.append(t); self.cur.append(cur)

    @property
    def duration(self):
        return (self.t[-1] - self.t[0]) if len(self.t) > 1 else 0.0

    def finalize(self):
        self.t_arr = np.array(self.t)
        self.p_arr, self.quat_arr, self.q_arr = np.array(self.p), np.array(self.quat), np.array(self.q)
        self.img_t_arr, self.obj_t_arr, self.cur_t_arr = map(np.array, (self.img_t, self.obj_t, self.cur_t))
        return self

    @staticmethod
    def _last(ts, vals, t, default=None):
        i = int(np.searchsorted(ts, t, side="right")) - 1
        return vals[i] if 0 <= i < len(vals) else default

    def sample(self, tau):
        """Lap time tau (s from start) -> (pose (p, R), joints, jpeg|None, objects, stop index, trail)."""
        t = self.t_arr[0] + min(max(tau, 0.0), self.duration)
        i = int(np.clip(np.searchsorted(self.t_arr, t), 1, len(self.t_arr) - 1))
        a = float(np.clip((t - self.t_arr[i - 1]) / max(self.t_arr[i] - self.t_arr[i - 1], 1e-6), 0, 1))
        p = (1 - a) * self.p_arr[i - 1] + a * self.p_arr[i]
        w, x, y, z = _slerp(self.quat_arr[i - 1], self.quat_arr[i], a)
        q = (1 - a) * self.q_arr[i - 1] + a * self.q_arr[i]
        trail = [tuple(v[:2]) for v in self.p_arr[:i:2]] + [tuple(p[:2])]
        return ((p, quat_to_mat(x, y, z, w)), q, self._last(self.img_t_arr, self.img, t),
                self._last(self.obj_t_arr, self.obj, t, []), self._last(self.cur_t_arr, self.cur, t), trail)

    def save(self, directory):
        import pickle
        os.makedirs(directory, exist_ok=True)
        path = os.path.join(directory, time.strftime("lap_%Y%m%d_%H%M%S.pkl"))
        with open(path, "wb") as f:
            pickle.dump({k: getattr(self, k) for k in ("t", "p", "quat", "q", "img_t", "img", "obj_t", "obj",
                                                       "cur_t", "cur", "meta")}, f)
        return path

    @classmethod
    def load_latest(cls, directory):
        import glob, pickle
        files = sorted(glob.glob(os.path.join(directory, "lap_*.pkl")))
        if not files:
            return None, None
        r = cls()
        with open(files[-1], "rb") as f:
            for k, v in pickle.load(f).items():
                setattr(r, k, v)
        return r.finalize(), files[-1]


from det_draw import class_colour, draw_box, label_text   # same style as the robot camera panel


def draw_objects(rgb, depth, c2w, fov_y, objects, pose, min_score=0.0, box_scale=1.0, style_scale=1.0):
    """Draw the detector's confirmed objects INTO a main-view frame, styled exactly like the
    robot camera's boxes: each object's 3D box (centre + size from the depth pixels) is
    projected through this view's pinhole camera; its screen-space bounding rectangle gets
    the shared det_draw box + "class score distance" tag. Hidden when the splat (or robot)
    is clearly in front of the object at its centre pixel. rgb is modified in place.
    min_score: hide objects below this confidence; box_scale: shrink the 3D extent before
    projecting (the projected envelope of a 3D box reads larger than the object);
    style_scale: line/tag size relative to the camera panel's style."""
    h, w = rgb.shape[:2]
    fy = (h / 2) / math.tan(fov_y / 2)
    w2c = np.linalg.inv(c2w)
    robot_xy = None if pose is None else pose[0][:2]
    items = []
    for o in objects:
        if o.get("score", 0.0) < min_score:
            continue
        c = np.asarray(o["xyz"], float)
        s = box_scale * np.asarray(o.get("size") or (0.4, 0.4, 0.4), float)
        corners = c + 0.5 * s * np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        pc = corners @ w2c[:3, :3].T + w2c[:3, 3]
        if (pc[:, 2] < 0.15).any():                       # (partly) behind the camera
            continue
        u, v = fy * pc[:, 0] / pc[:, 2] + w / 2, fy * pc[:, 1] / pc[:, 2] + h / 2
        x0, x1, y0, y1 = max(u.min(), 0), min(u.max(), w - 1), max(v.min(), 0), min(v.max(), h - 1)
        min_px = 14 * w / 640.0                           # tiny far boxes just stack into clutter
        if x1 - x0 < min_px or y1 - y0 < min_px or (x1 - x0) * (y1 - y0) > 0.8 * w * h:
            continue
        cc = w2c[:3, :3] @ c + w2c[:3, 3]
        if cc[2] > 12.0:                                  # beyond 12 m: drop (like a camera's range)
            continue
        uc, vc = int(fy * cc[0] / cc[2] + w / 2), int(fy * cc[1] / cc[2] + h / 2)
        if 0 <= uc < w and 0 <= vc < h:                   # occlusion at the centre pixel
            d_front = float(depth[max(vc - 2, 0):vc + 3, max(uc - 2, 0):uc + 3].min())
            if cc[2] - 0.5 * float(s.max()) > d_front + 0.3:
                continue
        dist = None if robot_xy is None else math.dist(robot_xy, c[:2])
        items.append((cc[2], (x0, y0, x1, y1), label_text(o["cls"], o.get("score", 0.0), dist), class_colour(o["cls"])))
    for _, box, text, col in sorted(items, key=lambda it: -it[0]):   # far first, near on top
        draw_box(rgb, box, text, col, scale=style_scale * w / 640.0)


class MiniMap:
    """Live top-down mini-map on the photoreal map rendered by tools/splat/render_topdown.py
    (scene yaml `topdown:` block): tour route + numbered stops, driven trail, and the robot
    drawn to scale (M20 footprint + heading). World (= sim = Nav2 map) metres -> pixels is
    the block's affine map: u = (x - x0) * ppm, v = (y1 - y) * ppm."""
    FOOT = (0.86, 0.56)                       # M20 footprint incl. wheels [m]

    def __init__(self, scene, scene_dir):
        import cv2
        self.cv2 = cv2
        td = scene["topdown"]
        img = cv2.imread(os.path.join(scene_dir, td["file"]), cv2.IMREAD_COLOR)
        if img is None:
            raise FileNotFoundError(td["file"])
        self.base = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.ppm, self.x0, self.y1 = float(td["px_per_m"]), float(td["x0"]), float(td["y1"])
        self.tour = [tuple(p) for p in scene.get("tour", [])]
        self.trail = []

    def px(self, x, y):
        return int(round((x - self.x0) * self.ppm)), int(round((self.y1 - y) * self.ppm))

    def add_trail(self, x, y):
        if not self.trail or math.dist(self.trail[-1], (x, y)) > 0.05:
            self.trail.append((x, y))

    def draw(self, pose=None, current=None, out_w=420, objects=()):
        """pose: (p, R) or None; current: 1-based index of the stop being driven to (None = idle);
        objects: confirmed detections [{cls, xyz, ...}] drawn as labelled diamonds."""
        cv2, img = self.cv2, self.base.copy()
        for o in objects:
            u, v = self.px(o["xyz"][0], o["xyz"][1])
            c = class_colour(o["cls"])
            pts = np.array([(u, v - 9), (u + 9, v), (u, v + 9), (u - 9, v)], np.int32)
            cv2.fillPoly(img, [pts], c, cv2.LINE_AA)
            cv2.polylines(img, [pts], True, (0, 0, 0), 1, cv2.LINE_AA)
            cv2.putText(img, o["cls"], (u + 11, v + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, o["cls"], (u + 11, v + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 1, cv2.LINE_AA)
        route = [self.px(0.0, 0.0)] + [self.px(*s) for s in self.tour]
        cv2.polylines(img, [np.array(route, np.int32)], False, (0, 200, 255), 2, cv2.LINE_AA)
        if len(self.trail) > 1:
            cv2.polylines(img, [np.array([self.px(*p) for p in self.trail], np.int32)], False,
                          (255, 140, 0), 4, cv2.LINE_AA)
        for k, s in enumerate(self.tour[:-1], 1):          # last stop = home (drawn as spawn)
            u, v = self.px(*s)
            done = current is not None and k < current
            col = (60, 220, 90) if done else (0, 200, 255)
            cv2.circle(img, (u, v), 9, col, -1 if done else 2, cv2.LINE_AA)
            if current == k:
                cv2.circle(img, (u, v), 16, (255, 230, 0), 3, cv2.LINE_AA)
            cv2.putText(img, str(k), (u + 11, v - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(img, str(k), (u + 11, v - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        hu, hv = self.px(0.0, 0.0)
        cv2.drawMarker(img, (hu, hv), (255, 255, 255), cv2.MARKER_STAR, 22, 2, cv2.LINE_AA)
        if pose is not None:
            p, R = pose
            yaw = math.atan2(R[1, 0], R[0, 0])
            c, s = math.cos(yaw), math.sin(yaw)
            L, W = self.FOOT
            corners = [(L / 2, W / 2), (L / 2, -W / 2), (-L / 2, -W / 2), (-L / 2, W / 2)]
            poly = [self.px(p[0] + c * a - s * b, p[1] + s * a + c * b) for a, b in corners]
            cv2.fillPoly(img, [np.array(poly, np.int32)], (230, 40, 40), cv2.LINE_AA)
            cv2.polylines(img, [np.array(poly, np.int32)], True, (255, 255, 255), 2, cv2.LINE_AA)
            tip = self.px(p[0] + c * 0.9, p[1] + s * 0.9)
            cv2.arrowedLine(img, self.px(p[0], p[1]), tip, (255, 255, 255), 3, cv2.LINE_AA, tipLength=0.35)
        h, w = img.shape[:2]
        return cv2.resize(img, (out_w, int(h * out_w / w)), interpolation=cv2.INTER_AREA) if out_w else img


def load_minimap(scene):
    if "topdown" not in scene:
        print("[splat_viewer] no topdown: block in the scene yaml — mini-map off "
              "(run tools/splat/render_topdown.py)", flush=True)
        return None
    try:
        mm = MiniMap(scene, os.path.dirname(SCENE))
        print(f"[splat_viewer] mini-map: {scene['topdown']['file']} ({mm.base.shape[1]}x{mm.base.shape[0]}, "
              f"cut {scene['topdown']['cut_z']} m)", flush=True)
        return mm
    except Exception as ex:
        print(f"[splat_viewer] mini-map unavailable: {ex}", flush=True)
        return None


def main():
    import viser
    import cv2                       # lap recorder (JPEG frames) + replay decode
    scene, g, render = load_world()
    print(f"[splat_viewer] {len(g['means'])} Gaussians loaded; open http://localhost:{PORT}", flush=True)
    mjr = load_mj_robot()

    tour = [tuple(p) for p in scene.get("tour", [])]
    minimap = load_minimap(scene)
    state = State()
    arm, goto, explore, stop = start_ros(state)

    server = viser.ViserServer(host="0.0.0.0", port=PORT)
    server.scene.set_up_direction("+z")
    robot = server.scene.add_frame("/robot", show_axes=False, position=(0, 0, -10))
    urdf, urdf_idx = None, None
    if os.path.exists(URDF):
        from viser.extras import ViserUrdf
        urdf = ViserUrdf(server, urdf_or_path=__import__("pathlib").Path(URDF), root_node_name="/robot")
        names = list(urdf.get_actuated_joint_names())
        urdf_idx = [SIM_JOINTS.index(n) for n in names]       # sim order -> URDF order
        print(f"[splat_viewer] robot: {URDF} ({len(names)} actuated joints)", flush=True)
    else:
        server.scene.add_box("/robot/body", dimensions=BODY, color=(255, 140, 0))
        print(f"[splat_viewer] {URDF} not found — drawing a box", flush=True)
    if tour:   # exploration stops as small markers on the floor
        server.scene.add_point_cloud("/tour", points=np.array([[x, y, 0.05] for x, y in tour]),
                                     colors=(0, 220, 255), point_size=0.12)

    server.gui.add_markdown("**M20 splat sim** — photoreal world (gsplat) + robot at ground truth")
    status = server.gui.add_text("Status", initial_value=state.status, disabled=True)
    follow = server.gui.add_checkbox("Follow robot (chase cam)", initial_value=True)
    quality = server.gui.add_slider("Render width", min=480, max=1920, step=160, initial_value=VIEW_W)
    modes = ROBOT_MODES if mjr is not None else ROBOT_MODES[1:]
    robot_mode = server.gui.add_dropdown("Robot", modes, initial_value=modes[
        -1 if os.environ.get("VIEW_ROBOT", "mujoco") == "urdf" else 0])
    perf = server.gui.add_text("Render", initial_value="", disabled=True)
    with server.gui.add_folder("Explore"):
        server.gui.add_markdown(f"Tour: {max(len(tour) - 1, 0)} stops covering the floor, then home.")
        server.gui.add_button("Explore the world").on_click(lambda _: explore(tour) if tour else None)
        server.gui.add_button("Stop exploring").on_click(lambda _: stop())
    with server.gui.add_folder("Drive"):
        gx = server.gui.add_number("Goal x [m]", initial_value=0.0, step=0.5)
        gy = server.gui.add_number("Goal y [m]", initial_value=-4.0, step=0.5)
        server.gui.add_button("Arm (ASSISTED)").on_click(lambda _: arm())
        server.gui.add_button("Go to goal").on_click(lambda _: goto(gx.value, gy.value))
    map_panel = None
    if minimap is not None:
        with server.gui.add_folder("Top-down map (photoreal)"):
            map_panel = server.gui.add_image(minimap.draw(), label="robot · trail · tour stops")
    with server.gui.add_folder("Replay"):
        replay_info = server.gui.add_text("Recording", initial_value="no lap recorded yet", disabled=True)
        replay_speed = server.gui.add_dropdown("Speed", ("1x", "2x", "4x", "8x"), initial_value="2x")
        replay_loop = server.gui.add_checkbox("Loop", initial_value=False)
        replay_pos = server.gui.add_slider("Position %", min=0, max=100, step=0.5, initial_value=0)
        replay_btn = server.gui.add_button("Replay last lap")
        replay_stop = server.gui.add_button("Stop replay")
    with server.gui.add_folder("Object detection"):
        show_det = server.gui.add_checkbox("Show detections (camera + map)", initial_value=True)
        show_3d = server.gui.add_checkbox("3D boxes in main view", initial_value=True)
        min_conf = server.gui.add_slider("Min confidence", min=0.3, max=0.9, step=0.05,
                                         initial_value=float(os.environ.get("VIEW_MIN_CONF", 0.5)))
        box_size = server.gui.add_slider("Box size", min=0.4, max=1.0, step=0.05, initial_value=0.7)
        det_summary = server.gui.add_text("Object map", initial_value="waiting for detector...", disabled=True)
    with server.gui.add_folder("Robot camera (photoreal)"):
        cam_panel = server.gui.add_image(np.zeros((240, 320, 3), np.uint8), label="/camera/image_raw")

    @server.on_client_connect
    def _(client):
        client.camera.position = (-3.0, 0.0, 2.5)
        client.camera.look_at = (0.0, 0.0, 0.3)

    last_img_t = last_log_t = time.time()
    ema = {"frame": 0.0, "splat": 0.0, "robot": 0.0, "send": 0.0}

    def tick(k, dt):
        ema[k] = dt if ema[k] == 0.0 else 0.9 * ema[k] + 0.1 * dt

    auto_started = False   # VIEW_AUTO_EXPLORE: exactly one lap per viewer start
    last_map_t, lap_ref, map_current = 0.0, None, None
    last_obj_t = 0.0                        # detector object-map summary + 3D boxes refresh
    box_handles = {}                        # object id -> ((label, box) handles, geometry key)
    # lap recorder (while exploring) + replay. The newest saved lap is loaded at start.
    rec, rec_ref = None, None
    last_lap, lap_file = LapRecording.load_latest(LAPS_DIR)
    if last_lap is not None:
        replay_info.value = f"{os.path.basename(lap_file)}: {last_lap.duration:.0f} s lap (saved)"
    rp = {"on": False, "start": 0.0, "offset": 0.0, "pos_set": False}

    def replay_tau(now):
        return rp["offset"] + (now - rp["start"]) * float(replay_speed.value.rstrip("x"))

    def start_replay(_, at=0.0):
        if state.explore is not None:
            state.status = "replay: wait for the current lap to finish"; return
        if last_lap is None or last_lap.duration <= 0:
            state.status = "replay: no lap recorded yet — run 'Explore the world' first"; return
        rp["offset"], rp["start"], rp["on"] = at, time.time(), True

    def stop_replay(_):
        if rp["on"]:
            rp["on"] = False
            state.status = "replay stopped"
            if minimap is not None:
                minimap.trail = []

    @replay_speed.on_update
    def _(_):                         # keep the lap position when the speed changes mid-replay
        if rp["on"]:
            now = time.time()
            rp["offset"], rp["start"] = replay_tau(now - 1e-9), now

    @replay_pos.on_update
    def _(_):                         # user scrub -> seek (ignore our own progress updates)
        if rp["pos_set"] or last_lap is None:
            return
        start_replay(None, at=replay_pos.value / 100.0 * last_lap.duration)

    replay_btn.on_click(start_replay)
    replay_stop.on_click(stop_replay)
    while True:
        t0 = time.time()
        with state.lock:
            pose, cam_img, q = state.pose, state.cam_img, state.joints
        det_fresh = state.det_img is not None and t0 - state.det_t < 1.5
        live_objects, live_det = state.objects, (state.det_img if det_fresh else None)
        # ---- RECORD the lap: pose + joints (20 Hz), the camera frame the panel shows, map state
        e_ = state.explore
        if e_ is not None and not rp["on"]:
            if rec_ref is not e_:
                rec, rec_ref = LapRecording(), e_
            if pose is not None:
                shown = live_det if (show_det.value and live_det is not None) else cam_img
                rec.add(t0, pose, q, shown, live_objects, max(e_["i"], 1), cv2)
        elif e_ is None and rec_ref is not None:          # lap ended: keep + save it
            rec_ref = None
            if rec is not None and rec.duration > 5:
                rec.meta = {"status": state.status, "scene": scene.get("name")}
                last_lap = rec.finalize()
                try:
                    lap_file = last_lap.save(LAPS_DIR)
                    replay_info.value = f"{os.path.basename(lap_file)}: {last_lap.duration:.0f} s lap (saved)"
                except OSError as ex:
                    replay_info.value = f"last lap: {last_lap.duration:.0f} s (not saved: {ex})"
        # ---- REPLAY: every view shows the recorded lap at lap time tau (smoothly interpolated)
        replay_frame = None
        if rp["on"]:
            tau, dur = replay_tau(t0), last_lap.duration
            if tau > dur:
                if replay_loop.value:
                    rp["offset"], rp["start"], tau = 0.0, t0, 0.0
                else:
                    rp["on"] = False
                    state.status = f"replay finished ({dur:.0f} s lap)"
            if rp["on"]:
                pose, q, jpg, r_objs, r_cur, r_trail = last_lap.sample(tau)
                replay_frame = (r_objs, r_cur, r_trail)
                if jpg is not None and jpg is not rp.get("jpg"):
                    rp["jpg"] = jpg
                    rp["img"] = cv2.cvtColor(cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR),
                                             cv2.COLOR_BGR2RGB)
                cam_img = rp.get("img", cam_img)
                spd = float(replay_speed.value.rstrip("x"))
                state.status = (f"REPLAY {spd:g}× {'(loop) ' if replay_loop.value else ''}— "
                                f"{tau / spd:.0f}/{dur / spd:.0f} s  (lap time {tau:.0f}/{dur:.0f} s)")
                if t0 - rp.get("pos_t", 0) > 0.5:       # progress slider, without triggering a seek
                    rp["pos_set"] = True
                    replay_pos.value = round(100.0 * tau / dur, 1)
                    rp["pos_set"], rp["pos_t"] = False, t0
        use_mj = mjr is not None and robot_mode.value == ROBOT_MODES[0]
        robot.visible = not use_mj          # composite replaces the URDF scene mesh
        if urdf is not None and q is not None and not use_mj:
            urdf.update_cfg(q[urdf_idx])
        if use_mj and pose is not None:
            mjr.set_state(pose[0], pose[1], q)
        if pose is not None:
            p, R = pose
            robot.position = tuple(p)
            robot.wxyz = tuple(mat_to_wxyz(R))
            if state.status.startswith("waiting"):
                state.status = "sim connected — 'Explore the world', or Arm + Go to goal"
            # demo mode: one lap, started as soon as SLAM gives a pose that agrees with truth
            if AUTO_EXPLORE and tour and not auto_started and state.explore is None \
                    and state.loc_err is not None and state.loc_err < LOC_ERR_START:
                auto_started = True
                explore(tour)
        status.value = state.status
        objects_now = replay_frame[0] if replay_frame else live_objects
        if t0 - last_img_t > 0.2 and (cam_img is not None or det_fresh):
            if replay_frame:          # the recorded frame (already annotated if detections were on)
                cam_panel.image = cam_img
            else:                     # annotated detector frame while detections are on and fresh
                cam_panel.image = live_det if (show_det.value and live_det is not None) else cam_img
            last_img_t = t0
        if t0 - last_obj_t > 0.5:                      # object map: summary + 3D boxes, 2 Hz
            objs = objects_now
            # 3D bounding boxes in the main view: class-coloured wireframe + label per confirmed
            # object at/above Min confidence, scaled by Box size. Viser depth-tests them against
            # the splat depth sent with each frame, so walls in front hide them. A box is rebuilt
            # only when its geometry (or the sliders) change.
            want = {o["id"]: o for o in objs if o.get("score", 0.0) >= min_conf.value} \
                if (show_3d.value and show_det.value) else {}
            for oid in list(box_handles):
                if oid not in want:
                    for h_ in box_handles.pop(oid)[0]:
                        h_.remove()
            for oid, o in want.items():
                size = tuple(box_size.value * v for v in (o.get("size") or (0.4, 0.4, 0.4)))
                key = (tuple(round(v, 1) for v in o["xyz"]), tuple(round(v, 2) for v in size))
                if oid in box_handles and box_handles[oid][1] == key:
                    continue
                for h_ in box_handles.pop(oid, ((), None))[0]:
                    h_.remove()
                bx = server.scene.add_box(f"/detections/{oid}", color=class_colour(o["cls"]),
                                          dimensions=size, wireframe=True, position=tuple(o["xyz"]))
                lb = server.scene.add_label(f"/detections/{oid}/label", text=o["cls"],
                                            position=(0.0, 0.0, size[2] / 2 + 0.12), depth_test=True)
                box_handles[oid] = ((lb, bx), key)      # child label first when removing
            by = {}
            for o in objs:
                by[o["cls"]] = by.get(o["cls"], 0) + 1
            det_summary.value = (f"{len(objs)} objects: " + ", ".join(f"{k} ×{v}" for k, v in
                                 sorted(by.items(), key=lambda kv: -kv[1]))) if objs else \
                ("no objects confirmed yet" if det_fresh else "waiting for detector...")
            last_obj_t = t0
        if map_panel is not None and t0 - last_map_t > 0.2:     # mini-map ~5 Hz
            if replay_frame:
                # trail = recorded poses up to the replay cursor; progress = recorded stop index
                objs_map, cur, minimap.trail = replay_frame
            else:
                e = state.explore
                if e is not None and lap_ref is not e:          # new lap -> fresh trail
                    minimap.trail.clear()
                    lap_ref = e
                if pose is not None and e is not None:
                    minimap.add_trail(pose[0][0], pose[0][1])
                if e is not None:
                    map_current = max(e["i"], 1)
                cur, objs_map = map_current, live_objects
            map_panel.image = minimap.draw(pose, cur, objects=[o for o in objs_map if o.get("score", 0) >= min_conf.value]
                                           if show_det.value else ())
            last_map_t = t0
        n_clients = 0
        for client in list(server.get_clients().values()):
            n_clients += 1
            cam = client.camera
            if follow.value and pose is not None:
                p, R = pose
                yaw = math.atan2(R[1, 0], R[0, 0])
                eye = p + np.array([-2.6 * math.cos(yaw), -2.6 * math.sin(yaw), 1.4])
                cam.position = tuple(eye)
                cam.look_at = tuple(p + np.array([0.0, 0.0, 0.2]))
            c2w = np.eye(4)
            c2w[:3, :3] = quat_to_mat(cam.wxyz[1], cam.wxyz[2], cam.wxyz[3], cam.wxyz[0])
            c2w[:3, 3] = cam.position
            w = int(quality.value); h = max(1, int(w / max(cam.aspect, 0.1)))
            t1 = time.time()
            rgb, depth, alpha = render(c2w, cam.fov, w, h)
            tick("splat", time.time() - t1)
            if use_mj and pose is not None:
                t1 = time.time()
                rgb, depth, _ = composite(rgb, depth, alpha, mjr.render(c2w, cam.fov, w, h))
                tick("robot", time.time() - t1)
            else:
                depth = np.where(alpha > 0.5, depth, NO_DEPTH).astype(np.float32)
            t1 = time.time()
            try:
                client.scene.set_background_image(rgb, format="jpeg", jpeg_quality=85, depth=depth)
            except TypeError:   # older viser without depth occlusion
                client.scene.set_background_image(rgb, format="jpeg", jpeg_quality=85)
            tick("send", time.time() - t1)
        if n_clients:
            tick("frame", time.time() - t0)
            perf.value = (f"{1 / max(ema['frame'], 1e-3):.1f} fps @ {int(quality.value)} px, "
                          f"{n_clients} client(s) — splat {ema['splat'] * 1e3:.0f} ms"
                          + (f", MuJoCo robot+composite {ema['robot'] * 1e3:.0f} ms" if use_mj else " (URDF mesh)")
                          + f", encode+send {ema['send'] * 1e3:.0f} ms")
            if t0 - last_log_t > 30:
                print(f"[splat_viewer] {perf.value}", flush=True)
                last_log_t = t0
        time.sleep(max(0.0, 1 / 20 - (time.time() - t0)))


def snapshot(eye, target, out, live=False):
    """Headless composite from a static camera: OUT.png + OUT_robot.png + alignment numbers."""
    scene, g, render = load_world()
    mjr = load_mj_robot()
    if mjr is None:
        raise SystemExit("MuJoCo renderer unavailable")
    pose, q = None, None
    if live:
        state = State()
        start_ros(state)
        t_end = time.time() + 8
        while time.time() < t_end and (state.pose is None or state.joints is None):
            time.sleep(0.1)
        pose, q = state.pose, state.joints
        print(f"[snapshot] live state: pose={'yes' if pose else 'NO'} joints={'yes' if q is not None else 'NO'}")
        if pose is not None:
            print(f"[snapshot] live base {np.round(pose[0], 3)}")
    if pose is None and os.environ.get("SNAP_BASE"):          # "X Y YAW_DEG": place the static robot
        x, y, yaw = (float(v) for v in os.environ["SNAP_BASE"].split())
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        pose = (np.array([x, y, 0.46]), np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]]))
    mjr.set_state(pose[0] if pose else None, pose[1] if pose else None, q)
    w, h, fov = 960, 540, math.radians(60)
    c2w = look_at_c2w(eye, target)
    rgb, sz, alpha = render(c2w, fov, w, h)
    t = time.time()
    for _ in range(10):
        layer = mjr.render(c2w, fov, w, h)
    t_mj = (time.time() - t) / 10
    t = time.time()
    for _ in range(10):
        render(c2w, fov, w, h)
    t_sp = (time.time() - t) / 10
    t = time.time()
    for _ in range(10):
        comp, cz, mask = composite(rgb.copy(), sz, alpha, layer)
    mrgb, mz = MjRobot.full(layer, w, h)
    t_cp = (time.time() - t) / 10
    write_png(out, comp)
    base = out[:-4] if out.endswith(".png") else out
    write_png(base + "_robot.png", np.where(np.isfinite(mz)[..., None], mrgb, np.uint8(255)))
    mm = load_minimap(scene)
    if mm is not None:   # full-resolution mini-map with the robot at the same pose
        write_png(base + "_minimap.png", mm.draw(pose, None, out_w=None))
        print(f"[snapshot] wrote {base}_minimap.png", flush=True)

    # --- alignment: project the model's own mesh vertices with the gsplat K / c2w
    fy = (h / 2) / math.tan(fov / 2)
    w2c = np.linalg.inv(c2w)
    P = mjr.mesh_points()
    pc = P @ w2c[:3, :3].T + w2c[:3, 3]
    pc = pc[pc[:, 2] > NEAR]
    u, v = fy * pc[:, 0] / pc[:, 2] + w / 2, fy * pc[:, 1] / pc[:, 2] + h / 2
    robot_px = np.isfinite(mz)
    print(f"[snapshot] GL: {mjr.gl_name}; per frame @ {w}x{h}: splat {t_sp * 1e3:.1f} ms, "
          f"MuJoCo {t_mj * 1e3:.1f} ms, composite {t_cp * 1e3:.1f} ms")
    print(f"[snapshot] robot pixels {robot_px.sum()}, visible in composite {mask.sum()} "
          f"({100 * mask.sum() / max(robot_px.sum(), 1):.0f}%; rest occluded by splat)")
    if robot_px.any():
        ys, xs = np.nonzero(robot_px)
        print(f"[snapshot] silhouette bbox  u[{xs.min()}, {xs.max() + 1}] v[{ys.min()}, {ys.max() + 1}]")
        print(f"[snapshot] projected verts  u[{u.min():.1f}, {u.max():.1f}] v[{v.min():.1f}, {v.max():.1f}]")
        inb = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        ui, vi = u[inb].astype(int), v[inb].astype(int)
        print(f"[snapshot] vertices landing on robot pixels: {100 * robot_px[vi, ui].mean():.1f}% of {inb.sum()}")
        # depth: front-most projected vertex z per pixel vs MuJoCo's z-depth at that pixel
        zbest = np.full((h, w), np.inf)
        np.minimum.at(zbest, (vi, ui), pc[inb, 2])
        sel = np.isfinite(zbest) & robot_px
        err = mz[sel] - zbest[sel]
        print(f"[snapshot] z-depth (MuJoCo minus nearest vertex) median {np.median(err) * 1e3:.1f} mm, "
              f"p90 |err| {np.percentile(np.abs(err), 90) * 1e3:.1f} mm over {sel.sum()} px")
    bz = w2c[:3, :3] @ mjr.d.qpos[0:3] + w2c[:3, 3]
    print(f"[snapshot] base origin -> pixel ({fy * bz[0] / bz[2] + w / 2:.1f}, {fy * bz[1] / bz[2] + h / 2:.1f}), "
          f"z {bz[2]:.3f} m; lowest robot mesh point z = {P[:, 2].min():.3f} m (world)")
    print(f"[snapshot] wrote {out} and {base}_robot.png", flush=True)
    sys.stdout.flush()
    os._exit(0)   # skip EGL/rclpy teardown noise


if __name__ == "__main__":
    if len(sys.argv) >= 9 and sys.argv[1] == "--snapshot":
        a = [float(x) for x in sys.argv[2:8]]
        snapshot(a[:3], a[3:6], sys.argv[8], live="--live" in sys.argv)
    else:
        main()
