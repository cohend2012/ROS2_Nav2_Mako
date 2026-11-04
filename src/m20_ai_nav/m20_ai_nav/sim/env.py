"""M20NavEnv: Gymnasium environment for the L1 learned local navigator.

Physics: the REAL vendor M20 MJCF (sdk_deploy pin, tools/vendor.repos) at 500 Hz,
legs held in the standing stance by the same PD law as tools/mujoco_sim.py, wheels
driven by SkidSteerMixer (= bridge SimVendorSDK) at 20 Hz behind an AccelLimiter
(= Nav2 velocity_smoother). The policy acts at 10 Hz with (vx, wz) — the robot is
NEVER commanded at joint level by the learner.

Sensor: a 72-azimuth x 16-ring cast of the real LiDAR ring table, height-sliced
exactly like pointcloud_to_laserscan.yaml (0.10-1.00 m above ground), min-pooled
per 5 deg bin -> the same 72-bin scan the ROS node builds from /scan.

Scenes:
  scene="procedural" : obstacle slots (boxes, posts, tanks, ground pipes) re-placed
                       every reset; difficulty curriculum via `density`.
  scene="field"      : tools/sim/oil_gas_field.xml as-is (evaluation only).

Domain randomization (Tobin 2017): floor friction, base mass, action latency,
scan noise + dropout, wheel-radius mismatch.

Env var M20_MJCF_DIR: directory holding M20.xml (default: ~/m20_sim/... per
tools/setup_sim.sh).
"""
import math
import os

import gymnasium as gym
import mujoco
import numpy as np

from m20_ai_nav import spec
from m20_ai_nav.mixer import SkidSteerMixer, AccelLimiter

DEFAULT_MJCF_DIR = os.path.expanduser(
    "~/m20_sim/sdk_deploy/src/M20_sdk_deploy/M20_description/m20_mjcf/mjcf")
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))

SIM_HZ = 500.0
MIX_HZ = 20.0
NJ = 16
WHEELS = (3, 7, 11, 15)
LEFT, RIGHT = (3, 11), (7, 15)
STANCE = np.array([0.0, -0.7, 1.4, 0.0, 0.0, -0.7, 1.4, 0.0,
                   0.0, 0.7, -1.4, 0.0, 0.0, 0.7, -1.4, 0.0])
LIDAR_ELEVS_DEG = [15, 10, 5, 2, 0, -2, -4, -6, -8, -10, -13, -16, -20, -25, -32, -45]
LIDAR_OFFSET = 0.10
SLICE_LO, SLICE_HI = 0.10, 1.00          # above ground (pointcloud_to_laserscan.yaml)
ROBOT_RADIUS = 0.45                      # nav2 robot_radius

# obstacle slots: (kind, size) — kinds are MuJoCo geom types
SLOTS = ([("box", (0.25, 0.25, 0.30))] * 4 + [("box", (0.5, 0.7, 0.35))] * 3
         + [("box", (0.15, 0.9, 0.5))] * 2                       # wall segments
         + [("cyl", (0.12, 0.5))] * 4 + [("cyl", (0.6, 0.8))] * 2  # posts, tanks
         + [("pipe", (0.15, 2.5))] * 2)                          # ground pipes (lying)
ARENA = 7.0                               # half-width of the procedural arena


def _yaw_quat(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


class M20NavEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, scene="procedural", density=1.0, randomize=True, seed=None,
                 max_steps=300, goal_range=(1.5, 7.0), mjcf_dir=None):
        super().__init__()
        self.scene, self.density, self.randomize = scene, density, randomize
        self.max_steps, self.goal_range = max_steps, goal_range
        mjcf_dir = mjcf_dir or os.environ.get("M20_MJCF_DIR", DEFAULT_MJCF_DIR)
        robot_xml = os.path.join(mjcf_dir, "M20.xml")
        if not os.path.exists(robot_xml):
            raise FileNotFoundError(f"{robot_xml} missing — run tools/setup_sim.sh first")
        self.m = self._build_model(mjcf_dir)
        self.m.opt.timestep = 1.0 / SIM_HZ
        for w in WHEELS:
            self.m.dof_armature[6 + w] = 0.03       # same as mujoco_sim.py
        self.d = mujoco.MjData(self.m)
        self.lo = self.m.actuator_ctrlrange[:, 0].copy()
        self.hi = self.m.actuator_ctrlrange[:, 1].copy()
        self.kp = np.full(NJ, 200.0); self.kd = np.full(NJ, 4.0)
        for w in WHEELS:
            self.kp[w], self.kd[w] = 0.0, 5.0
        self.floor_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.robot_geom = self.m.geom_bodyid > 0
        self.obst_ids = np.array([g for g in range(self.m.ngeom)
                                  if self.m.geom_bodyid[g] == 0 and g != self.floor_id])
        self._is_obst = np.zeros(self.m.ngeom, bool); self._is_obst[self.obst_ids] = True
        self.slot_ids = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, f"slot{i}")
                         for i in range(len(SLOTS))] if scene == "procedural" else []
        self.base_friction = self.m.geom_friction[self.floor_id].copy()
        self.base_mass = self.m.body_mass[1]
        self._settle()
        # ray table (azimuth-major), sensor frame
        dirs, az_idx = [], []
        for k in range(spec.N_BINS):
            az = 2 * math.pi * k / spec.N_BINS
            for e in LIDAR_ELEVS_DEG:
                el = math.radians(e)
                dirs.append([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az),
                             math.sin(el)])
                az_idx.append(k)
        self.ray_dirs = np.array(dirs); self.ray_az = np.array(az_idx)
        self.nray = len(dirs)
        self._gid = np.full(self.nray, -1, np.int32); self._dist = np.zeros(self.nray)
        self.geomgroup = np.array([1, 0, 0, 0, 0, 0], np.uint8)
        self.substeps = int(SIM_HZ / spec.CONTROL_HZ)
        self.mix_every = int(SIM_HZ / MIX_HZ)
        self.mixer = SkidSteerMixer(); self.limiter = AccelLimiter()
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (spec.OBS_DIM,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (2,), np.float32)
        self.rng = np.random.default_rng(seed)
        self.goal = np.zeros(2)

    # ------------------------------------------------------------------ model
    def _build_model(self, mjcf_dir):
        if self.scene.endswith(".xml"):          # any scene file next to M20.xml
            path = self.scene if os.path.isabs(self.scene) else os.path.join(mjcf_dir, self.scene)
            return mujoco.MjModel.from_xml_path(path)
        if self.scene == "field":
            path = os.path.join(mjcf_dir, "oil_gas_field.xml")
            if not os.path.exists(path):
                import shutil
                shutil.copy(os.path.join(REPO, "tools", "sim", "oil_gas_field.xml"), path)
            return mujoco.MjModel.from_xml_path(path)
        s = mujoco.MjSpec.from_file(os.path.join(mjcf_dir, "M20.xml"))
        wb = s.worldbody
        for i, (kind, size) in enumerate(SLOTS):
            g = wb.add_geom()
            g.name = f"slot{i}"
            g.rgba = [0.55, 0.55, 0.6, 1]
            if kind == "box":
                g.type = mujoco.mjtGeom.mjGEOM_BOX
                g.size = [size[0], size[1], size[2]]
                g.pos = [50 + 3 * i, 50, size[2]]
            elif kind == "cyl":
                g.type = mujoco.mjtGeom.mjGEOM_CYLINDER
                g.size = [size[0], size[1], 0]
                g.pos = [50 + 3 * i, 50, size[1]]
            else:   # pipe: cylinder lying along x
                g.type = mujoco.mjtGeom.mjGEOM_CYLINDER
                g.size = [size[0], size[1], 0]
                g.pos = [50 + 3 * i, 50, size[0]]
                g.quat = [math.cos(math.pi / 4), 0, math.sin(math.pi / 4), 0]
        return s.compile()

    def _settle(self):
        """Stand once from a safe height, keep the settled state as the reset template."""
        d = self.d
        mujoco.mj_resetData(self.m, d)
        d.qpos[2] = 0.6; d.qpos[3] = 1.0; d.qpos[7:7 + NJ] = STANCE
        for _ in range(1500):
            self._pd(np.zeros(NJ))
            mujoco.mj_step(self.m, d)
        if not (0.40 < d.qpos[2] < 0.52):
            raise RuntimeError(f"robot failed to stand (base z={d.qpos[2]:.3f})")
        self.q_stand = d.qpos.copy(); self.q_stand[0:2] = 0.0
        self.q_stand[3:7] = [1, 0, 0, 0]

    def _pd(self, wheel_vel):
        q = self.d.qpos[7:7 + NJ]; qd = self.d.qvel[6:6 + NJ]
        t = self.kp * (STANCE - q) + self.kd * (wheel_vel - qd)
        self.d.ctrl[:] = np.clip(t, self.lo, self.hi)

    # ------------------------------------------------------------------ scene
    def _free(self, xy, placed, clearance):
        return all(math.hypot(xy[0] - p[0], xy[1] - p[1]) > p[2] + clearance for p in placed)

    def _layout(self):
        """Place slots randomly; returns the list of (x, y, radius) footprints."""
        placed = []
        n_use = int(round(len(SLOTS) * min(1.0, self.density)))
        use = set(self.rng.choice(len(SLOTS), n_use, replace=False).tolist())
        for i, (kind, size) in enumerate(SLOTS):
            gid = self.slot_ids[i]
            if i not in use:
                self.m.geom_pos[gid][:2] = [60 + 3 * i, 60]
                continue
            rad = math.hypot(size[0], size[1]) if kind == "box" else (
                size[0] if kind == "cyl" else size[1])
            for _ in range(30):
                xy = self.rng.uniform(-ARENA, ARENA, 2)
                if self._free(xy, placed, 0.3) and math.hypot(*xy) > rad + 0.9:
                    break
            yaw = self.rng.uniform(-math.pi, math.pi)
            self.m.geom_pos[gid][:2] = xy
            if kind == "box":
                self.m.geom_quat[gid] = _yaw_quat(yaw)
            elif kind == "pipe":
                # lying cylinder: rotate axis into the plane, then yaw
                qy = np.array([math.cos(math.pi / 4), 0, math.sin(math.pi / 4), 0])
                q = np.zeros(4); mujoco.mju_mulQuat(q, _yaw_quat(yaw), qy)
                self.m.geom_quat[gid] = q
                # footprint of a pipe = a line; approximate by samples along it
                for t in np.linspace(-size[1], size[1], 6):
                    placed.append((xy[0] + t * math.cos(yaw), xy[1] + t * math.sin(yaw), 0.2))
                continue
            placed.append((xy[0], xy[1], rad))
        return placed

    def _gap_layout(self, w, x=2.5):
        """G2 gap-width sweep: a barrier across the route at x with one gap of width w
        (walls + big skids, half-span w/2 + 3.2 m). Start (0,0) yaw 0, goal (5,0)."""
        for i in range(len(SLOTS)):
            self.m.geom_pos[self.slot_ids[i]][:2] = [60 + 3 * i, 60]
        walls = [i for i, s in enumerate(SLOTS) if s == ("box", (0.15, 0.9, 0.5))]
        skids = [i for i, s in enumerate(SLOTS) if s == ("box", (0.5, 0.7, 0.35))][:2]
        placed = []
        for sign, wi, si in ((1, walls[0], skids[0]), (-1, walls[1], skids[1])):
            for idx, yc in ((wi, w / 2 + 0.9), (si, w / 2 + 1.8 + 0.7)):
                g = self.slot_ids[idx]
                self.m.geom_pos[g][:2] = [x, sign * yc]
                self.m.geom_quat[g] = [1, 0, 0, 0]
                placed.append((x, sign * yc, 0.9))
        return placed

    def _field_footprints(self):
        out = []
        for g in self.obst_ids:
            p = self.m.geom_pos[g]; s = self.m.geom_size[g]
            if self.m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
                out.append((p[0], p[1], math.hypot(s[0], s[1])))
            else:
                out.append((p[0], p[1], s[0] + 0.1))
        return out

    # ------------------------------------------------------------------ gym API
    def reset(self, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        opts = options or {}
        if self.scene == "procedural" and "gap" in opts:
            placed = self._gap_layout(float(opts["gap"]))
        else:
            placed = self._layout() if self.scene == "procedural" else self._field_footprints()
        d = self.d
        mujoco.mj_resetData(self.m, d)
        d.qpos[:] = self.q_stand
        start = np.asarray(opts.get("start", (0.0, 0.0)), float)
        if self.scene != "procedural" and "start" not in opts:
            for _ in range(100):
                start = self.rng.uniform(-6, 6, 2)
                if self._free(start, placed, ROBOT_RADIUS + 0.3):
                    break
        yaw0 = float(opts.get("yaw", self.rng.uniform(-math.pi, math.pi)))
        d.qpos[0:2] = start; d.qpos[3:7] = _yaw_quat(yaw0)
        if "goal" in opts:
            self.goal = np.asarray(opts["goal"], float)
        else:
            for _ in range(200):
                r = self.rng.uniform(*self.goal_range)
                a = self.rng.uniform(-math.pi, math.pi)
                g = start + r * np.array([math.cos(a), math.sin(a)])
                lim = ARENA if self.scene == "procedural" else 8.0
                if max(abs(g)) < lim and self._free(g, placed, ROBOT_RADIUS + 0.25):
                    break
            self.goal = g
        # domain randomization
        if self.randomize:
            self.m.geom_friction[self.floor_id][0] = self.base_friction[0] * self.rng.uniform(0.6, 1.2)
            self.m.body_mass[1] = self.base_mass * self.rng.uniform(0.9, 1.25)
            self.latency = int(self.rng.integers(0, 2))          # 0 or 100 ms
            self.scan_sigma = self.rng.uniform(0.0, 0.03)
            self.dropout = self.rng.uniform(0.0, 0.06)
            self.mixer.R = 0.072 * self.rng.uniform(0.92, 1.08)  # calibration error
        else:
            self.latency, self.scan_sigma, self.dropout = 0, 0.0, 0.0
            self.mixer.R = 0.072
        mujoco.mj_forward(self.m, d)
        self.mixer.reset(); self.limiter.reset()
        self.wheel_vel = np.zeros(NJ)
        self.steps = 0
        self.last_action = np.zeros(2, np.float32)
        self.pending = [np.zeros(2, np.float32)] * self.latency
        self.prev_dist = self._goal_dist()
        self.path_len = 0.0
        self.start_dist = self.prev_dist
        self.scan = self._scan(); self.prev_scan = self.scan.copy()
        return self._obs(), {}

    def step(self, action):
        action = np.clip(np.asarray(action, np.float32), -1, 1)
        self.pending.append(action)
        act = self.pending.pop(0)
        vx_t, wz_t = spec.action_to_cmd(act)
        x0 = self.d.qpos[0:2].copy()
        collided = False
        dt_mix = 1.0 / MIX_HZ
        for k in range(self.substeps):
            if k % self.mix_every == 0:
                vx, wz = self.limiter.step((vx_t, wz_t), dt_mix)
                vl, vr = self.mixer.wheels(vx, wz, float(self.d.qvel[5]), dt_mix)
                self.wheel_vel[:] = 0.0
                self.wheel_vel[list(LEFT)] = vl; self.wheel_vel[list(RIGHT)] = vr
            self._pd(self.wheel_vel)
            mujoco.mj_step(self.m, self.d)
            if k % 10 == 9 and self._in_collision():
                collided = True
                break
        self.steps += 1
        self.path_len += float(np.linalg.norm(self.d.qpos[0:2] - x0))
        self.prev_scan = self.scan; self.scan = self._scan()
        dist = self._goal_dist()
        roll, pitch = self._tilt()
        tipped = abs(roll) > 0.5 or abs(pitch) > 0.5 or self.d.qpos[2] < 0.30
        success = dist < 0.25
        # reward
        r = 5.0 * (self.prev_dist - dist) - 0.01
        r -= 0.05 * float(np.sum(np.abs(action - self.last_action)))
        near = float(self.scan.min())
        if near < 0.9:
            r -= 0.15 * (0.9 - near)
        if success:
            r += 10.0
        if collided or tipped:
            r -= 10.0
        self.prev_dist = dist
        self.last_action = action
        terminated = bool(success or collided or tipped)
        truncated = self.steps >= self.max_steps
        info = {"success": bool(success), "collision": bool(collided), "tipped": bool(tipped),
                "path_len": self.path_len, "start_dist": self.start_dist, "dist": dist}
        return self._obs(), float(r), terminated, truncated, info

    # ------------------------------------------------------------------ helpers
    def _in_collision(self):
        d = self.d
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = c.geom1, c.geom2
            if (self._is_obst[g1] and self.robot_geom[g2]) or (self._is_obst[g2] and self.robot_geom[g1]):
                return True
        return False

    def _tilt(self):
        w, x, y, z = self.d.qpos[3:7]
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        return roll, pitch

    def yaw(self):
        w, x, y, z = self.d.qpos[3:7]
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def _goal_dist(self):
        return float(np.linalg.norm(self.goal - self.d.qpos[0:2]))

    def _scan(self):
        R = np.empty(9); mujoco.mju_quat2Mat(R, self.d.qpos[3:7]); R = R.reshape(3, 3)
        origin = self.d.qpos[0:3] + R @ np.array([0, 0, LIDAR_OFFSET])
        vecs = self.ray_dirs @ R.T
        mujoco.mj_multiRay(self.m, self.d, origin, vecs.reshape(-1), self.geomgroup, 1, -1,
                           self._gid, self._dist, None, self.nray, 12.0)
        dist = self._dist.copy()
        hit = (self._gid >= 0) & (dist > spec.SCAN_MIN)
        hz = origin[2] + vecs[:, 2] * dist
        hit &= (hz > SLICE_LO) & (hz < SLICE_HI)
        horiz = dist * np.hypot(vecs[:, 0], vecs[:, 1])
        if self.scan_sigma > 0:
            horiz = horiz + self.rng.normal(0, self.scan_sigma, self.nray)
        if self.dropout > 0:
            hit &= self.rng.random(self.nray) > self.dropout
        out = np.full(spec.N_BINS, spec.SCAN_MAX)
        np.minimum.at(out, self.ray_az[hit], np.clip(horiz[hit], spec.SCAN_MIN, spec.SCAN_MAX))
        return out.astype(np.float32)

    def goal_body(self):
        dx, dy = self.goal - self.d.qpos[0:2]
        y = self.yaw()
        return (math.cos(y) * dx + math.sin(y) * dy, -math.sin(y) * dx + math.cos(y) * dy)

    def _obs(self):
        gx, gy = self.goal_body()
        y = self.yaw()
        v = self.d.qvel[0:2]
        vx_body = math.cos(y) * v[0] + math.sin(y) * v[1]
        return spec.build_obs(self.scan, self.prev_scan, gx, gy, self.last_action,
                              vx_body, float(self.d.qvel[5]))
