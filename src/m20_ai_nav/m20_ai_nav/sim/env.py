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
                 max_steps=300, goal_range=(1.5, 7.0), mjcf_dir=None, shield=False):
        super().__init__()
        # shield=True: SHIELDED RL. The policy's command passes through the same
        # core.shield() (front stop + footprint guard) and blind-zone memory that
        # NavCore applies on the robot, so it learns to make progress INSIDE the
        # safety layer it is deployed with (v2 office training without it plateaued
        # at ~21% success / 78% collisions: it kept pivoting corners into walls).
        self.use_shield = shield
        from m20_ai_nav.core import ObstacleMemory
        self.memory = ObstacleMemory()
        self.eff = self.prev_eff = None
        self.scene, self.density, self.randomize = scene, density, randomize
        self.max_steps, self.goal_range = max_steps, goal_range
        mjcf_dir = mjcf_dir or os.environ.get("M20_MJCF_DIR", DEFAULT_MJCF_DIR)
        robot_xml = os.path.join(mjcf_dir, "M20.xml")
        if not os.path.exists(robot_xml):
            raise FileNotFoundError(f"{robot_xml} missing — run tools/setup_sim.sh first")
        self.mjcf_dir = mjcf_dir
        self.rng = np.random.default_rng(seed)
        self.layout = None
        if scene == "office":
            from m20_ai_nav.sim import office
            self.layout = office.generate(self.rng)
        self.kp = np.full(NJ, 200.0); self.kd = np.full(NJ, 4.0)
        for w in WHEELS:
            self.kp[w], self.kd[w] = 0.0, 5.0
        self._bind(self._build_model(mjcf_dir))
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
        self.goal = np.zeros(2)
        self.path_mode = scene == "office"
        self.tracker = None

    def _bind(self, m):
        """(Re)bind a compiled model: the office scene compiles a NEW layout per
        episode; robot geoms/qpos layout are identical so the stance template holds."""
        self.m = m
        m.opt.timestep = 1.0 / SIM_HZ
        for w in WHEELS:
            m.dof_armature[6 + w] = 0.03       # same as mujoco_sim.py
        self.d = mujoco.MjData(m)
        self.lo = m.actuator_ctrlrange[:, 0].copy()
        self.hi = m.actuator_ctrlrange[:, 1].copy()
        self.floor_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        # robot = everything in base_link's kinematic tree (by ROOT body, so mocap
        # people — separate roots — are obstacles, not robot)
        base_root = m.body_rootid[1]
        self.robot_geom = m.body_rootid[m.geom_bodyid] == base_root
        self.robot_geom &= m.geom_bodyid > 0
        self.obst_ids = np.array([g for g in range(m.ngeom)
                                  if not self.robot_geom[g] and g != self.floor_id])
        self._is_obst = np.zeros(m.ngeom, bool); self._is_obst[self.obst_ids] = True
        self.slot_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"slot{i}")
                         for i in range(len(SLOTS))] if self.scene == "procedural" else []
        self.base_friction = m.geom_friction[self.floor_id].copy()
        self.base_mass = m.body_mass[1]
        self.people_mocap = []
        for k in range(8):
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"person{k}")
            if b >= 0:
                self.people_mocap.append(int(m.body_mocapid[b]))

    # ------------------------------------------------------------------ model
    def _build_model(self, mjcf_dir):
        if self.scene == "office":
            from m20_ai_nav.sim import office
            sp = mujoco.MjSpec.from_file(os.path.join(mjcf_dir, "M20.xml"))
            office.add_to_spec(sp, self.layout)
            return sp.compile()
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
        """Stand once from a safe height, keep the settled state as the reset template.
        Office layouts can put furniture at the origin, so the office scene settles on
        the bare robot model (same qpos layout: people are mocap, not joints)."""
        m_keep, d_keep = self.m, self.d
        if self.scene == "office":
            self.m = mujoco.MjModel.from_xml_path(os.path.join(self.mjcf_dir, "M20.xml"))
            self.m.opt.timestep = 1.0 / SIM_HZ
            for w in WHEELS:
                self.m.dof_armature[6 + w] = 0.03
            self.d = mujoco.MjData(self.m)
        d = self.d
        mujoco.mj_resetData(self.m, d)
        d.qpos[2] = 0.6; d.qpos[3] = 1.0; d.qpos[7:7 + NJ] = STANCE
        for _ in range(1500):
            self._pd(np.zeros(NJ))
            mujoco.mj_step(self.m, d)
        self.m, self.d = m_keep, d_keep
        if not (0.40 < d.qpos[2] < 0.52):  # noqa: E501 (d = the settle model's data)
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
        if self.scene == "office":
            return self._reset_office(opts)
        self.tracker = None
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
        self.memory.reset(); self.eff = self.prev_eff = None
        return self._obs(), {}

    OFFICE_PLAN_RADIUS = 0.32     # m: half the M20's 0.53 m width + 0.05; doors >= 0.64 m plan
    OFFICE_MAX_STEPS = 600

    def _reset_office(self, opts):
        """New office layout -> compile -> A* path start->goal -> carrot tracking,
        exactly as standalone/follow_path deploy it (goal feature = 1.5 m carrot)."""
        from m20_ai_nav.sim import office
        from m20_ai_nav.sim.raster import rasterize
        from m20_ai_nav.grid_planner import GridPlanner
        from m20_ai_nav.geometry import CarrotTracker
        for _attempt in range(20):
            self.layout = opts.get("layout") or office.generate(
                self.rng, door_range=opts.get("door_range", (0.80, 1.20)),
                n_people=opts.get("n_people"))
            self._bind(self._build_model(self.mjcf_dir))
            mujoco.mj_resetData(self.m, self.d)
            self.d.qpos[:] = self.q_stand
            self._t = 0.0
            self._place_people()
            mujoco.mj_forward(self.m, self.d)
            pl = GridPlanner(robot_radius=self.OFFICE_PLAN_RADIUS, inflation_radius=0.8, plan_res=0.10)
            pl.set_map(*rasterize(self.m, self.d, res=0.10))
            self.planner = pl
            free = np.argwhere(pl.dist > 0.55)          # start/goal: whole robot fits
            path = None
            for _ in range(60):
                if "start" in opts:
                    s0 = tuple(opts["start"]); g0 = tuple(opts["goal"])
                else:
                    s0 = pl.cell_to_world(*free[self.rng.integers(len(free))])
                    g0 = pl.cell_to_world(*free[self.rng.integers(len(free))])
                    if not 3.0 < math.hypot(g0[0] - s0[0], g0[1] - s0[1]) < 13.0:
                        continue
                    if any(math.hypot(s0[0] - px, s0[1] - py) < 1.2 for px, py in self._people_xy()):
                        continue
                path = pl.plan(*s0, *g0)
                if path is not None or "start" in opts:
                    break
            if path is not None:
                break
        if path is None:
            raise RuntimeError("office reset: no connected start/goal pair found")
        self.tracker = CarrotTracker(lookahead=1.5)
        self.tracker.set_path(path)
        self.plan_len = float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path[:-1], path[1:])))
        d = self.d
        yaw0 = float(opts.get("yaw", self.rng.uniform(-math.pi, math.pi)))
        d.qpos[0:2] = s0; d.qpos[3:7] = _yaw_quat(yaw0)
        self.goal = np.asarray(g0, float)
        self._randomize()
        mujoco.mj_forward(self.m, d)
        self.mixer.reset(); self.limiter.reset()
        self.wheel_vel = np.zeros(NJ)
        self.steps = 0
        self.last_action = np.zeros(2, np.float32)
        self.pending = [np.zeros(2, np.float32)] * self.latency
        self.prev_dist = self.tracker.remaining(*s0)
        self.path_len = 0.0
        self.start_dist = self.plan_len            # SPL reference = A* length
        self.scan = self._scan(); self.prev_scan = self.scan.copy()
        self.memory.reset(); self.eff = self.prev_eff = None
        return self._obs(), {}

    def _randomize(self):
        if self.randomize:
            self.m.geom_friction[self.floor_id][0] = self.base_friction[0] * self.rng.uniform(0.6, 1.2)
            self.m.body_mass[1] = self.base_mass * self.rng.uniform(0.9, 1.25)
            self.latency = int(self.rng.integers(0, 2))
            self.scan_sigma = self.rng.uniform(0.0, 0.03)
            self.dropout = self.rng.uniform(0.0, 0.06)
            self.mixer.R = 0.072 * self.rng.uniform(0.92, 1.08)
        else:
            self.latency, self.scan_sigma, self.dropout = 0, 0.0, 0.0
            self.mixer.R = 0.072

    def _people_xy(self):
        from m20_ai_nav.sim import office
        if self.layout is None:
            return []
        return [office.person_xy(p, p_t) for p, p_t in zip(self.layout.people, self._pt)]

    def _place_people(self):
        """Park unused mocap slots; put each person on their path."""
        n = len(self.layout.people) if self.layout else 0
        self._pt = [0.0] * n
        for k, mid in enumerate(self.people_mocap):
            if k < n:
                x, y = self._people_xy()[k]
                self.d.mocap_pos[mid] = [x, y, 0.85]
            else:
                self.d.mocap_pos[mid] = [60 + 3 * k, 60, 0.85]

    def _move_people(self, dt):
        """People walk their paths but are POLITE: anyone within 0.9 m of the robot
        stops (real people step around a robot; an unavoidable rear-ending by a
        pedestrian would only teach the policy noise)."""
        if not self.layout or not self.layout.people:
            return
        rx, ry = self.d.qpos[0], self.d.qpos[1]
        cur = self._people_xy()
        for k, p in enumerate(self.layout.people):
            if math.hypot(cur[k][0] - rx, cur[k][1] - ry) > 0.9:
                self._pt[k] += dt
        for k, (x, y) in enumerate(self._people_xy()):
            if k < len(self.people_mocap):
                self.d.mocap_pos[self.people_mocap[k]] = [x, y, 0.85]

    def step(self, action):
        action = np.clip(np.asarray(action, np.float32), -1, 1)
        self.pending.append(action)
        act = self.pending.pop(0)
        vx_t, wz_t = spec.action_to_cmd(act)
        shielded = False
        if self.use_shield and self.eff is not None:
            from m20_ai_nav.core import shield
            vx_t, wz_t, shielded, _ = shield(vx_t, wz_t, self.eff)
        x0 = self.d.qpos[0:2].copy()
        collided = False
        dt_mix = 1.0 / MIX_HZ
        for k in range(self.substeps):
            if k % self.mix_every == 0:
                if self.tracker is not None:
                    self._move_people(dt_mix)
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
        # progress: straight-line to goal (outdoor) or remaining PATH length (office,
        # path mode: the robot is paid for following the plan, as deployed)
        prog = self.tracker.remaining(*self.d.qpos[0:2]) if self.tracker is not None else dist
        r = 5.0 * (self.prev_dist - prog) - 0.01
        r -= 0.05 * float(np.sum(np.abs(action - self.last_action)))
        near = float(self.scan.min())
        if near < 0.9:
            r -= 0.15 * (0.9 - near)
        if shielded:
            r -= 0.05        # don't LEAN on the shield: it is a backstop, not a driver
        if success:
            r += 10.0
        if collided or tipped:
            r -= 10.0
        self.prev_dist = prog
        self.last_action = action
        terminated = bool(success or collided or tipped)
        truncated = self.steps >= (self.OFFICE_MAX_STEPS if self.tracker is not None else self.max_steps)
        info = {"success": bool(success), "collision": bool(collided), "tipped": bool(tipped),
                "shielded": bool(shielded),
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
        if self.tracker is not None:                 # path mode: goal feature = carrot
            cx, cy = self.tracker.carrot(*self.d.qpos[0:2])
            dx, dy = cx - self.d.qpos[0], cy - self.d.qpos[1]
            y_ = self.yaw()
            gx, gy = math.cos(y_) * dx + math.sin(y_) * dy, -math.sin(y_) * dx + math.cos(y_) * dy
        else:
            gx, gy = self.goal_body()
        y = self.yaw()
        v = self.d.qvel[0:2]
        vx_body = math.cos(y) * v[0] + math.sin(y) * v[1]
        if self.use_shield:          # exactly what NavCore feeds the policy on the robot
            bins = self.memory.update(self.steps * 0.1, float(self.d.qpos[0]), float(self.d.qpos[1]),
                                      y, self.scan)
            prev = self.prev_eff if self.prev_eff is not None else bins
            self.prev_eff = self.eff = bins
            return spec.build_obs(bins, prev, gx, gy, self.last_action, vx_body, float(self.d.qvel[5]))
        return spec.build_obs(self.scan, self.prev_scan, gx, gy, self.last_action,
                              vx_body, float(self.d.qvel[5]))
