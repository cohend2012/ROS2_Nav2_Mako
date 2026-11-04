"""Unit tests for m20_ai_nav runtime logic (ROS-free, MuJoCo-free)."""
import math
import sys
import types

import numpy as np
import pytest

from m20_ai_nav import spec
from m20_ai_nav.core import NavCore, SUCCEEDED, ALIGNING, BLOCKED, STUCK, TRACKING
from m20_ai_nav.geometry import CarrotTracker, to_body
from m20_ai_nav.grid_planner import GridPlanner
from m20_ai_nav.mixer import SkidSteerMixer, AccelLimiter

CLEAR = np.full(spec.N_BINS, spec.SCAN_MAX, np.float32)


class Fwd:
    """Fake policy: always full forward, no turn. Records the obs it saw."""
    def __init__(self): self.obs = []
    def __call__(self, obs): self.obs.append(obs); return np.array([1.0, 0.0])


# ---------------- spec: the policy contract ----------------
def test_obs_dim_is_frozen():
    # changing this invalidates the shipped model: bump OBS_VERSION + retrain
    assert spec.OBS_DIM == 151 and spec.OBS_VERSION == 1


def test_action_scaling_respects_smoother_limits():
    assert spec.action_to_cmd([1, 1]) == (pytest.approx(0.4), pytest.approx(0.7))
    assert spec.action_to_cmd([-1, -1]) == (pytest.approx(-0.2), pytest.approx(-0.7))
    assert spec.action_to_cmd([5, -5]) == (pytest.approx(0.4), pytest.approx(-0.7))
    for vx, wz in [(0.3, 0.2), (-0.1, -0.5), (0.0, 0.0)]:
        assert spec.action_to_cmd(spec.cmd_to_action(vx, wz)) == (pytest.approx(vx), pytest.approx(wz))


def test_bin_scan_orientation_and_minpool():
    # 360 x 1-deg scan like pointcloud_to_laserscan; obstacle at +90 deg (left), 1.0 m
    r = np.full(360, np.inf); ang_min = -math.pi
    left = int(round((math.pi / 2 - ang_min) / math.radians(1)))
    r[left] = 1.0; r[left + 1] = 2.0
    b = spec.bin_scan(r, ang_min, math.radians(1))
    assert b[18] == pytest.approx(1.0)                  # bin 18 = 90 deg CCW
    assert np.sum(b < spec.SCAN_MAX) == 1               # 2.0 at 91 deg pooled into same bin
    assert b[0] == spec.SCAN_MAX


def test_bin_scan_treats_blind_and_nan_as_no_return():
    b = spec.bin_scan([0.2, float("nan"), 0.0], 0.0, 0.01)
    assert np.all(b == spec.SCAN_MAX)


def test_bin_scan_refuses_empty():
    with pytest.raises(ValueError):
        spec.bin_scan([], 0.0, 0.01)


# ---------------- mixer parity with the bridge ----------------
def test_mixer_matches_bridge():
    """The training plant must be the stack's plant (SimVendorSDK.send_velocity)."""
    fake = types.ModuleType("drdds"); fake.msg = types.ModuleType("drdds.msg")

    class _J:  # minimal JointsDataCmd
        def __init__(self):
            self.data = types.SimpleNamespace(joints_data=[types.SimpleNamespace(
                position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque=0.0) for _ in range(16)])
            self.header = types.SimpleNamespace(stamp=None)
    fake.msg.JointsDataCmd = _J; fake.msg.ImuData = object
    sys.modules["drdds"] = fake; sys.modules["drdds.msg"] = fake.msg
    from m20_locomotion_bridge.bridge_node import SimVendorSDK

    class _Clock:
        t = 0.0
        def now(self): return types.SimpleNamespace(nanoseconds=int(self.t * 1e9), to_msg=lambda: None)
    clock = _Clock()
    sent = []
    node = types.SimpleNamespace(
        create_publisher=lambda *a: types.SimpleNamespace(publish=sent.append),
        create_subscription=lambda *a: None, get_clock=lambda: clock)
    sdk = SimVendorSDK(node)
    mix = SkidSteerMixer()
    rng = np.random.default_rng(0)
    for k in range(200):
        clock.t = 0.05 * (k + 1)
        vx, wz, om = rng.uniform(-0.2, 0.4), rng.uniform(-0.7, 0.7), rng.uniform(-0.5, 0.5)
        if k % 7 == 0:
            vx = 0.0
        sdk._omega = om
        sdk.send_velocity(vx, 0.0, wz)
        vl, vr = mix.wheels(vx, wz, om, 0.05)
        j = sent[-1].data.joints_data
        assert j[3].velocity == pytest.approx(vl) and j[7].velocity == pytest.approx(vr)
    del sys.modules["drdds"], sys.modules["drdds.msg"]


def test_accel_limiter_ramps():
    lim = AccelLimiter()
    v = lim.step((0.4, 0.0), 0.05)
    assert v[0] == pytest.approx(0.05)          # 1.0 m/s^2 * 0.05 s
    for _ in range(20):
        v = lim.step((0.4, 0.0), 0.05)
    assert v[0] == pytest.approx(0.4)


# ---------------- carrot ----------------
def test_carrot_lookahead_and_monotone():
    t = CarrotTracker(lookahead=1.0)
    t.set_path([(x * 0.1, 0.0) for x in range(51)])   # 5 m straight line
    assert t.carrot(0.0, 0.0) == pytest.approx((1.0, 0.0))
    assert t.carrot(2.0, 0.3) == pytest.approx((3.0, 0.0))
    assert t.carrot(0.0, 0.0)[0] >= 2.0                # never snaps back behind progress
    assert t.carrot(4.8, 0.0) == pytest.approx((5.0, 0.0))


def test_to_body():
    bx, by = to_body(1.0, 1.0, 0.0, 0.0, math.pi / 2)
    assert (bx, by) == (pytest.approx(1.0), pytest.approx(-1.0))


# ---------------- NavCore state machine ----------------
def test_core_tracks_then_aligns_then_succeeds():
    pol = Fwd(); c = NavCore(pol)
    c.set_path([(0, 0), (2, 0)], goal_yaw=math.pi / 2)
    cmd = c.tick(0.0, 0.0, 0.0, 0.0, CLEAR)
    assert cmd.state == TRACKING and cmd.vx == pytest.approx(0.4)
    gx, gy, gcos = pol.obs[-1][2 * spec.N_BINS:2 * spec.N_BINS + 3]
    assert gy == pytest.approx(0.0) and gcos == pytest.approx(1.0)   # carrot dead ahead
    cmd = c.tick(1.0, 1.9, 0.0, 0.0, CLEAR)
    assert cmd.state == ALIGNING and cmd.vx == 0.0 and cmd.wz > 0
    cmd = c.tick(2.0, 1.9, 0.0, math.pi / 2 - 0.05, CLEAR)
    assert cmd.state == SUCCEEDED


def test_core_alignment_command_beats_bridge_deadzone():
    c = NavCore(Fwd()); c.set_path([(0, 0), (1, 0)], goal_yaw=0.28)
    cmd = c.tick(0.0, 1.0, 0.0, 0.0, CLEAR)
    assert abs(cmd.wz) >= 0.15


def test_safety_envelope_blocks_forward_but_allows_turning():
    c = NavCore(Fwd()); c.set_path([(0, 0), (5, 0)])
    near = CLEAR.copy(); near[0] = 0.55
    cmd = c.tick(0.0, 0.0, 0.0, 0.0, near)
    assert cmd.state == BLOCKED and cmd.vx == 0.0
    side = CLEAR.copy(); side[18] = 0.55               # obstacle at 90 deg: not in cone
    c2 = NavCore(Fwd()); c2.set_path([(0, 0), (5, 0)])
    assert c2.tick(0.0, 0.0, 0.0, 0.0, side).state == TRACKING


def test_stuck_detection():
    c = NavCore(Fwd(), stuck_timeout_s=5.0); c.set_path([(0, 0), (5, 0)])
    assert c.tick(0.0, 0.0, 0.0, 0.0, CLEAR).state == TRACKING
    assert c.tick(4.0, 0.1, 0.0, 0.0, CLEAR).state == TRACKING
    assert c.tick(6.0, 0.1, 0.0, 0.0, CLEAR).state == STUCK


# ---------------- A* ----------------
def _grid_with_wall():
    w = h = 100                                   # 5 m x 5 m at 0.05
    g = np.zeros((h, w), np.int8)
    g[0:80, 48:52] = 100                          # wall at x=2.4-2.6 from y=0..4
    return g.reshape(-1), w, h


def test_astar_goes_around_wall_with_clearance():
    data, w, h = _grid_with_wall()
    pl = GridPlanner(plan_res=0.1)
    pl.set_map(data, w, h, 0.05, 0.0, 0.0)
    path = pl.plan(1.0, 1.0, 4.0, 1.0)
    assert path is not None and path[-1] == pytest.approx((4.0, 1.0))
    ys = [p[1] for p in path]
    assert max(ys) > 4.0 + 0.55                    # went over the top of the wall
    for x, y in path:                              # never inside robot radius of wall
        assert pl.dist[pl.world_to_cell(x, y)] > 0.55


def test_astar_no_path_returns_none():
    g = np.zeros((100, 100), np.int8); g[:, 48:52] = 100
    pl = GridPlanner(); pl.set_map(g.reshape(-1), 100, 100, 0.05, 0.0, 0.0)
    assert pl.plan(1.0, 1.0, 4.0, 1.0) is None


def test_astar_requires_map():
    with pytest.raises(RuntimeError):
        GridPlanner().plan(0, 0, 1, 1)


def test_blind_zone_memory_keeps_obstacle_visible():
    """Obstacle seen at 0.7 m ahead, then the robot creeps forward 0.3 m: the scan now
    returns nothing there (inside the 0.5 m blind radius) — memory must still block."""
    c = NavCore(Fwd()); c.set_path([(0, 0), (5, 0)])
    seen = CLEAR.copy(); seen[0] = 0.7
    c.tick(0.0, 0.0, 0.0, 0.0, seen)
    cmd = c.tick(0.5, 0.3, 0.0, 0.0, CLEAR)          # obstacle now 0.4 m away: invisible
    assert cmd.state == BLOCKED and cmd.vx == 0.0
    # after pivoting 90 deg left, the remembered obstacle is on the RIGHT (bin 54)
    c2 = NavCore(Fwd()); c2.set_path([(0, 0), (5, 0)])
    c2.tick(0.0, 0.0, 0.0, 0.0, seen)
    obs_seen = []
    c2.policy = lambda o: (obs_seen.append(o), np.array([0.0, 0.0]))[1]
    c2.tick(0.5, 0.0, 0.0, math.pi / 2, CLEAR)
    assert obs_seen[-1][54] * spec.SCAN_MAX == pytest.approx(0.7, abs=0.05)


def test_blind_zone_memory_expires():
    c = NavCore(Fwd()); c.set_path([(0, 0), (5, 0)])
    seen = CLEAR.copy(); seen[0] = 0.7
    c.tick(0.0, 0.0, 0.0, 0.0, seen)
    assert c.tick(4.0, 0.0, 0.0, 0.0, CLEAR).state == TRACKING


def test_carrot_on_two_point_path_advances():
    """direct mode passes [start, goal]: the carrot must keep moving with the robot
    (regression: it froze at start+lookahead and the robot orbited it)."""
    t = CarrotTracker(lookahead=1.5)
    t.set_path([(0.0, 0.0), (-6.0, -0.5)])
    c0 = t.carrot(0.0, 0.0)
    c1 = t.carrot(-1.5, -0.12)
    assert math.hypot(c1[0] + 1.5, c1[1] + 0.12) == pytest.approx(1.5, abs=0.1)
    assert c1[0] < c0[0] - 1.0
    assert t.remaining(-1.5, -0.12) == pytest.approx(math.hypot(4.5, 0.38), abs=0.15)


def test_align_drift_returns_to_tracking():
    c = NavCore(Fwd()); c.set_path([(0, 0), (2, 0)], goal_yaw=math.pi)
    assert c.tick(0.0, 1.9, 0.0, 0.0, CLEAR).state == ALIGNING
    assert c.tick(1.0, 1.4, 0.3, 1.0, CLEAR).state == TRACKING     # pivot drifted 0.67 m
