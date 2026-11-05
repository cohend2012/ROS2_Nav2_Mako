"""Unit tests for m20_agent (ROS-free): grounding, executor safety rules, backends."""
import io
import json
import math
import os

import numpy as np
import pytest

from m20_agent.executor import Executor, FallbackBackend
from m20_agent.gauge_dial import render_dial, read_dial, value_to_angle, angle_to_value
from m20_agent.landmarks import LandmarkRegistry
from m20_agent.plan import validate, PlanError
from m20_agent.vlm_offline import OfflineBackend
from m20_agent.vlm_openai_compat import _extract_json
from m20_agent.vlm_claude import BackendError

ROOT = os.path.dirname(os.path.dirname(__file__))
LM = os.path.join(ROOT, "src", "m20_agent", "config", "oil_gas_field_landmarks.yaml")


@pytest.fixture
def reg():
    return LandmarkRegistry.load(LM)


class FakeIO:
    def __init__(self, fail_at=None, frame=None):
        self.p = (0.5, -0.2, 0.1); self.calls = []; self.fail_at = fail_at
        self.frame = frame
    def pose(self): return self.p
    def navigate(self, x, y, yaw, label):
        self.calls.append(label)
        if label == self.fail_at:
            return False, "blocked"
        self.p = (x, y, yaw); return True, "ok"
    def wait(self, s): self.calls.append("wait"); return True, "ok"
    def capture(self): return self.frame


def png_of(value, lo, hi):
    buf = io.BytesIO(); render_dial(value, lo, hi, "psi", "T").save(buf, "PNG")
    return buf.getvalue()


# ---------------- registry + grounding ----------------
def test_registry_aliases(reg):
    assert reg.resolve("PG-101").name == "gauge_pg101"
    assert reg.resolve("tank 2 gauge").name == "gauge_pg102"
    assert reg.resolve("the kitchen") is None


def test_registry_rejects_gauge_without_scale(tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text("landmarks:\n  - {name: g, type: gauge, position: [0,0], approach: [0,0,0]}\n")
    with pytest.raises(ValueError):
        LandmarkRegistry.load(str(p))


def test_validate_grounds_and_rejects(reg):
    steps = validate({"steps": [{"action": "read_gauge", "target": "PG-101"},
                                {"action": "return_home"}]}, reg)
    assert steps[0]["target"] == "gauge_pg101"
    with pytest.raises(PlanError, match="unknown landmark"):
        validate({"steps": [{"action": "goto", "target": "kitchen"}]}, reg)
    with pytest.raises(PlanError, match="not a gauge"):
        validate({"steps": [{"action": "read_gauge", "target": "tank1"}]}, reg)
    with pytest.raises(PlanError):
        validate({"steps": [{"action": "wait", "seconds": 999}]}, reg)
    with pytest.raises(PlanError):
        validate({"steps": [{"action": "fly", "target": "tank1"}]}, reg)


# ---------------- offline planner ----------------
@pytest.mark.parametrize("req,expect", [
    ("read all the gauges and return to where you started", ("read_gauge",) * 6 + ("return_home",)),
    ("go to tank 1", ("goto",)),
    ("read the PG-301 gauge", ("read_gauge",)),
    ("patrol the site and come back", None),
])
def test_offline_plans(reg, req, expect):
    p = validate(OfflineBackend(reg).plan(req), reg)
    acts = tuple(s["action"] for s in p)
    if expect is None:
        assert acts[-1] == "return_home" and acts.count("goto") >= 5
    else:
        assert acts == expect


def test_offline_unknown_target_does_not_move(reg):
    p = OfflineBackend(reg).plan("find the kitchen")
    assert [s["action"] for s in p["steps"]] == ["wait"]


# ---------------- executor rules ----------------
def test_executor_records_home_and_returns(reg, tmp_path):
    io_ = FakeIO(frame=png_of(40, 0, 100))
    rep = Executor(reg, OfflineBackend(reg), io_, str(tmp_path), log=lambda s: None).run(
        "read the tank 1 gauge and return home")
    assert rep["success"] and rep["home"] == [0.5, -0.2, 0.1]
    assert io_.calls == ["gauge_pg101", "home"]
    r = rep["readings"][0]
    assert abs(r["value"] - 40) < 1.0 and r["status"] == "OK"
    assert os.path.exists(r["image"]) and os.path.exists(rep["report_path"])


def test_executor_stops_on_failure_but_still_goes_home(reg):
    io_ = FakeIO(fail_at="gauge_pg102", frame=png_of(10, 0, 100))
    rep = Executor(reg, OfflineBackend(reg), io_, log=lambda s: None).run(
        "read all gauges and come back")
    st = [s["status"] for s in rep["steps"]]
    assert "FAILED" in st and "SKIPPED" in st
    assert rep["steps"][-1]["action"] == "return_home" and rep["steps"][-1]["status"] == "OK"
    assert not rep["success"]


def test_executor_refuses_without_pose(reg):
    io_ = FakeIO(); io_.pose = lambda: None
    rep = Executor(reg, OfflineBackend(reg), io_, log=lambda s: None).run("go to tank 1")
    assert not rep["success"] and io_.calls == []


def test_unreadable_frame_flagged(reg):
    blank = io.BytesIO()
    from PIL import Image
    Image.new("RGB", (320, 240), (90, 90, 90)).save(blank, "PNG")
    io_ = FakeIO(frame=blank.getvalue())
    rep = Executor(reg, OfflineBackend(reg), io_, log=lambda s: None).run("read PG-101")
    assert rep["readings"][0]["status"] == "UNREADABLE" and not rep["success"]


class Boom:
    name = "boom"
    def plan(self, *a): raise BackendError("link down")
    def read_gauge(self, *a): raise BackendError("link down")
    def describe(self, *a): raise BackendError("link down")


def test_fallback_backend_keeps_mission_alive(reg):
    io_ = FakeIO(frame=png_of(55, 0, 100))
    be = FallbackBackend(Boom(), OfflineBackend(reg), log=lambda s: None)
    rep = Executor(reg, be, io_, log=lambda s: None).run("read the tank 1 gauge")
    assert rep["success"] and rep["readings"][0]["fallback_reason"] == "link down"


# ---------------- dial convention + classical reader ----------------
def test_dial_angle_roundtrip():
    for v in (0, 12.5, 50, 99):
        assert angle_to_value(value_to_angle(v, 0, 100), 0, 100) == pytest.approx(v, abs=1e-6)
    assert value_to_angle(0, 0, 100) == pytest.approx(225)
    assert value_to_angle(100, 0, 100) == pytest.approx(-45)


@pytest.mark.parametrize("v", [3, 25, 50, 72.5, 97])
def test_read_dial_on_rendered_faces(v):
    img = np.asarray(render_dial(v, 0, 100, "psi", "X").convert("RGB"))
    frac, conf, _ = read_dial(img)
    assert frac * 100 == pytest.approx(v, abs=1.5) and conf > 0.7


# ---------------- cosmos JSON extraction ----------------
def test_extract_json_strips_think_and_fences():
    t = '<think>{"not": "this"}</think> Sure:\n```json\n{"steps": [], "rationale": "x"}\n```'
    assert _extract_json(t) == {"steps": [], "rationale": "x"}
    with pytest.raises(BackendError):
        _extract_json("no json here")


# ---------------- Claude backend request shape (fake client, no network) ----------------
class _FakeMsgs:
    def __init__(self, resp): self.resp, self.kw = resp, None
    def create(self, **kw): self.kw = kw; return self.resp


def _client(resp):
    from types import SimpleNamespace as NS
    m = _FakeMsgs(resp)
    return NS(beta=NS(messages=m)), m


def test_claude_backend_plan_request_and_parse(reg):
    from types import SimpleNamespace as NS
    from m20_agent.vlm_claude import ClaudeBackend
    resp = NS(stop_reason="tool_use", stop_details=None, content=[
        NS(type="tool_use", name="submit_plan",
           input={"steps": [{"action": "goto", "target": "tank1"}], "rationale": "r"})])
    c, m = _client(resp)
    out = ClaudeBackend(client=c).plan("go to tank 1", reg.describe())
    assert out["steps"][0]["target"] == "tank1"
    kw = m.kw
    assert kw["model"] == "claude-opus-5-5" and kw["tool_choice"] == {"type": "auto"}
    assert kw["tools"][0]["strict"] is True and kw["fallbacks"] == "default"
    assert "gauge_pg101" in kw["system"]          # registry is grounded in the prompt


def test_claude_backend_refusal_raises(reg):
    from types import SimpleNamespace as NS
    from m20_agent.vlm_claude import ClaudeBackend
    c, _ = _client(NS(stop_reason="refusal", stop_details=NS(category="cyber"), content=[]))
    with pytest.raises(BackendError, match="declined"):
        ClaudeBackend(client=c).plan("x", reg.describe())


def test_claude_backend_gauge_sends_image(reg):
    from types import SimpleNamespace as NS
    from m20_agent.vlm_claude import ClaudeBackend
    resp = NS(stop_reason="tool_use", stop_details=None, content=[NS(
        type="tool_use", name="report_reading",
        input={"readable": True, "value": 40, "units": "psi", "confidence": 0.9, "notes": ""})])
    c, m = _client(resp)
    r = ClaudeBackend(client=c).read_gauge(png_of(40, 0, 100), "gauge_pg101",
                                           {"units": "psi", "min": 0, "max": 100})
    assert r["value"] == 40
    blocks = m.kw["messages"][0]["content"]
    assert blocks[0]["type"] == "image" and blocks[0]["source"]["media_type"] == "image/png"


# ---------------- office features: teach places, routines, stop, unknown places ----------------
OFFICE = os.path.join(ROOT, "src", "m20_agent", "config", "office_landmarks.yaml")


def test_teach_place_persists_and_reloads(reg, tmp_path):
    taught = str(tmp_path / "taught.yaml")
    r = LandmarkRegistry.load(LM, taught)
    io_ = FakeIO(); io_.p = (1.5, 2.5, 0.7)
    rep = Executor(r, OfflineBackend(r), io_, log=lambda s: None).run("this is the kitchen")
    assert rep["success"] and io_.calls == []                 # teaching never moves the robot
    r2 = LandmarkRegistry.load(LM, taught)                    # survives a restart
    assert r2.get("kitchen").approach == (1.5, 2.5, 0.7)
    io2 = FakeIO()
    rep = Executor(r2, OfflineBackend(r2), io2, log=lambda s: None).run("go to the kitchen and come back")
    assert rep["success"] and io2.calls == ["kitchen", "home"]


def test_cannot_overwrite_surveyed_landmark_by_voice(tmp_path):
    r = LandmarkRegistry.load(LM, str(tmp_path / "t.yaml"))
    with pytest.raises(PlanError, match="surveyed"):
        validate({"steps": [{"action": "save_place", "target": "tank1"}]}, r)


def test_start_the_mission_runs_default_routine():
    r = LandmarkRegistry.load(OFFICE)
    steps = validate(OfflineBackend(r).plan("start the mission"), r)
    assert steps[-1] == {"action": "goto", "target": "dock"}
    assert sum(s["action"] == "inspect" for s in steps) == len(r.of_type("room"))
    steps = validate(OfflineBackend(r).plan("run the coffee run"), r)
    assert [s["action"] for s in steps] == ["goto", "wait", "return_home"]


def test_routines_cannot_nest(tmp_path):
    r = LandmarkRegistry.load(OFFICE)
    r.routines["loop"] = {"aliases": [], "description": "", "steps": [{"action": "run_routine", "target": "loop"}]}
    with pytest.raises(PlanError, match="cannot call routines"):
        validate({"steps": [{"action": "run_routine", "target": "loop"}]}, r)


def test_unknown_place_refuses_and_explains(reg):
    p = OfflineBackend(reg).plan("go to the kitchen and come back")
    assert [s["action"] for s in p["steps"]] == ["wait"] and "teach" in p["rationale"]


def test_stop_cancels_remaining_steps(reg):
    class StopAfterFirst(FakeIO):
        def navigate(self, x, y, yaw, label):
            ok = super().navigate(x, y, yaw, label)
            exe.stop()
            return ok
    io_ = StopAfterFirst(frame=png_of(40, 0, 100))
    exe = Executor(reg, OfflineBackend(reg), io_, log=lambda s: None)
    rep = exe.run("start the mission")
    assert not rep["success"] and rep["detail"] == "stopped by operator"
    assert len(io_.calls) == 1                                  # nothing after STOP, not even home
    assert all(s["status"] == "CANCELLED" for s in rep["steps"][1:])


def test_office_overlay_merges_footprint():
    import subprocess, sys, tempfile, yaml as y
    out = tempfile.mktemp(suffix=".yaml")
    subprocess.run([sys.executable, os.path.join(ROOT, "tools", "nav", "apply_overlay.py"),
                    os.path.join(ROOT, "src", "m20_navigation", "config", "nav2_params.yaml"),
                    os.path.join(ROOT, "src", "m20_navigation", "config", "nav2_office_overlay.yaml"), out],
                   check=True, capture_output=True)
    p = y.safe_load(open(out))
    for cm in ("local_costmap", "global_costmap"):
        rp = p[cm][cm]["ros__parameters"]
        assert "robot_radius" not in rp and rp["footprint"].startswith("[[0.46")
    assert p["velocity_smoother"]["ros__parameters"]["max_velocity"][0] == 0.3


def test_overlay_stacks_variants_and_rejects_unknown_nodes(tmp_path):
    import importlib.util
    spec_ = importlib.util.spec_from_file_location("apply_overlay", os.path.join(ROOT, "tools", "nav", "apply_overlay.py"))
    ao = importlib.util.module_from_spec(spec_); spec_.loader.exec_module(ao)
    base = {"controller_server": {"ros__parameters": {"FollowPath": {"plugin": "dwb", "max_vel_x": 0.4}}}}
    ao.merge(base, {"controller_server": {"ros__parameters": {"FollowPath": {
        "plugin": "mppi", "CostCritic": {"enabled": True}}}}})            # new nested block: allowed
    fp = base["controller_server"]["ros__parameters"]["FollowPath"]
    assert fp["plugin"] == "mppi" and fp["CostCritic"] == {"enabled": True} and fp["max_vel_x"] == 0.4
    with pytest.raises(KeyError, match="node section"):
        ao.merge(base, {"controler_server": {"ros__parameters": {}}})     # typo'd NODE: refused
    for v in ("shim_rpp", "lattice_rpp", "mppi"):                         # every shipped variant parses
        import yaml as y
        doc = y.safe_load(open(os.path.join(ROOT, "src", "m20_navigation", "config", "office_variants", v + ".yaml")))
        assert "controller_server" in doc or "planner_server" in doc


# ---------------- find the gauges (no prior positions) ----------------
from m20_agent import gauge_finder as gf  # noqa: E402
from m20_agent.gauge_search import search_and_read  # noqa: E402


class SearchIO(FakeIO):
    """A flat world with dial plates at hidden positions. The camera renders each plate
    the pinhole model says is in view; range_along hits a plate when the ray passes it."""
    PLATE_R = 0.16

    def __init__(self, gauges, viewpoints, wall=None):
        super().__init__()
        self.p = (0.0, 0.0, 0.0)
        self.g, self.vps = gauges, viewpoints     # [(x, y, value_frac)]

    def survey_viewpoints(self): return list(self.vps)

    def turn_to(self, yaw):
        self.p = (self.p[0], self.p[1], yaw); return True

    def navigate(self, x, y, yaw, label):
        self.calls.append(label)
        self.p = (x, y, self.p[2] if yaw is None else yaw); return True, "ok"

    def _rel(self, gx, gy):
        x, y, yaw = self.p
        d = math.hypot(gx - x, gy - y)
        b = math.atan2(math.sin(math.atan2(gy - y, gx - x) - yaw), math.cos(math.atan2(gy - y, gx - x) - yaw))
        return d, b

    def capture(self):
        from PIL import Image
        img = Image.new("RGB", (gf.IMG_W, gf.IMG_H), (90, 95, 100))
        for gx, gy, v in self.g:
            d, b = self._rel(gx, gy)
            if d < 0.3 or abs(b) > math.radians(27):
                continue
            r = int(gf.FOCAL_PX * self.PLATE_R / d)
            if r < 3:
                continue
            face = render_dial(v, 0, 1, "", "", size=max(8, int(2 * r / 0.92)))
            cx = int(gf.IMG_W / 2 - gf.FOCAL_PX * math.tan(b))
            img.paste(face, (cx - face.size[0] // 2, gf.IMG_H // 2 - face.size[1] // 2))
        buf = io.BytesIO(); img.save(buf, "PNG"); return buf.getvalue()

    def range_along(self, bearing, elev):
        for gx, gy, _ in self.g:
            d, b = self._rel(gx, gy)
            if abs(d * math.sin(b - bearing)) < self.PLATE_R and math.cos(b - bearing) > 0:
                return d, (gx, gy)
        return None


def test_find_gauges_locates_and_reads_hidden_gauges(tmp_path):
    reg = LandmarkRegistry.load(LM, str(tmp_path / "taught.yaml"))
    for n in [n for n in reg.names() if reg.get(n).type == "gauge"]:
        reg._lm.pop(n)                                  # positions removed
    truth = [(3.0, 1.0, 0.25), (-2.0, 3.0, 0.7)]
    io_ = SearchIO(truth, [(0.0, 0.0), (0.5, 2.5)])
    res = search_and_read(io_, OfflineBackend(reg), reg, log=lambda *_: None)
    assert len(res["gauges"]) == 2 and not res["rejected"]
    for (gx, gy, v) in truth:
        g = min(res["gauges"], key=lambda g: math.hypot(g["position"][0] - gx, g["position"][1] - gy))
        assert math.hypot(g["position"][0] - gx, g["position"][1] - gy) < 0.3
        assert g["status"] in ("OK", "NEEDS_REVIEW")
        assert abs(g["reading"]["value"] - v) < 0.05      # fraction of scale
    # saved as gauges with a viewing spot, persisted to the taught overlay
    found = reg.of_type("gauge")
    assert len(found) == 2 and all(lm.taught for lm in found)
    again = LandmarkRegistry.load(LM, str(tmp_path / "taught.yaml"))
    assert {lm.name for lm in again.of_type("gauge") if lm.taught} == {lm.name for lm in found}


def test_find_gauges_saves_nothing_it_cannot_confirm(tmp_path):
    reg = LandmarkRegistry.load(LM)
    io_ = SearchIO([], [(0.0, 0.0)])
    io_.range_along = lambda b, e: (2.0, (2.0, 0.0))     # LiDAR hits something...
    orig = io_.capture
    calls = {"n": 0}

    def capture():                                       # ...a dial-like blob far off once
        calls["n"] += 1
        if calls["n"] == 1:
            io_.g = [(3.5, 0.0, 0.5)]
            png = orig(); io_.g = []; return png
        return orig()
    io_.capture = capture
    n0 = len(reg.of_type("gauge"))
    res = search_and_read(io_, OfflineBackend(reg), reg, log=lambda *_: None)
    assert not res["gauges"] and res["rejected"]
    assert len(reg.of_type("gauge")) == n0


def test_offline_plans_find_gauges(reg, tmp_path):
    b = OfflineBackend(reg)
    steps = validate(b.plan("find the gauges and come back"), reg)
    assert [s["action"] for s in steps] == ["find_gauges", "return_home"]
    empty = LandmarkRegistry.load(os.path.join(ROOT, "src", "m20_agent", "config", "office_landmarks.yaml"))
    steps = validate(OfflineBackend(empty).plan("read all the gauges"), empty)
    assert steps[0]["action"] == "find_gauges"


def test_executor_find_gauges_reports_readings(tmp_path):
    reg = LandmarkRegistry.load(LM)
    for n in [n for n in reg.names() if reg.get(n).type == "gauge"]:
        reg._lm.pop(n)
    io_ = SearchIO([(3.0, 1.0, 0.4)], [(0.0, 0.0)])
    rep = Executor(reg, OfflineBackend(reg), io_, report_dir=str(tmp_path),
                   log=lambda *_: None).run("look for gauges and return to where you started")
    assert rep["success"], rep["detail"]
    assert [r["found"] for r in rep["readings"]] == [True]
    assert rep["find_gauges"]["candidates"] == 1
    assert io_.calls[-1] == "home"


def test_executor_find_gauges_needs_capable_io(reg):
    rep = Executor(reg, OfflineBackend(reg), FakeIO(), log=lambda *_: None).run("find the gauges")
    assert not rep["success"]
    assert "cannot search" in rep["steps"][0]["detail"]


@pytest.mark.parametrize("req,target", [
    ("read the oil pipe gauge", "gauge_fl401"),
    ("what does the flow gauge on the pipe rack say", "gauge_fl401"),
    ("read the gauge on tank 1", "gauge_pg101"),
])
def test_offline_gauge_by_description(reg, req, target):
    steps = validate(OfflineBackend(reg).plan(req), reg)
    assert steps == [{"action": "read_gauge", "target": target}]


def test_offline_find_unknown_place_explains(reg):
    raw = OfflineBackend(reg).plan("find the kitchen")
    assert [s["action"] for s in validate(raw, reg)] == ["wait"]
    assert "don't know where 'kitchen'" in raw["rationale"]


def test_offline_ambiguous_gauge_asks(reg):
    raw = OfflineBackend(reg).plan("read the pressure gauge")
    assert [s["action"] for s in validate(raw, reg)] == ["wait"] and "Which gauge" in raw["rationale"]


def test_view_plan_sees_every_face_including_thin_racks():
    """Planned views (no spinning) must look head-on at all 4 faces of a box AND both
    faces of a 0.1 m thin rack: the rack's two faces used to be merged into one."""
    import sys as _s
    _s.path.insert(0, os.path.join(ROOT, "src", "m20_ai_nav"))
    from m20_ai_nav.grid_planner import GridPlanner
    from m20_agent.gauge_finder import view_plan
    res, n = 0.05, 240                                   # 12 x 12 m
    g = np.zeros((n, n), np.int8)
    g[[0, -1], :] = 100; g[:, [0, -1]] = 100             # yard fence
    g[100:140, 60:100] = 100                             # 2 x 2 m box centred (-2, 0)
    g[118:120, 150:200] = 100                            # 0.1 m thick, 2.5 m rack at y=-0.05
    pl = GridPlanner(robot_radius=0.4, inflation_radius=0.8)
    pl.set_map(g.reshape(-1), n, n, res, -6.0, -6.0)
    views = view_plan(pl, (0.0, -4.0), bounds=(-5.5, 5.5, -5.5, 5.5))
    assert 0 < len(views) < 40

    def seen(px, py, nx, ny):
        for x, y, yaw in views:
            dx, dy = px - x, py - y
            d = math.hypot(dx, dy)
            rel = abs(math.atan2(math.sin(math.atan2(dy, dx) - yaw), math.cos(math.atan2(dy, dx) - yaw)))
            if 1.0 <= d <= 2.9 and rel <= math.radians(20) and -(dx * nx + dy * ny) / d >= math.cos(math.radians(45)):
                return True
        return False
    for face in [(-2.0, -1.0, 0, -1), (-2.0, 1.0, 0, 1), (-3.0, 0.0, -1, 0), (-1.0, 0.0, 1, 0),
                 (2.75, -0.1, 0, -1), (2.75, 0.0, 0, 1)]:          # rack: south AND north face
        assert seen(*face), f"no view of face {face}"
