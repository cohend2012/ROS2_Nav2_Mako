#!/usr/bin/env python3
"""End-to-end M20-Brain mission in MuJoCo, no ROS: gates G4 + G5 (AI_NAV_BRIEF §5).

    MUJOCO_GL=osmesa python3 tools/agent/sim_mission.py \
        --request "read all the gauges and return to where you started" \
        --backend offline --model src/m20_ai_nav/models/m20_nav_policy.onnx \
        --out docs/ai_nav/mission_offline

Full stack, every layer real except ROS transport and localization:
  L3 agent    : Executor + backend (offline | claude | cosmos)
  L2 memory   : src/m20_agent/config/oil_gas_field_landmarks.yaml
  global plan : GridPlanner A* on an occupancy grid rasterized from the scene
                (= standalone mode; the stack uses slam_toolbox's /map)
  L1 policy   : NavCore + ONNX PPO policy -> (vx, wz)
  plant       : vendor M20 MJCF, PD legs, SkidSteerMixer + AccelLimiter at 500 Hz
  camera      : first-person render posed exactly like tools/mujoco_sim.py
Pose comes from simulator ground truth. Localization is gated separately (M1a), and
this harness tests navigation + perception + agent, not the estimator.

G5 scoring: each reading vs tools/sim/gauges/truth.json, PASS if within 5% of full
scale; the gate needs >= 5/6 within and 0 confident-but-wrong (status OK but
outside tolerance).
"""
import argparse
import io
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "src", "m20_ai_nav"))
sys.path.insert(0, os.path.join(REPO, "src", "m20_agent"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from m20_ai_nav import spec  # noqa: E402
from m20_ai_nav.core import NavCore, SUCCEEDED, STUCK  # noqa: E402
from m20_ai_nav.grid_planner import GridPlanner  # noqa: E402
from m20_ai_nav.policy import OnnxPolicy  # noqa: E402
from m20_ai_nav.sim.env import M20NavEnv  # noqa: E402
from m20_ai_nav.sim.raster import rasterize  # noqa: E402
from m20_agent.executor import Executor, make_backend  # noqa: E402
from m20_agent.landmarks import LandmarkRegistry  # noqa: E402

LANDMARKS = os.path.join(REPO, "src", "m20_agent", "config", "oil_gas_field_landmarks.yaml")
TRUTH = os.path.join(REPO, "tools", "sim", "gauges", "truth.json")


class SimRobotIO:
    def __init__(self, env, policy_path, log=print, nav_timeout_s=120.0, plan_radius=0.55):
        self.env, self.log, self.timeout = env, log, nav_timeout_s
        office = plan_radius < 0.5        # = the office site profile (ai_nav.launch SITE_PARAMS)
        self.core = NavCore(OnnxPolicy(policy_path), lookahead=0.9 if office else 1.5,
                            stuck_timeout_s=20.0 if office else 12.0)
        self.planner = GridPlanner(robot_radius=plan_radius, inflation_radius=max(0.8, plan_radius + 0.5),
                                   pivot_clear=0.6 if plan_radius < 0.5 else None)
        mujoco.mj_forward(env.m, env.d)
        self.planner.set_map(*rasterize(env.m, env.d))
        self.renderer = mujoco.Renderer(env.m, 240, 320)
        self.cam = mujoco.MjvCamera(); self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.t = 0.0
        self.collisions = 0
        self.trace = []
        self.events = []                  # (label, t_start, t_end, ok, detail) per leg

    def pose(self):
        d = self.env.d
        return float(d.qpos[0]), float(d.qpos[1]), self.env.yaw()

    def _drive(self, vx, wz):
        env = self.env
        _, _, _, _, info = env.step(spec.cmd_to_action(vx, wz))
        self.t += 1.0 / spec.CONTROL_HZ
        x, y, yaw = self.pose()
        self.trace.append((round(self.t, 1), round(x, 3), round(y, 3), round(yaw, 3)))
        return info

    def navigate(self, gx, gy, gyaw, label):
        t0 = self.t
        self.cur_label, self.in_nav = label, True
        ok, why = self._navigate(gx, gy, gyaw, label)
        self.in_nav = False
        self.events.append((label, round(t0, 1), round(self.t, 1), bool(ok), why))
        return ok, why

    def _navigate(self, gx, gy, gyaw, label):
        x, y, _ = self.pose()
        path = self.planner.plan_pose(x, y, gx, gy, gyaw)
        if path is None:
            return False, f"A*: no path to {label}"
        self.core.set_path(path, goal_yaw=gyaw)
        self.env.goal = np.array([gx, gy])
        t_end = self.t + self.timeout
        last_plan = self.t
        while self.t < t_end:
            if self.t - last_plan > 1.0:                  # 1 Hz replan like standalone mode
                last_plan = self.t
                new = self.planner.plan_pose(*self.pose()[:2], gx, gy, gyaw)
                if new is not None:
                    self.core.update_path(new)
            x, y, yaw = self.pose()
            v = self.env.d.qvel
            vxm = math.cos(yaw) * v[0] + math.sin(yaw) * v[1]
            cmd = self.core.tick(self.t, x, y, yaw, self.env.scan, vxm, float(v[5]))
            if cmd.state == SUCCEEDED:
                for _ in range(5):
                    self._drive(0.0, 0.0)                 # come to rest before photos
                return True, cmd.detail
            if cmd.state == STUCK:
                return False, cmd.detail
            info = self._drive(cmd.vx, cmd.wz)
            if info["collision"]:
                self.collisions += 1
                return False, f"collision near ({x:.2f},{y:.2f}){self._back_off(cmd.vx)}"
            if info["tipped"]:
                return False, "tipped over"
        return False, f"timeout ({self.timeout:.0f} s) to {label}"

    def _back_off(self, vx_at_contact):
        """Nav2's BackUp recovery equivalent: leave the contact before the next leg
        (without it every later leg 'collides' at t=0 and the mission is lost)."""
        for direction in ((-1.0, 1.0) if vx_at_contact >= 0 else (1.0, -1.0)):
            clear = 0
            for _ in range(400):            # in contact a step is cut to 20 ms: up to ~8 s
                clear = 0 if self._drive(0.15 * direction, 0.0)["collision"] else clear + 1
                if clear >= 8:              # ~0.12 m clear of the contact
                    for _ in range(5):
                        self._drive(0.0, 0.0)
                    return ", backed off"
        return ", still in contact"

    def wait(self, seconds):
        for _ in range(int(seconds * spec.CONTROL_HZ)):
            self._drive(0.0, 0.0)
        return True, f"waited {seconds} s"

    def capture(self):
        from PIL import Image
        d = self.env.d
        x, y, z = d.qpos[0:3]
        yaw = self.env.yaw()
        ex, ey = x + 0.30 * math.cos(yaw), y + 0.30 * math.sin(yaw)
        self.cam.lookat[:] = [ex + 0.5 * math.cos(yaw), ey + 0.5 * math.sin(yaw), z + 0.05]
        self.cam.distance = 0.5; self.cam.azimuth = math.degrees(yaw); self.cam.elevation = 0.0
        self.renderer.update_scene(d, camera=self.cam)
        buf = io.BytesIO(); Image.fromarray(self.renderer.render()).save(buf, "PNG")
        return buf.getvalue()


def score_g5(report, tol=0.05):
    truth = json.load(open(TRUTH))
    rows, within, wrong_ok = [], 0, 0
    for g, t in truth.items():
        r = next((x for x in report["readings"] if x.get("gauge") == g), None)
        fs = t["max"] - t["min"]
        if r is None or not r.get("readable"):
            rows.append({"gauge": g, "true": t["value"], "read": None, "status": "MISSING" if r is None else r["status"]})
            continue
        err = abs(r["value"] - t["value"]) / fs
        ok = err <= tol
        within += ok
        wrong_ok += (not ok) and r["status"] == "OK"
        rows.append({"gauge": g, "true": t["value"], "read": r["value"], "err_pct_fs": round(100 * err, 2),
                     "status": r["status"], "within": bool(ok)})
    return {"rows": rows, "within": within, "of": len(truth), "confident_wrong": wrong_ok,
            "pass": within >= 5 and wrong_ok == 0}


def score_found(report, tol_m=0.6, tol_fs=0.05):
    """find_gauges: each true plate matched to a found gauge (< tol_m), reading compared
    as a fraction of full scale (the offline reader's unit when no scale is on file)."""
    truth = json.load(open(TRUTH))
    found = [r for r in report["readings"] if r.get("found")]
    rows, used = [], set()
    for g, t in truth.items():
        tx, ty = t["plate"][:2]
        best = min((r for r in found if r["gauge"] not in used),
                   key=lambda r: math.hypot(r["position"][0] - tx, r["position"][1] - ty), default=None)
        d = math.hypot(best["position"][0] - tx, best["position"][1] - ty) if best else None
        if best is None or d >= tol_m:
            rows.append({"gauge": g, "found": False}); continue
        used.add(best["gauge"])
        tf = (t["value"] - t["min"]) / (t["max"] - t["min"])
        err = abs(best["value"] - tf) if best.get("readable") else None
        rows.append({"gauge": g, "found": True, "as": best["gauge"], "pos_err_m": round(d, 3),
                     "true_frac": round(tf, 4), "read_frac": best.get("value"), "status": best["status"],
                     "err_pct_fs": None if err is None else round(100 * err, 2),
                     "within": err is not None and err <= tol_fs})
    return {"rows": rows, "found": sum(r["found"] for r in rows), "of": len(truth),
            "read_within": sum(bool(r.get("within")) for r in rows),
            "false_positives": len(found) - len(used),
            "confident_wrong": sum(1 for r in rows if r.get("status") == "OK" and not r.get("within"))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--request", required=True)
    ap.add_argument("--backend", default="offline")
    ap.add_argument("--model", default=os.path.join(REPO, "src", "m20_ai_nav", "models", "m20_nav_policy.onnx"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--start", default="0,0,0")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--hide-gauges", action="store_true",
                    help="remove every gauge from the landmark file first (find_gauges missions); "
                         "found gauges are scored against truth.json")
    ap.add_argument("--video", help="record the mission live to this .mp4 (tools/agent/mission_video.py)")
    ap.add_argument("--site", default="outdoor", choices=["outdoor", "office"],
                    help="office: tools/sim/office.xml + office_landmarks.yaml, start at reception, "
                         "0.32 m planner clearance (= M20_SITE=office)")
    a = ap.parse_args()
    if a.site == "office":
        import yaml
        lm_path = os.path.join(REPO, "src", "m20_agent", "config", "office_landmarks.yaml")
        rec = {l["name"]: l for l in yaml.safe_load(open(lm_path))["landmarks"]}["reception"]["approach"]
        if a.start == "0,0,0":
            a.start = ",".join(str(v) for v in rec)
        scene, plan_radius = "office.xml", 0.32
    else:
        lm_path, scene, plan_radius = LANDMARKS, "oil_gas_field_gauges.xml", 0.55
    sx, sy, syaw = (float(v) for v in a.start.split(","))
    env = M20NavEnv(scene=scene, randomize=True, seed=a.seed, max_steps=10 ** 9)
    env.reset(seed=a.seed, options={"start": (sx, sy), "yaw": syaw, "goal": (sx, sy)})
    reg = LandmarkRegistry.load(lm_path, os.path.join(a.out, "taught.yaml") if a.hide_gauges else None)
    if a.hide_gauges:
        for n in [n for n in reg.names() if reg.get(n).type == "gauge"]:
            reg._lm.pop(n)
        sys.path.insert(0, HERE)
        from find_gauges import FinderIO, BOUNDS          # + range_along / turn_to / face_normal
        from m20_agent.gauge_finder import survey_viewpoints
        io_ = FinderIO(env, a.model, plan_radius=plan_radius)
        io_.survey_viewpoints = lambda: survey_viewpoints(io_.planner, (sx, sy), spacing=2.0,
                                                          clear=0.9, bounds=BOUNDS)
    else:
        io_ = SimRobotIO(env, a.model, plan_radius=plan_radius)
    video = None
    if a.video:
        sys.path.insert(0, HERE)
        from mission_video import MissionVideo
        n_pts = len(io_.survey_viewpoints()) if hasattr(io_, "survey_viewpoints") else None
        video = MissionVideo(io_, reg, a.video, total_points=n_pts)
    exe = Executor(reg, make_backend(a.backend, reg), io_, report_dir=a.out)
    t0 = time.time()
    rep = exe.run(a.request)
    if video:
        video.close()
    x, y, _ = io_.pose()
    rep["sim"] = {"sim_time_s": round(io_.t, 1), "wall_s": round(time.time() - t0, 1),
                  "collisions": io_.collisions, "final_pose": [round(x, 3), round(y, 3)],
                  "home_error_m": round(math.hypot(x - sx, y - sy), 3)}
    if a.site == "outdoor" and any(s["action"] == "read_gauge" for s in rep.get("plan", [])):
        rep["G5"] = score_g5(rep)
    if a.hide_gauges:
        rep["G6"] = score_found(rep)
    with open(os.path.join(a.out, "trace.json"), "w") as f:
        json.dump(io_.trace, f)
    with open(os.path.join(a.out, "events.json"), "w") as f:
        json.dump(io_.events, f)
    with open(rep["report_path"], "w") as f:
        json.dump(rep, f, indent=2)
    print(json.dumps({k: rep[k] for k in ("success", "detail", "sim") if k in rep}, indent=1))
    for k in ("G5", "G6"):
        if k in rep:
            print(json.dumps({x: y for x, y in rep[k].items() if x != "rows"}, indent=1))


if __name__ == "__main__":
    main()
