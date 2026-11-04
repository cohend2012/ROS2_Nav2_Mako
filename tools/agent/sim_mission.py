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
from m20_agent.executor import Executor, make_backend  # noqa: E402
from m20_agent.landmarks import LandmarkRegistry  # noqa: E402

LANDMARKS = os.path.join(REPO, "src", "m20_agent", "config", "oil_gas_field_landmarks.yaml")
TRUTH = os.path.join(REPO, "tools", "sim", "gauges", "truth.json")


def rasterize(m, d, res=0.05, half=10.0, z_probe=(0.15, 0.4, 0.8)):
    """Occupancy grid (0 free / 100 occupied) of world obstacle geoms, by testing cell
    centres against each geom's true shape at a few heights (what a 3D-sliced map sees)."""
    n = int(2 * half / res)
    xs = -half + (np.arange(n) + 0.5) * res
    X, Y = np.meshgrid(xs, xs)                    # rows = y, cols = x
    occ = np.zeros((n, n), bool)
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != 0 or g == floor:
            continue
        p, R, s = d.geom_xpos[g], d.geom_xmat[g].reshape(3, 3), m.geom_size[g]
        for z in z_probe:
            P = np.stack([X - p[0], Y - p[1], np.full_like(X, z - p[2])], -1) @ R   # geom frame
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
                inside = (np.abs(P[..., 0]) <= s[0]) & (np.abs(P[..., 1]) <= s[1]) & (np.abs(P[..., 2]) <= s[2])
            elif m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER:
                inside = (np.hypot(P[..., 0], P[..., 1]) <= s[0]) & (np.abs(P[..., 2]) <= s[1])
            else:
                continue
            occ |= inside
    return np.where(occ, 100, 0).astype(np.int8).reshape(-1), n, n, res, -half, -half


class SimRobotIO:
    def __init__(self, env, policy_path, log=print, nav_timeout_s=120.0):
        self.env, self.log, self.timeout = env, log, nav_timeout_s
        self.core = NavCore(OnnxPolicy(policy_path))
        self.planner = GridPlanner()
        mujoco.mj_forward(env.m, env.d)
        self.planner.set_map(*rasterize(env.m, env.d))
        self.renderer = mujoco.Renderer(env.m, 240, 320)
        self.cam = mujoco.MjvCamera(); self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.t = 0.0
        self.collisions = 0
        self.trace = []

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
        x, y, _ = self.pose()
        path = self.planner.plan(x, y, gx, gy)
        if path is None:
            return False, f"A*: no path to {label}"
        self.core.set_path(path, goal_yaw=gyaw)
        self.env.goal = np.array([gx, gy])
        t_end = self.t + self.timeout
        last_plan = self.t
        while self.t < t_end:
            if self.t - last_plan > 1.0:                  # 1 Hz replan like standalone mode
                last_plan = self.t
                new = self.planner.plan(*self.pose()[:2], gx, gy)
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
                return False, f"collision near ({x:.2f},{y:.2f})"
            if info["tipped"]:
                return False, "tipped over"
        return False, f"timeout ({self.timeout:.0f} s) to {label}"

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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--request", required=True)
    ap.add_argument("--backend", default="offline")
    ap.add_argument("--model", default=os.path.join(REPO, "src", "m20_ai_nav", "models", "m20_nav_policy.onnx"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--start", default="0,0,0")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    sx, sy, syaw = (float(v) for v in a.start.split(","))
    env = M20NavEnv(scene="oil_gas_field_gauges.xml", randomize=True, seed=a.seed,
                    max_steps=10 ** 9)
    env.reset(seed=a.seed, options={"start": (sx, sy), "yaw": syaw, "goal": (sx, sy)})
    reg = LandmarkRegistry.load(LANDMARKS)
    io_ = SimRobotIO(env, a.model)
    exe = Executor(reg, make_backend(a.backend, reg), io_, report_dir=a.out)
    t0 = time.time()
    rep = exe.run(a.request)
    x, y, _ = io_.pose()
    rep["sim"] = {"sim_time_s": round(io_.t, 1), "wall_s": round(time.time() - t0, 1),
                  "collisions": io_.collisions, "final_pose": [round(x, 3), round(y, 3)],
                  "home_error_m": round(math.hypot(x - sx, y - sy), 3)}
    if any(s["action"] == "read_gauge" for s in rep.get("plan", [])):
        rep["G5"] = score_g5(rep)
    with open(os.path.join(a.out, "trace.json"), "w") as f:
        json.dump(io_.trace, f)
    with open(rep["report_path"], "w") as f:
        json.dump(rep, f, indent=2)
    print(json.dumps({k: rep[k] for k in ("success", "detail", "sim") if k in rep}, indent=1))
    if "G5" in rep:
        print(json.dumps(rep["G5"], indent=1))


if __name__ == "__main__":
    main()
