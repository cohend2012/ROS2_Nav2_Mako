#!/usr/bin/env python3
"""Cosmos (or Claude) as the GLOBAL planner, a classical controller as the LOCAL planner,
head to head with the classical global planner (A*), on the robot's own LiDAR map.

    MUJOCO_GL=osmesa python3 tools/agent/vlm_route_eval.py --out <dir> [--backend cosmos] [--seed 3]

For each start -> goal leg (landmark to landmark, chosen to need a detour round tanks,
skids, the pipe rack or the ground pipes):
  astar : A* global path, replanned at 1 Hz (= Nav2 planner_server)            -> local
  vlm   : the model's waypoints from the map image, checked and, if rejected,
          corrected by feedback (m20_agent.vlm_route); NOT replanned by A*       -> local
The local planner is the same for both: NavCore + RPP-law pursuit + shield (what Nav2's
controller_server + local costmap do on the robot). Physics, collisions and the robot's
LiDAR are real; the global map is the LiDAR-built one (tools/agent/lidar_map.py).
Writes <dir>/route_eval.json and a map image per leg with both routes drawn.
"""
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "src", "m20_ai_nav"))
sys.path.insert(0, os.path.join(REPO, "src", "m20_agent"))

import numpy as np  # noqa: E402

import lidar_map  # noqa: E402
from sim_mission import LANDMARKS, PursuitDriver, SimRobotIO  # noqa: E402
from m20_ai_nav.core import NavCore, STUCK, SUCCEEDED  # noqa: E402
from m20_ai_nav.geometry import densify  # noqa: E402
from m20_ai_nav.sim.env import M20NavEnv  # noqa: E402
from m20_agent.executor import make_backend  # noqa: E402
from m20_agent.gauge_finder import survey_viewpoints  # noqa: E402
from m20_agent.landmarks import LandmarkRegistry  # noqa: E402
from m20_agent.vlm_route import VlmGlobalPlanner  # noqa: E402

LEGS = [("home", "tank2"), ("tank2", "skid1"), ("skid1", "pipe_rack"), ("pipe_rack", "west_field"),
        ("west_field", "wellhead"), ("wellhead", "tank1"), ("tank1", "east_field"), ("east_field", "home")]


def drive_path(io, path, label, timeout_s=150.0):
    """Local planner only: follow a fixed global path (no A* replanning)."""
    io.core.set_path(densify(np.asarray(path, float), 0.1), goal_yaw=None)
    t_end, t0 = io.t + timeout_s, io.t
    x0 = None
    dist = 0.0
    while io.t < t_end:
        x, y, yaw = io.pose()
        if x0 is not None:
            dist += math.hypot(x - x0[0], y - x0[1])
        x0 = (x, y)
        v = io.env.d.qvel
        cmd = io.core.tick(io.t, x, y, yaw, io.env.scan, math.cos(yaw) * v[0] + math.sin(yaw) * v[1], float(v[5]))
        if cmd.state == SUCCEEDED:
            return True, "arrived", io.t - t0, dist
        if cmd.state == STUCK:
            return False, cmd.detail, io.t - t0, dist
        info = io._drive(cmd.vx, cmd.wz)
        if info["collision"]:
            io.collisions += 1
            io._back_off(cmd.vx)
            return False, "collision", io.t - t0, dist
    return False, "timeout", io.t - t0, dist


def plen(p):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p, p[1:]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--backend", default="cosmos")
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--map", default="lidar", choices=["lidar", "truth"])
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    reg = LandmarkRegistry.load(LANDMARKS)
    home = (0.0, 0.0)
    env = M20NavEnv(scene="oil_gas_field_gauges.xml", randomize=True, seed=a.seed, max_steps=10 ** 9)
    env.reset(seed=a.seed, options={"start": home, "yaw": 0.0, "goal": home})
    io = SimRobotIO(env, os.path.join(REPO, "src", "m20_ai_nav", "models", "m20_nav_policy.onnx"))
    io.core = NavCore(PursuitDriver(), lookahead=0.6, stuck_timeout_s=20.0)
    if a.map == "lidar":
        tp = io.planner
        loop = survey_viewpoints(tp, home, spacing=3.0, clear=0.9)
        route, cur = [home], home
        for p in loop + [home]:
            seg = tp.plan(cur[0], cur[1], p[0], p[1])
            if seg:
                route += [tuple(q[:2]) for q in seg[1:]]; cur = p
        io.set_plan_map(lidar_map.build(env, route, seed=a.seed, log=lambda *_: None))
    vlm = make_backend(a.backend, reg)
    gp = VlmGlobalPlanner(vlm, io.planner)
    where = lambda n: home if n == "home" else tuple(reg.get(n).approach[:2])  # noqa: E731
    rows = []
    for src, dst in LEGS:
        s, g = where(src), where(dst)
        astar = io.planner.plan(s[0], s[1], g[0], g[1])
        t0 = time.time()
        r = gp.plan(s, g, landmarks=reg)
        lat = round(time.time() - t0, 1)
        with open(os.path.join(a.out, f"map_{src}_{dst}.png"), "wb") as f:
            f.write(r.pop("map_png"))
        row = {"leg": f"{src}->{dst}", "start": s, "goal": g,
               "astar_len_m": round(plen(astar), 2) if astar else None,
               "vlm_source": r["source"], "vlm_tries": len(r["tries"]), "vlm_latency_s": lat,
               "vlm_waypoints": r["waypoints"] and [list(map(lambda v: round(v, 2), w)) for w in r["waypoints"]],
               "vlm_len_m": round(plen(r["waypoints"]), 2) if r["waypoints"] else None,
               "vlm_first_errors": r["tries"][0].get("errors") if r["tries"] else None,
               "vlm_rationale": r.get("rationale", "")}
        for name, path in (("astar", astar), ("vlm", r["waypoints"] if r["source"] != "astar-fallback" else None)):
            if not path:
                row[name] = {"ok": None}
                continue
            # drive it: teleport to the start (same start state for both), then local planner
            env.reset(seed=a.seed, options={"start": s, "yaw": math.atan2(path[1][1] - s[1], path[1][0] - s[0]), "goal": g})
            io.core = NavCore(PursuitDriver(), lookahead=0.6, stuck_timeout_s=20.0)
            c0 = io.collisions
            ok, why, t, dist = drive_path(io, path, f"{name}:{src}->{dst}")
            tx, ty, _ = io.true_pose()
            row[name] = {"ok": ok, "detail": why, "time_s": round(t, 1), "driven_m": round(dist, 2),
                         "final_err_m": round(math.hypot(tx - g[0], ty - g[1]), 2),
                         "collisions": io.collisions - c0}
        rows.append(row)
        print(json.dumps({k: row[k] for k in ("leg", "astar_len_m", "vlm_source", "vlm_tries", "vlm_len_m",
                                              "vlm_latency_s")}), row["astar"], row["vlm"], flush=True)
    summ = {}
    for name in ("astar", "vlm"):
        dr = [r_[name] for r_ in rows if r_[name].get("ok") is not None]
        summ[name] = {"driven": len(dr), "succeeded": sum(d["ok"] for d in dr),
                      "collisions": sum(d["collisions"] for d in dr),
                      "mean_time_s": round(float(np.mean([d["time_s"] for d in dr if d["ok"]])), 1) if any(d["ok"] for d in dr) else None}
    summ["vlm_valid_first_try"] = sum(1 for r_ in rows if r_["vlm_source"] == "vlm")
    summ["vlm_valid_after_feedback"] = sum(1 for r_ in rows if r_["vlm_source"] == "vlm+feedback")
    summ["vlm_fell_back_to_astar"] = sum(1 for r_ in rows if r_["vlm_source"] == "astar-fallback")
    summ["vlm_vs_astar_length"] = round(float(np.mean([r_["vlm_len_m"] / r_["astar_len_m"] for r_ in rows
                                                       if r_["vlm_source"] != "astar-fallback" and r_["astar_len_m"]])), 2) \
        if any(r_["vlm_source"] != "astar-fallback" for r_ in rows) else None
    out = {"backend": vlm.name, "map": a.map, "seed": a.seed, "summary": summ, "legs": rows}
    json.dump(out, open(os.path.join(a.out, "route_eval.json"), "w"), indent=1)
    print(json.dumps(summ, indent=1))


if __name__ == "__main__":
    main()
