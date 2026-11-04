#!/usr/bin/env python3
"""Gates G1 + G2 for the L1 navigator (docs/AI_NAV_BRIEF.md §5), in the training sim.

    python3 tools/ai_nav/evaluate.py --model src/m20_ai_nav/models/m20_nav_policy.onnx \
        --episodes 500 --out docs/ai_nav/eval_v1.json
    python3 tools/ai_nav/evaluate.py --baseline ...    # scripted reactive baseline

G1: held-out procedural episodes (seeds 100000+, full density, domain randomization ON)
    -> success / collision / tip / timeout rates + SPL (Anderson et al. 2018).
    SPL uses the straight-line start->goal distance as the shortest-path length l,
    which UNDER-estimates l around obstacles, so the SPL reported here is conservative.
G2: gap-width sweep: barrier with one gap of width w across the route; 10 seeds per w.
F : the real oil_gas_field.xml scene as DEPLOYED (standalone: A* + NavCore + policy),
    start/goal from the planner's free space, connected pairs only; SPL vs the A* length.

Baseline = a scripted reactive controller (vector-field-histogram style), NOT DWB:
DWB is gated in the full stack by G3 (batch_nav_test.sh) on the owner's box.
"""
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "src", "m20_ai_nav"))

import numpy as np  # noqa: E402

from m20_ai_nav import spec  # noqa: E402
from m20_ai_nav.sim.env import M20NavEnv  # noqa: E402

PASS_G1 = {"success": 0.95, "collision": 0.02, "spl": 0.80}


class Baseline:
    """Pick the clear 5-deg heading closest to the goal; speed by clearance."""

    def __call__(self, obs):
        scan = obs[:spec.N_BINS] * spec.SCAN_MAX
        d, s, c = obs[2 * spec.N_BINS:2 * spec.N_BINS + 3]
        goal_th = math.atan2(s, c)
        n = spec.N_BINS
        w = 2 * math.pi / n
        best, best_score = None, -1e9
        for k in range(n):
            th = math.atan2(math.sin(k * w), math.cos(k * w))
            # clearance over the robot's angular width at 1.2 m
            span = int(math.ceil(math.atan2(0.55, 1.2) / w))
            clear = min(scan[(k + j) % n] for j in range(-span, span + 1))
            if clear < min(1.2, d * spec.GOAL_MAX + 0.3):
                continue
            score = math.cos(th - goal_th)
            if score > best_score:
                best, best_score = th, score
        if best is None:
            return spec.cmd_to_action(0.0, 0.6)          # boxed in: rotate
        front = min(scan[i % n] for i in range(-6, 7))
        vx = 0.4 * max(0.0, math.cos(best)) * min(1.0, (front - 0.5) / 1.5)
        wz = max(-0.7, min(0.7, 1.5 * best))
        return spec.cmd_to_action(max(0.0, vx), wz)


class Onnx:
    def __init__(self, path):
        from m20_ai_nav.policy import OnnxPolicy
        self.p = OnnxPolicy(path)

    def __call__(self, obs):
        return self.p(obs)


def run_episode(env, pol, seed, options=None):
    obs, _ = env.reset(seed=seed, options=options)
    info = {}
    while True:
        obs, _, term, trunc, info = env.step(pol(obs))
        if term or trunc:
            break
    s = info["success"]
    l, p = info["start_dist"], info["path_len"]
    return {"success": s, "collision": info["collision"], "tipped": info["tipped"],
            "timeout": not (s or info["collision"] or info["tipped"]),
            "spl": (l / max(p, l)) if s else 0.0, "steps": env.steps}


def drive_navcore(env, pol, path, timeout_s=90.0):
    """Drive one episode the way the ROS node does: NavCore (carrot, safety envelope,
    blind-zone memory, pivot guard, stuck detection) + policy -> env.step(cmd)."""
    from m20_ai_nav.core import NavCore, SUCCEEDED, STUCK
    core = NavCore(pol); core.set_path(path)          # xy only: no final-yaw phase
    t, info, out = 0.0, {}, None
    plen = sum(math.hypot(b[0] - a_[0], b[1] - a_[1]) for a_, b in zip(path[:-1], path[1:]))
    while t < timeout_s:
        x, y, yaw = float(env.d.qpos[0]), float(env.d.qpos[1]), env.yaw()
        v = env.d.qvel
        cmd = core.tick(t, x, y, yaw, env.scan, math.cos(yaw) * v[0] + math.sin(yaw) * v[1], float(v[5]))
        if cmd.state in (SUCCEEDED, STUCK):
            out = cmd.state; break
        _, _, _, _, info = env.step(spec.cmd_to_action(cmd.vx, cmd.wz)); t += 0.1
        if info["collision"] or info["tipped"]:
            break
    ok = out == SUCCEEDED
    return {"success": ok, "collision": bool(info.get("collision")), "tipped": bool(info.get("tipped")),
            "timeout": not ok and not info.get("collision") and not info.get("tipped"),
            "spl": (plen / max(env.path_len, plen)) if ok else 0.0, "steps": int(t * 10)}


def run_office(pol, n, seed0=400000, door_range=(0.80, 1.20), n_people=None, timeout_s=120.0):
    """Office gates: held-out procedural offices, A* path (office planner radius)
    driven through NavCore, exactly like standalone/follow_path deployment."""
    env = M20NavEnv(scene="office", randomize=True, seed=seed0)
    rows = []
    for i in range(n):
        env.reset(seed=seed0 + i, options={"door_range": door_range, "n_people": n_people})
        rows.append(drive_navcore(env, pol, env.tracker.path.tolist(), timeout_s))
    return rows


def run_field(pol, n, seed0=300000, timeout_s=90.0):
    """Field scene AS DEPLOYED (standalone mode): A* on the scene's occupancy grid
    gives the path, NavCore + the policy drive it. Start/goal are sampled from the
    planner's FREE space (>= 0.8 m from any obstacle) and must be connected. The ground
    pipes fence the pad, so a straight-line goal across one is unreachable for a
    wheeled robot, and no deployed mode ever asks the bare policy to do that."""
    from m20_ai_nav.sim.raster import rasterize
    from m20_ai_nav.grid_planner import GridPlanner
    env = M20NavEnv(scene="field", randomize=True, max_steps=10 ** 9)
    env.reset(seed=seed0)
    import mujoco
    mujoco.mj_forward(env.m, env.d)
    pl = GridPlanner(); pl.set_map(*rasterize(env.m, env.d))
    rng = np.random.default_rng(seed0)
    free = np.argwhere(pl.dist > 0.8)
    rows = []
    while len(rows) < n:
        (r0, c0), (r1, c1) = free[rng.integers(len(free))], free[rng.integers(len(free))]
        s, g = pl.cell_to_world(r0, c0), pl.cell_to_world(r1, c1)
        if not (1.5 < math.hypot(g[0] - s[0], g[1] - s[1]) < 9.0) or max(map(abs, s + g)) > 8.5:
            continue
        path = pl.plan(*s, *g)
        if path is None:
            continue
        env.reset(seed=seed0 + len(rows), options={"start": s, "goal": g,
                                                    "yaw": float(rng.uniform(-math.pi, math.pi))})
        rows.append(drive_navcore(env, pol, path, timeout_s))
    return rows


def summarize(rows):
    a = {k: float(np.mean([r[k] for r in rows])) for k in
         ("success", "collision", "tipped", "timeout", "spl")}
    a["n"] = len(rows)
    return a


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--model"); g.add_argument("--baseline", action="store_true")
    ap.add_argument("--episodes", type=int, default=500)
    ap.add_argument("--gap-seeds", type=int, default=10)
    ap.add_argument("--field", type=int, default=50)
    ap.add_argument("--out", required=True)
    ap.add_argument("--only-field", action="store_true", help="re-run F only, merge into --out")
    ap.add_argument("--office", type=int, default=0, help="office gates O1-O3 with N episodes (merge into --out)")
    a = ap.parse_args()
    pol = Baseline() if a.baseline else Onnx(a.model)
    name = "baseline" if a.baseline else os.path.basename(a.model)
    t0 = time.time()
    res = {"policy": name, "date": time.strftime("%Y-%m-%d")}

    if a.office:
        res = json.load(open(a.out)) if os.path.exists(a.out) else {"policy": name}
        res["O1"] = summarize(run_office(pol, a.office))
        print("O1", json.dumps(res["O1"]), flush=True)
        res["O2"] = {}
        for w in (0.80, 0.85, 0.90, 1.00):
            res["O2"][f"{w:.2f}"] = summarize(run_office(pol, max(10, a.office // 4), seed0=500000,
                                                         door_range=(w, w), n_people=0))
            print("O2 door", w, json.dumps(res["O2"][f"{w:.2f}"]), flush=True)
        res["O3"] = summarize(run_office(pol, max(10, a.office // 2), seed0=600000, n_people=3))
        print("O3 people", json.dumps(res["O3"]), flush=True)
        json.dump(res, open(a.out, "w"), indent=2)
        return
    if a.only_field:
        res = json.load(open(a.out))
        res["field"] = summarize(run_field(pol, a.field))
        print("field", json.dumps(res["field"]), flush=True)
        json.dump(res, open(a.out, "w"), indent=2)
        return
    env = M20NavEnv(density=1.0, randomize=True)
    rows = [run_episode(env, pol, 100000 + i) for i in range(a.episodes)]
    res["G1"] = summarize(rows)
    res["G1"]["pass"] = bool(res["G1"]["success"] >= PASS_G1["success"]
                             and res["G1"]["collision"] <= PASS_G1["collision"]
                             and res["G1"]["spl"] >= PASS_G1["spl"])
    print("G1", json.dumps(res["G1"]), flush=True)

    res["G2"] = {}
    for w in (1.0, 1.1, 1.2, 1.4, 1.6, 2.0):
        rows = [run_episode(env, pol, 200000 + i,
                            {"gap": w, "start": (0.0, 0.0), "yaw": 0.0, "goal": (5.0, 0.0)})
                for i in range(a.gap_seeds)]
        res["G2"][f"{w:.1f}"] = summarize(rows)
        print("G2 gap", w, json.dumps(res["G2"][f"{w:.1f}"]), flush=True)

    res["field"] = summarize(run_field(pol, a.field))
    print("field", json.dumps(res["field"]), flush=True)
    res["wall_s"] = round(time.time() - t0)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    json.dump(res, open(a.out, "w"), indent=2)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
