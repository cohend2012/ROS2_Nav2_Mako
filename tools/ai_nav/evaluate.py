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
F : the real oil_gas_field.xml scene, random collision-free start/goal pairs.

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
    a = ap.parse_args()
    pol = Baseline() if a.baseline else Onnx(a.model)
    name = "baseline" if a.baseline else os.path.basename(a.model)
    t0 = time.time()
    res = {"policy": name, "date": time.strftime("%Y-%m-%d")}

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

    fenv = M20NavEnv(scene="field", randomize=True)
    rows = [run_episode(fenv, pol, 300000 + i) for i in range(a.field)]
    res["field"] = summarize(rows)
    print("field", json.dumps(res["field"]), flush=True)
    res["wall_s"] = round(time.time() - t0)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    json.dump(res, open(a.out, "w"), indent=2)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
