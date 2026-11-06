#!/usr/bin/env python3
"""Where each dial detector works, measured in the sim (no training poses):

    MUJOCO_GL=osmesa python3 tools/perception/eval_dial_detector.py [--out result.json]

1) Envelope: for every gauge, camera placed on a grid of distance (1-6 m) x angle off the
   face (0-65 deg, both sides) x position in the image (0/12/24 deg off-centre); only
   poses where the plate is really visible count. Hit = a detection on that plate.
2) False alarms: 300 random frames on the site and 300 in the office (no dials); a
   detection that matches no labelled plate counts. On the site these are almost all
   plate BACKS / edge-on plates (the labeller only counts a plate whose face is towards the camera).
"""
import argparse
import json
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "src", "m20_ai_nav"))
sys.path.insert(0, os.path.join(REPO, "src", "m20_agent"))

import numpy as np  # noqa: E402

from make_dial_dataset import F, H, W, Scene  # noqa: E402
from m20_agent.dial_detector import make_detector  # noqa: E402

DISTS, OFFS, BEARS = (1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0), (0, 20, 35, 50, 65), (0, 12, 24)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out")
    a = ap.parse_args()
    dets = {"classical": make_detector("classical")}
    if os.path.exists(os.path.join(REPO, "src", "m20_agent", "models", "dial_yolo.onnx")):
        dets["yolo"] = make_detector("yolo")
    sc = Scene("oil_gas_field_gauges.xml", 11)
    rng = random.Random(5)
    hits = {k: {} for k in dets}
    ms = {k: [] for k in dets}
    for g in sc.plates:
        c = sc.d.geom_xpos[g].copy()
        n = sc.d.geom_xmat[g].reshape(3, 3)[:2, 2]
        n = n / np.linalg.norm(n)
        for dist in DISTS:
            for off in OFFS:
                for side in (1, -1):
                    for bear in BEARS:
                        ang = math.atan2(n[1], n[0]) + side * math.radians(off)
                        x, y = c[0] + (dist + 0.3) * math.cos(ang), c[1] + (dist + 0.3) * math.sin(ang)
                        r_, c_ = sc.planner.world_to_cell(x, y)
                        if not (0 <= r_ < sc.planner.h and 0 <= c_ < sc.planner.w) or sc.planner.dist[r_, c_] < 0.45:
                            continue
                        sc.place(x, y, math.atan2(c[1] - y, c[0] - x) + math.radians(bear * rng.choice((1, -1))))
                        v = c - sc.eye
                        fw = v @ sc.fwd
                        if fw < 0.3:
                            continue
                        pu, pv = W / 2 - F * (v @ sc.left) / fw, H / 2 - F * (v @ sc.up) / fw
                        if not any(abs(b[0] * W - pu) < 8 and abs(b[1] * H - pv) < 8 for b in sc.boxes()):
                            continue
                        img = sc.render()
                        for k, d in dets.items():
                            t = time.time(); out = d(img); ms[k].append(1000 * (time.time() - t))
                            hit = any(math.hypot(o[0] - pu, o[1] - pv) < max(6, 0.6 * o[2]) for o in out)
                            hits[k].setdefault((dist, off, bear), []).append(hit)
    res = {}
    for k in dets:
        H_ = hits[k]
        res[k] = {"ms_per_frame": round(float(np.mean(ms[k])), 1), "poses": sum(len(v) for v in H_.values()),
                  "by_distance_m": {d: round(float(np.mean([h for (dd, _, _), v in H_.items() if dd == d for h in v])), 3) for d in DISTS},
                  "by_off_axis_deg": {o: round(float(np.mean([h for (_, oo, _), v in H_.items() if oo == o for h in v])), 3) for o in OFFS},
                  "by_image_bearing_deg": {b: round(float(np.mean([h for (_, _, bb), v in H_.items() if bb == b for h in v])), 3) for b in BEARS}}
    for name in ("oil_gas_field_gauges.xml", "office.xml"):
        s2 = Scene(name, 21); r2 = random.Random(9)
        fa = {k: 0 for k in dets}
        for _ in range(300):
            x, y = r2.choice(s2.free); s2.place(x, y, r2.uniform(-math.pi, math.pi))
            boxes = s2.boxes() if "gauges" in name else []
            img = s2.render()
            for k, d in dets.items():
                fa[k] += sum(1 for o in d(img) if not any(abs(o[0] - b[0] * W) < max(8, b[2] * W)
                                                          and abs(o[1] - b[1] * H) < max(8, b[3] * H) for b in boxes))
        for k in dets:
            res[k][f"false_alarms_per_300_frames_{name.split('.')[0]}"] = fa[k]
    print(json.dumps(res, indent=1))
    if a.out:
        json.dump(res, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
