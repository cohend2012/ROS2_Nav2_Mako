#!/usr/bin/env python3
"""Top-down picture of a find_gauges mission: map, path driven, survey points, the TRUE
gauges (never shown to the robot) and what the robot found, plus its up-close photos.

    python3 tools/agent/plot_find_gauges.py <mission out dir> [--seed N]
Writes <dir>/find_gauges_map.png and <dir>/find_gauges_photos.png.
"""
import argparse
import glob
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "src", "m20_ai_nav"))
sys.path.insert(0, os.path.join(REPO, "src", "m20_agent"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from m20_ai_nav.sim.env import M20NavEnv  # noqa: E402
from m20_ai_nav.sim.raster import rasterize  # noqa: E402

TRUTH = os.path.join(REPO, "tools", "sim", "gauges", "truth.json")
BOUNDS = (-8.0, 8.0, -3.2, 7.0)
PX = 60                                     # pixels per metre


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    rep = json.load(open(sorted(glob.glob(os.path.join(a.out, "report_*.json")))[-1]))
    trace = json.load(open(os.path.join(a.out, "trace.json")))
    truth = json.load(open(TRUTH))
    env = M20NavEnv(scene="oil_gas_field_gauges.xml", randomize=False, seed=a.seed, max_steps=10)
    env.reset(seed=a.seed, options={"start": (0, 0), "yaw": 0, "goal": (0, 0)})
    mujoco.mj_forward(env.m, env.d)
    data, w, h, res, ox, oy = rasterize(env.m, env.d)
    occ = np.asarray(data).reshape(h, w)

    x0, x1, y0, y1 = BOUNDS
    W, H = int((x1 - x0) * PX), int((y1 - y0) * PX)
    img = Image.new("RGB", (W, H + 90), (250, 250, 247))
    dr = ImageDraw.Draw(img)
    to = lambda x, y: ((x - x0) * PX, (y1 - y) * PX)  # noqa: E731
    ys, xs = np.nonzero(occ >= 50)
    for r, c in zip(ys, xs):
        wx, wy = ox + (c + 0.5) * res, oy + (r + 0.5) * res
        if x0 <= wx <= x1 and y0 <= wy <= y1:
            px, py = to(wx, wy)
            dr.rectangle([px - res * PX / 2, py - res * PX / 2, px + res * PX / 2, py + res * PX / 2],
                         fill=(150, 150, 150))
    for gx in range(int(x0), int(x1) + 1):
        dr.line([to(gx, y0), to(gx, y1)], fill=(232, 232, 228))
    for gy in range(int(y0), int(y1) + 1):
        dr.line([to(x0, gy), to(x1, gy)], fill=(232, 232, 228))
    pts = [to(t[1], t[2]) for t in trace[::3]]
    dr.line(pts, fill=(70, 130, 200), width=2)
    fg = rep.get("find_gauges", {})
    font = ImageFont.load_default(size=15)
    small = ImageFont.load_default(size=12)
    found = [r for r in rep["readings"] if r.get("found")]
    for name, t in truth.items():
        px, py = to(*t["plate"][:2])
        dr.ellipse([px - 11, py - 11, px + 11, py + 11], outline=(200, 40, 40), width=3)
        tf = (t["value"] - t["min"]) / (t["max"] - t["min"])
        dr.text((px + 14, py - 22), f"{name} true {tf:.2f}", fill=(200, 40, 40), font=small)
    for r in found:
        px, py = to(*r["position"])
        dr.line([px - 7, py - 7, px + 7, py + 7], fill=(20, 140, 60), width=3)
        dr.line([px - 7, py + 7, px + 7, py - 7], fill=(20, 140, 60), width=3)
        val = f"{r['value']:.2f}" if r.get("readable") else "-"
        dr.text((px + 14, py + 4), f"{r['gauge']} read {val} {r['status']}", fill=(20, 140, 60), font=small)
    hx, hy = rep["home"][:2]
    px, py = to(hx, hy)
    dr.rectangle([px - 7, py - 7, px + 7, py + 7], fill=(30, 30, 30))
    dr.text((px + 10, py + 6), "start / home", fill=(30, 30, 30), font=small)
    ex, ey = trace[-1][1], trace[-1][2]
    px, py = to(ex, ey)
    dr.ellipse([px - 4, py - 4, px + 4, py + 4], fill=(70, 130, 200))
    g6 = rep.get("G6", {})
    sim = rep.get("sim", {})
    dr.text((10, H + 8), f"request: \"{rep['request']}\"   plan: "
            f"{' -> '.join(s['action'] for s in rep.get('plan', []))}   mission: "
            f"{'SUCCEEDED' if rep['success'] else 'FAILED'}", fill=(0, 0, 0), font=font)
    dr.text((10, H + 32), f"red circle = true gauge (hidden from robot)   green x = found by robot   "
            f"blue = path driven", fill=(0, 0, 0), font=small)
    dr.text((10, H + 54), f"found {g6.get('found')}/{g6.get('of')}, read within 5% "
            f"{g6.get('read_within')}, false positives {g6.get('false_positives')}, "
            f"survey points {fg.get('survey_points', 0) - fg.get('unreached_survey_points', 0)}/"
            f"{fg.get('survey_points')}, back home within {sim.get('home_error_m')} m, "
            f"{sim.get('sim_time_s')} s sim", fill=(0, 0, 0), font=small)
    img.save(os.path.join(a.out, "find_gauges_map.png"))

    tiles = []
    for r in found:
        if r.get("image") and os.path.exists(r["image"]):
            t = Image.open(r["image"]).convert("RGB")
            d2 = ImageDraw.Draw(t)
            val = f"{r['value']:.2f}" if r.get("readable") else "unreadable"
            d2.rectangle([0, 0, 320, 18], fill=(0, 0, 0))
            d2.text((4, 2), f"{r['gauge']}: {val} ({r['status']}, conf {r.get('confidence')})",
                    fill=(255, 255, 255), font=small)
            tiles.append(t)
    if tiles:
        cols = 3
        sheet = Image.new("RGB", (320 * cols, 240 * math.ceil(len(tiles) / cols)), (0, 0, 0))
        for i, t in enumerate(tiles):
            sheet.paste(t, ((i % cols) * 320, (i // cols) * 240))
        sheet.save(os.path.join(a.out, "find_gauges_photos.png"))
    print("wrote", os.path.join(a.out, "find_gauges_map.png"))


if __name__ == "__main__":
    main()
