#!/usr/bin/env python3
"""Honest verification of the Nav2 + slam_toolbox run WITH drifting odometry.
Left  : oil&gas field (true obstacle geometry) + TRUE path (green) + RAW dead-reckoned
        odom (orange, drifts) + SLAM-corrected estimate (blue dashed) + start/goal.
Right : the occupancy map slam_toolbox built, with the true path overlaid.
The orange-vs-green gap = drift SLAM had to fight; blue-near-green = SLAM working.
"""
import csv, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

OUT = os.environ.get("OUTDIR", "/data/out")
TANKS = [(-5, 4, 1.4), (-1, 4.5, 1.4)]
RECTS = [(-4, -1, 0.5, 0.7, "skid1"), (0, 1.5, 0.4, 0.4, "skid2"),
         (6, -1, 0.4, 0.4, "wellhead"), (5, 2, 0.1, 0.1, ""), (7, 2, 0.1, 0.1, "")]
PIPES = [((-8, -3), (8, -3)), ((-8, -3.4), (8, -3.4)), ((2, -3.4), (2, 3))]
GOAL = (-6.0, -0.5)

def load(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        r = csv.reader(f); hdr = next(r, None)
        return [tuple(float(v) for v in row) for row in r if row]

def xy(rows, has_t=True):
    i = (1, 2) if has_t else (0, 1)
    return [p[i[0]] for p in rows], [p[i[1]] for p in rows]

true = load(f"{OUT}/traj_true.csv");  txx, tyy = xy(true)
drift = load(f"{OUT}/traj_drift.csv"); dxx, dyy = xy(drift)
est = load(f"{OUT}/traj_est.csv");    exx, eyy = xy(est)
plan = load(f"{OUT}/plan.csv");       pxx, pyy = xy(plan, has_t=False)

def err(a, b):
    return np.hypot(a[0]-b[0], a[1]-b[1]) if a and b else float("nan")

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

for (x, y, rad) in TANKS:
    ax1.add_patch(Circle((x, y), rad, color="#b0a070", alpha=0.8))
    ax1.text(x, y, "tank", ha="center", va="center", fontsize=8)
for (x, y, hx, hy, name) in RECTS:
    c = "#c04030" if name == "skid1" else "#7088a0"
    ax1.add_patch(Rectangle((x-hx, y-hy), 2*hx, 2*hy, color=c, alpha=0.85))
    if name:
        ax1.text(x, y-hy-0.25, name, ha="center", va="top", fontsize=7)
for (a, b) in PIPES:
    ax1.plot([a[0], b[0]], [a[1], b[1]], "--", color="#888", lw=3, alpha=0.5)
ax1.plot([], [], "--", color="#888", lw=3, label="ground pipes (below 2D scan)")
if drift: ax1.plot(dxx, dyy, "-", color="#ff7f0e", lw=2, alpha=0.9, label="raw wheel odom (drifts, uncorrected)")
if est:   ax1.plot(exx, eyy, "--", color="#1f77b4", lw=2, label="SLAM-corrected estimate (Nav2 uses)")
if true:  ax1.plot(txx, tyy, "-", color="#2ca02c", lw=2.8, label="TRUE path (ground truth)")
if plan:  ax1.plot(pxx, pyy, ":", color="#111", lw=1.5, alpha=0.7, label="Nav2 plan (final)")
ax1.plot(0, 0, "o", color="k", ms=11, label="start")
ax1.plot(*GOAL, "X", color="red", ms=16, label="goal")
if true:  ax1.plot(txx[-1], tyy[-1], "*", color="#2ca02c", ms=20)
te = err((txx[-1], tyy[-1]), GOAL) if true else float("nan")
de = err((dxx[-1], dyy[-1]), (txx[-1], tyy[-1])) if (drift and true) else float("nan")
ax1.set_title(f"Honest nav (drifting odom): TRUE reaches goal within {te:.2f} m\n"
              f"raw-odom drift at end = {de:.2f} m (what SLAM had to correct)", fontsize=11)
ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
ax1.set_xlim(-9, 9); ax1.set_ylim(-5, 6); ax1.set_aspect("equal"); ax1.grid(alpha=0.3)
ax1.legend(loc="upper right", fontsize=8)

mp = f"{OUT}/map.npz"
if os.path.exists(mp):
    d = np.load(mp)
    grid = d["grid"]; res = float(d["res"]); ox = float(d["ox"]); oy = float(d["oy"])
    h, w = grid.shape
    disp = np.full_like(grid, 205, dtype=np.uint8)
    disp[grid == 0] = 255; disp[grid > 50] = 0
    ax2.imshow(disp, cmap="gray", origin="lower", extent=[ox, ox+w*res, oy, oy+h*res], vmin=0, vmax=255)
    if true: ax2.plot(txx, tyy, "-", color="#2ca02c", lw=2, label="true path")
    ax2.plot(*GOAL, "X", color="red", ms=14)
    ax2.set_title("slam_toolbox occupancy map (built from LiDAR)", fontsize=12)
    ax2.set_xlabel("x (m)"); ax2.set_ylabel("y (m)"); ax2.set_aspect("equal")
    ax2.legend(loc="upper right", fontsize=8)

plt.tight_layout()
plt.savefig(f"{OUT}/nav_result.png", dpi=110)
print(f"true_err_to_goal={te:.2f}m  drift_at_end={de:.2f}m  "
      f"pts true={len(true)} drift={len(drift)} est={len(est)}")
