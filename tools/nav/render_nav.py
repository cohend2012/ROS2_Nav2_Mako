#!/usr/bin/env python3
"""Honest verification of the Nav2 + slam_toolbox run.
Left  : the oil&gas field (true obstacle geometry from the MJCF) + Nav2 global plan
        + the robot's ACTUAL ground-truth trajectory + start/goal.
Right : the occupancy map slam_toolbox actually built from the LiDAR, trajectory overlaid.
"""
import csv, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

OUT = os.environ.get("OUTDIR", "/data/out")

# --- true field obstacles (from tools/sim/oil_gas_field.xml), at the 0.35 m scan height ---
# circles: (x, y, r) ; rects: (x, y, hx, hy)  -- only those the 2D scan sees (top >= 0.35)
TANKS = [(-5, 4, 1.4), (-1, 4.5, 1.4)]
WELL_STACK = [(6, -1, 0.12)]
RECTS = [(-4, -1, 0.5, 0.7, "skid1"), (0, 1.5, 0.4, 0.4, "skid2"),
         (6, -1, 0.4, 0.4, "wellhead"), (5, 2, 0.1, 0.1, "rack_post"),
         (7, 2, 0.1, 0.1, "rack_post")]
# ground pipelines (tops at 0.30 m -> BELOW the 2D scan, drawn dashed to show 2D misses them)
PIPES = [((-8, -3), (8, -3)), ((-8, -3.4), (8, -3.4)), ((2, -3.4), (2, 3))]

def load_csv(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        r = csv.reader(f); next(r, None)
        return [tuple(float(v) for v in row) for row in r if row]

traj = load_csv(f"{OUT}/traj.csv")     # (t, x, y)
plan = load_csv(f"{OUT}/plan.csv")     # (x, y)
tx = [p[1] for p in traj]; ty = [p[2] for p in traj]
px = [p[0] for p in plan]; py = [p[1] for p in plan]

GOAL = (-6.0, -0.5)
START = (tx[0], ty[0]) if traj else (1.0, 0.2)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))

# ---------- LEFT: world + plan + actual path ----------
for (x, y, r) in TANKS:
    ax1.add_patch(Circle((x, y), r, color="#b0a070", alpha=0.8))
    ax1.text(x, y, "tank", ha="center", va="center", fontsize=8)
for (x, y, r) in WELL_STACK:
    ax1.add_patch(Circle((x, y), max(r, 0.2), color="#b04030", alpha=0.8))
for (x, y, hx, hy, name) in RECTS:
    c = "#c04030" if name == "skid1" else "#7088a0"
    ax1.add_patch(Rectangle((x - hx, y - hy), 2 * hx, 2 * hy, color=c, alpha=0.85))
    ax1.text(x, y - hy - 0.25, name, ha="center", va="top", fontsize=7)
for (a, b) in PIPES:
    ax1.plot([a[0], b[0]], [a[1], b[1]], "--", color="#888", lw=3, alpha=0.6)
ax1.plot([], [], "--", color="#888", lw=3, label="ground pipes (below 2D scan)")
if plan:
    ax1.plot(px, py, "-", color="#1f77b4", lw=2.5, label="Nav2 global plan")
if traj:
    ax1.plot(tx, ty, "-", color="#2ca02c", lw=2.5, label="actual path (ground truth)")
ax1.plot(*START, "o", color="k", ms=12, label="start")
ax1.plot(*GOAL, "X", color="red", ms=16, label="goal")
if traj:
    ax1.plot(tx[-1], ty[-1], "*", color="#2ca02c", ms=20, label="final pose")
ax1.set_title("Autonomous nav: Nav2 plan vs actual path, avoiding skid1", fontsize=12)
ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
ax1.set_xlim(-9, 9); ax1.set_ylim(-5, 6); ax1.set_aspect("equal"); ax1.grid(alpha=0.3)
ax1.legend(loc="upper right", fontsize=8)

# ---------- RIGHT: the SLAM map the robot actually built ----------
mp = f"{OUT}/map.npz"
if os.path.exists(mp):
    d = np.load(mp)
    grid = d["grid"]; res = float(d["res"]); ox = float(d["ox"]); oy = float(d["oy"])
    h, w = grid.shape
    disp = np.full_like(grid, 205, dtype=np.uint8)   # unknown = grey
    disp[grid == 0] = 255                             # free = white
    disp[grid > 50] = 0                               # occupied = black
    extent = [ox, ox + w * res, oy, oy + h * res]
    ax2.imshow(disp, cmap="gray", origin="lower", extent=extent, vmin=0, vmax=255)
    if traj:
        ax2.plot(tx, ty, "-", color="#2ca02c", lw=2, label="actual path")
    ax2.plot(*GOAL, "X", color="red", ms=14)
    ax2.set_title("slam_toolbox occupancy map (built from LiDAR)", fontsize=12)
    ax2.set_xlabel("x (m)"); ax2.set_ylabel("y (m)"); ax2.set_aspect("equal")
    ax2.legend(loc="upper right", fontsize=8)
else:
    ax2.text(0.5, 0.5, "no map captured", ha="center")

plt.tight_layout()
plt.savefig(f"{OUT}/nav_result.png", dpi=110)
print(f"wrote {OUT}/nav_result.png  traj_pts={len(traj)} plan_pts={len(plan)}")
