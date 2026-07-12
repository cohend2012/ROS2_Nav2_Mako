#!/usr/bin/env python3
"""Animated GIF of the REAL logged Nav2 run with drifting odometry:
robot driving the field (TRUE, green) vs where raw dead-reckoning THINKS it is (orange).
The gap = drift that slam_toolbox corrects so the robot still reaches the true goal."""
import csv, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import matplotlib.animation as animation

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
        r = csv.reader(f); next(r)
        return [(float(a[1]), float(a[2])) for a in r if a]

true = load(f"{OUT}/traj_true.csv")
drift = load(f"{OUT}/traj_drift.csv")
step = max(1, len(true) // 140)
tp = true[::step]; dp = drift[::step]
n = min(len(tp), len(dp))
tp, dp = tp[:n], dp[:n]

fig, ax = plt.subplots(figsize=(9, 6.5))
for (x, y, rad) in TANKS:
    ax.add_patch(Circle((x, y), rad, color="#b0a070", alpha=0.8))
for (x, y, hx, hy, name) in RECTS:
    c = "#c04030" if name == "skid1" else "#7088a0"
    ax.add_patch(Rectangle((x-hx, y-hy), 2*hx, 2*hy, color=c, alpha=0.85))
    if name:
        ax.text(x, y-hy-0.25, name, ha="center", va="top", fontsize=7)
for (a, b) in PIPES:
    ax.plot([a[0], b[0]], [a[1], b[1]], "--", color="#888", lw=3, alpha=0.5)
ax.plot(*GOAL, "X", color="red", ms=16, label="goal")
ax.plot(0, 0, "o", color="k", ms=10, label="start")
dtrail, = ax.plot([], [], "-", color="#ff7f0e", lw=2, alpha=0.9, label="raw odom (drifts)")
drob, = ax.plot([], [], "o", color="#ff7f0e", ms=9, mec="k")
ttrail, = ax.plot([], [], "-", color="#2ca02c", lw=2.6, label="TRUE (SLAM keeps on track)")
trob, = ax.plot([], [], "o", color="#2ca02c", ms=13, mec="k")
ax.set_xlim(-9, 9); ax.set_ylim(-5, 6); ax.set_aspect("equal"); ax.grid(alpha=0.3)
ax.set_title("M20 autonomous nav (Nav2 + slam_toolbox, drifting odom) — oil & gas field")
ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.legend(loc="upper right", fontsize=8)

def upd(i):
    ttrail.set_data([p[0] for p in tp[:i+1]], [p[1] for p in tp[:i+1]])
    trob.set_data([tp[i][0]], [tp[i][1]])
    dtrail.set_data([p[0] for p in dp[:i+1]], [p[1] for p in dp[:i+1]])
    drob.set_data([dp[i][0]], [dp[i][1]])
    return ttrail, trob, dtrail, drob

ani = animation.FuncAnimation(fig, upd, frames=n, interval=80, blit=True)
ani.save(f"{OUT}/nav_run.gif", writer=animation.PillowWriter(fps=14))
print(f"wrote {OUT}/nav_run.gif  frames={n}")
