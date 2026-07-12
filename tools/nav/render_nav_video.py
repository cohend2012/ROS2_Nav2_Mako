#!/usr/bin/env python3
"""Animated GIF of the REAL logged Nav2 run: robot driving the field, trailing path."""
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

with open(f"{OUT}/traj.csv") as f:
    r = csv.reader(f); next(r)
    traj = [(float(a[1]), float(a[2])) for a in r if a]
# downsample to ~140 frames
step = max(1, len(traj) // 140)
pts = traj[::step]
tx = [p[0] for p in pts]; ty = [p[1] for p in pts]

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
ax.plot(tx[0], ty[0], "o", color="k", ms=10, label="start")
trail, = ax.plot([], [], "-", color="#2ca02c", lw=2.5, label="path driven")
robot, = ax.plot([], [], "o", color="#2ca02c", ms=13, mec="k")
ax.set_xlim(-9, 9); ax.set_ylim(-5, 6); ax.set_aspect("equal"); ax.grid(alpha=0.3)
ax.set_title("M20 autonomous nav (Nav2 + slam_toolbox) — oil & gas field")
ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.legend(loc="upper right", fontsize=8)

def upd(i):
    trail.set_data(tx[:i+1], ty[:i+1])
    robot.set_data([tx[i]], [ty[i]])
    return trail, robot

ani = animation.FuncAnimation(fig, upd, frames=len(pts), interval=80, blit=True)
ani.save(f"{OUT}/nav_run.gif", writer=animation.PillowWriter(fps=14))
print(f"wrote {OUT}/nav_run.gif  frames={len(pts)}")
