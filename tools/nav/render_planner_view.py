#!/usr/bin/env python3
"""Render the PLANNER'S view of a logged run as an MP4: occupancy map, global
plan, local (DWB) plan, live laser hits, robot pose + trail, goal.

Companion to replay_render.py (the cinematic MuJoCo view). Demo videos pair
them:  ffmpeg -i replay.mp4 -i nav.mp4 -filter_complex hstack out.mp4

Inputs (from nav_logger in OUTDIR): map.npz, traj_true.csv, traj_est.csv,
plan.csv, scans.csv, local_plans.jsonl.
Env: OUTDIR (default .), OUTMP4 (default OUTDIR/nav_view.mp4),
     GOAL_X/GOAL_Y (default -6.0/-0.5), FPS (default 5).
"""
import csv, json, math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import imageio.v2 as imageio

OUTDIR = os.environ.get("OUTDIR", ".")
OUTMP4 = os.environ.get("OUTMP4", f"{OUTDIR}/nav_view.mp4")
GOAL = (float(os.environ.get("GOAL_X", "-6.0")), float(os.environ.get("GOAL_Y", "-0.5")))
FPS = int(os.environ.get("FPS", "5"))
ANG = np.radians(np.arange(-180.0, 180.0, 1.0))   # matches pc2ls: 1 deg bins from -pi


def rows(path, ncols=None):
    out = []
    if not os.path.exists(path):
        return out
    with open(path) as f:
        for a in csv.reader(f):
            try:
                out.append([float(v) for v in (a[:ncols] if ncols else a)])
            except ValueError:
                pass   # header line
    return out


true = rows(f"{OUTDIR}/traj_true.csv", 4)
est = rows(f"{OUTDIR}/traj_est.csv", 3)
plan = rows(f"{OUTDIR}/plan.csv", 2)
scans = rows(f"{OUTDIR}/scans.csv")
lplans = []
if os.path.exists(f"{OUTDIR}/local_plans.jsonl"):
    with open(f"{OUTDIR}/local_plans.jsonl") as f:
        lplans = [json.loads(l) for l in f if l.strip()]
if os.path.exists(f"{OUTDIR}/map.npz"):
    m = np.load(f"{OUTDIR}/map.npz")
    grid, res, ox, oy = m["grid"], float(m["res"]), float(m["ox"]), float(m["oy"])
    H, W = grid.shape
    img = np.full((H, W), 0.82)
    img[grid == 0] = 1.0
    img[grid > 50] = 0.15
else:
    # localization mode: the logger gets no /map — use the SHIPPED map artifact
    # (it is exactly the map the planner navigated against)
    import yaml
    prefix = os.environ.get("MAP_PREFIX",
                            os.path.join(os.path.dirname(__file__), "../..",
                                         "maps/oil_gas_field"))
    meta = yaml.safe_load(open(prefix + ".yaml"))
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    pix = imageio.imread(prefix + ".pgm")
    img = np.flipud(pix.astype(float) / 255.0)   # pgm row 0 = top; imshow origin=lower
    H, W = img.shape
extent = [ox, ox + W * res, oy, oy + H * res]

t0, t_end = true[0][0], true[-1][0]
frames_t = np.arange(0.0, t_end - t0, 1.0 / FPS)


import bisect
_tcache = {}
def latest(seq, tt):
    """Last element with time <= t0+tt (bisect; the linear walk was
    O(frames*rows) and timed out on the 67k-row patrol log)."""
    if not seq:
        return None
    key = id(seq)
    if key not in _tcache:
        _tcache[key] = [(e["t"] if isinstance(e, dict) else e[0]) - t0 for e in seq]
    i = bisect.bisect(_tcache[key], tt) - 1
    return seq[i] if i >= 0 else None


fig, ax = plt.subplots(figsize=(9.6, 5.4), dpi=100)
writer = imageio.get_writer(OUTMP4, fps=FPS, macro_block_size=1)
trail_x, trail_y = [], []
for tt in frames_t:
    ax.clear()
    ax.imshow(img, cmap="gray", vmin=0, vmax=1, extent=extent, origin="lower")
    if plan:
        ax.plot([p[0] for p in plan], [p[1] for p in plan], "-", lw=1.6,
                color="#1f77e0", label="global plan (NavFn)")
    lp = latest(lplans, tt) if lplans else None
    if lp:
        pts = lp["pts"]
        ax.plot(pts[0::2], pts[1::2], "-", lw=2.4, color="#ff9500",
                label="local plan (DWB)")
    sc = latest(scans, tt)
    if sc and len(sc) > 8:
        sx, sy, syaw, rr = sc[1], sc[2], sc[3], np.array(sc[4:])
        n_ang = min(len(rr), len(ANG))
        rr = rr[:n_ang]
        ok = np.isfinite(rr) & (rr > 0.05) & (rr < 11.9)
        a = ANG[:n_ang][ok] + syaw
        d = rr[ok]
        ax.plot(sx + d * np.cos(a), sy + d * np.sin(a), ".", ms=2.0,
                color="#d62728", label="laser")
    p = latest(true, tt)
    if p:
        trail_x.append(p[1]); trail_y.append(p[2])
        ax.plot(trail_x, trail_y, "-", lw=1.0, color="#2ca02c", alpha=0.8)
        ax.plot(p[1], p[2], "o", ms=7, color="#2ca02c", label="robot (true)")
        ax.arrow(p[1], p[2], 0.7 * math.cos(p[3]), 0.7 * math.sin(p[3]),
                 head_width=0.22, color="#2ca02c")
    # interpolate est between samples (sample-hold made it trail even more
    # than its real ~0.8 s latency — measured 2026-08-03)
    e = latest(est, tt)
    if e:
        i = est.index(e)
        if i + 1 < len(est) and est[i+1][0] > e[0]:
            f = min(1.0, (tt - (e[0] - t0)) / (est[i+1][0] - e[0]))
            ex, ey = e[1] + f*(est[i+1][1]-e[1]), e[2] + f*(est[i+1][2]-e[2])
        else:
            ex, ey = e[1], e[2]
        ax.plot(ex, ey, "x", ms=7, color="#9467bd", label="robot (est)")
    ax.plot(*GOAL, "*", ms=15, color="#d62728", label="goal")
    ax.set_xlim(extent[0], extent[1]); ax.set_ylim(extent[2], extent[3])
    ax.set_title(f"planner view  t={tt:5.1f}s")
    ax.legend(loc="lower right", fontsize=7, framealpha=0.85)
    ax.set_aspect("equal")
    fig.canvas.draw()
    writer.append_data(np.asarray(fig.canvas.buffer_rgba())[:, :, :3])
writer.close()
print(f"wrote {OUTMP4} frames={len(frames_t)} dur={t_end - t0:.1f}s")
