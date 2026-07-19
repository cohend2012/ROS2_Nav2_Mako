#!/usr/bin/env python3
"""GPS-in-the-loop video: MuJoCo replay of the REAL logged run (left) beside a live
estimate-race panel (right):
  GREEN  = ground truth            (what actually happened)
  ORANGE = dead-reckoned odometry  (drifts, uncorrected)
  RED  x = raw GPS fixes           (noisy absolutes, 5 Hz, sigma 0.8 m)
  BLUE   = GPS-fused EKF           (/odom_filtered — wheel+IMU+GPS; Phase A preview)
All four are real logged signals from one run — nothing re-simulated.
Usage: OUTDIR=<logdir> OUTMP4=<file> python3 render_gps_video.py   (host, egl)
"""
import os, csv, math
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np, mujoco, imageio
from pathlib import Path
from PIL import Image, ImageDraw

OUTDIR = os.environ.get("OUTDIR", ".")
OUTMP4 = os.environ.get("OUTMP4", f"{OUTDIR}/gps_run.mp4")
GOAL = (float(os.environ.get("GOAL_X", "-6.0")), float(os.environ.get("GOAL_Y", "-0.5")))
FPS = 20
M = Path.home()/"m20_sim/sdk_deploy/src/M20_sdk_deploy/M20_description/m20_mjcf/mjcf/oil_gas_field.xml"
STAND = np.array([0,-0.7,1.4,0, 0,-0.7,1.4,0, 0,0.7,-1.4,0, 0,0.7,-1.4,0], float)
WHEELS = (3, 7, 11, 15)
W3D, WP, H = 820, 460, 560            # 3D view + plot panel widths, height

def load(name, ncol):
    p = f"{OUTDIR}/{name}"
    if not os.path.exists(p):
        return []
    with open(p) as f:
        r = csv.reader(f); next(r, None)
        return [tuple(float(v) for v in row[:ncol]) for row in r if row]

true = load("traj_true.csv", 4)        # t x y yaw
drift = load("traj_drift.csv", 3)
gps = load("gps.csv", 3)
ekf = load("traj_ekf.csv", 3)
if not true:
    raise SystemExit("no traj_true.csv — run the instrumented bringup first")
t0, t1 = true[0][0], true[-1][0]
frames_t = np.arange(0.0, t1 - t0, 1.0 / FPS)

def upto(rows, tt):
    return [r for r in rows if r[0] - t0 <= tt]

def at(rows, tt):
    lo, hi = 0, len(rows) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if rows[mid][0] - t0 < tt: lo = mid + 1
        else: hi = mid
    return rows[lo]

# --- MuJoCo replay setup (same approach as replay_render.py) ---
m = mujoco.MjModel.from_xml_path(str(M))
m.vis.global_.offwidth = W3D; m.vis.global_.offheight = H
d = mujoco.MjData(m)
d.qpos[:] = 0; d.qpos[3] = 1.0; d.qpos[7:23] = STAND
d.qpos[2] = 0.6; mujoco.mj_forward(m, d)
z_stand = 0.6 - float(d.geom_xpos[:, 2].min()) + 0.02
rend = mujoco.Renderer(m, H, W3D)
cam = mujoco.MjvCamera(); cam.distance = 8.5; cam.azimuth = 120; cam.elevation = -28

def set_pose(x, y, yaw, wa):
    d.qpos[:] = 0
    d.qpos[0], d.qpos[1], d.qpos[2] = x, y, z_stand
    d.qpos[3] = math.cos(yaw/2); d.qpos[6] = math.sin(yaw/2)
    d.qpos[7:23] = STAND
    for w in WHEELS: d.qpos[7+w] = wa
    mujoco.mj_forward(m, d)

def add_goal():
    s = rend.scene
    if s.ngeom < s.maxgeom:
        g = s.geoms[s.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.3]*3),
                            np.array([GOAL[0], GOAL[1], 0.3]), np.eye(3).flatten(),
                            np.array([0.9, 0.1, 0.1, 1.0], np.float32))
        s.ngeom += 1

# --- plot panel: world 20x14 m window ---
XMIN, XMAX, YMIN, YMAX = -9.0, 9.0, -5.0, 6.5
def to_px(x, y):
    px = (x - XMIN) / (XMAX - XMIN) * (WP - 20) + 10
    py = (1 - (y - YMIN) / (YMAX - YMIN)) * (H - 60) + 10
    return px, py

COL = {"true": (60, 190, 90), "drift": (235, 150, 40),
       "gps": (225, 60, 60), "ekf": (70, 130, 220)}
OBST = [(-5, 4, 1.4), (-1, 4.5, 1.4)]
RECT = [(-4, -1, .5, .7), (0, 1.5, .4, .4), (6, -1, .4, .4)]

def draw_panel(tt):
    img = Image.new("RGB", (WP, H), (18, 20, 24))
    dr = ImageDraw.Draw(img)
    for (x, y, r) in OBST:
        p1 = to_px(x - r, y + r); p2 = to_px(x + r, y - r)
        dr.ellipse([p1, p2], outline=(90, 90, 95), width=2)
    for (x, y, hx, hy) in RECT:
        p1 = to_px(x - hx, y + hy); p2 = to_px(x + hx, y - hy)
        dr.rectangle([p1, p2], outline=(90, 90, 95), width=2)
    gp = to_px(*GOAL); dr.line([gp[0]-7, gp[1]-7, gp[0]+7, gp[1]+7], fill=(220,60,60), width=3)
    dr.line([gp[0]-7, gp[1]+7, gp[0]+7, gp[1]-7], fill=(220,60,60), width=3)
    for name, rows, wdt in (("drift", drift, 2), ("ekf", ekf, 3), ("true", true, 3)):
        pts = [to_px(r[1], r[2]) for r in upto(rows, tt)]
        if len(pts) > 1:
            dr.line(pts, fill=COL[name], width=wdt)
    for r in upto(gps, tt)[-40:]:
        p = to_px(r[1], r[2])
        dr.line([p[0]-3, p[1]-3, p[0]+3, p[1]+3], fill=COL["gps"], width=1)
        dr.line([p[0]-3, p[1]+3, p[0]+3, p[1]-3], fill=COL["gps"], width=1)
    y0 = H - 44
    legend = [("truth", "true"), ("dead-reckon", "drift"), ("GPS fix", "gps"), ("GPS-fused EKF", "ekf")]
    x0 = 10
    for label, key in legend:
        dr.rectangle([x0, y0 + 4, x0 + 12, y0 + 14], fill=COL[key])
        dr.text((x0 + 16, y0), label, fill=(200, 203, 208))
        x0 += 16 + 7 * len(label) + 18
    # live error readouts
    if ekf and drift:
        tr = at(true, tt); ek = at(ekf, tt); dfr = at(drift, tt)
        e_ekf = math.hypot(ek[1]-tr[1], ek[2]-tr[2])
        e_dr = math.hypot(dfr[1]-tr[1], dfr[2]-tr[2])
        dr.text((10, y0 + 22), f"error vs truth   dead-reckon: {e_dr:4.2f} m    GPS-fused: {e_ekf:4.2f} m",
                fill=(200, 203, 208))
    return img

writer = imageio.get_writer(OUTMP4, fps=FPS, quality=7, macro_block_size=8)
wa = 0.0; prev = None
for tt in frames_t:
    _, x, y, yaw = at(true, tt)
    if prev is not None:
        wa -= math.hypot(x - prev[0], y - prev[1]) / 0.10
    prev = (x, y)
    set_pose(x, y, yaw, wa)
    cam.lookat = np.array([x, y, 0.3])
    rend.update_scene(d, cam); add_goal()
    left = Image.fromarray(rend.render().copy())
    canvas = Image.new("RGB", (W3D + WP, H), (10, 10, 12))
    canvas.paste(left, (0, 0)); canvas.paste(draw_panel(tt), (W3D, 0))
    writer.append_data(np.asarray(canvas))
writer.close()
fx, fy = true[-1][1], true[-1][2]
print(f"wrote {OUTMP4} frames={len(frames_t)} dur={t1-t0:.1f}s "
      f"final=({fx:.2f},{fy:.2f}) goal_err={math.hypot(fx-GOAL[0],fy-GOAL[1]):.2f}m "
      f"gps_fixes={len(gps)} ekf_pts={len(ekf)}")
