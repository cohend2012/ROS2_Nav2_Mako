#!/usr/bin/env python3
"""Replay-render the REAL logged Nav2+SLAM run as an MP4 (host, egl).
Places the M20 at each logged TRUE pose (x,y,yaw) and renders in the field with a
tracking camera + a red goal marker. Real trajectory -> honest 'working or not' video."""
import os, sys, csv, math
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np, mujoco, imageio
from pathlib import Path

OUTDIR = os.environ.get("OUTDIR", ".")
OUTMP4 = os.environ.get("OUTMP4", f"{OUTDIR}/nav_live.mp4")
GOAL = (-6.0, -0.5)
FPS = 20
M = Path.home()/"m20_sim/sdk_deploy/src/M20_sdk_deploy/M20_description/m20_mjcf/mjcf/oil_gas_field.xml"

STAND = np.array([0,-0.7,1.4,0, 0,-0.7,1.4,0, 0,0.7,-1.4,0, 0,0.7,-1.4,0], float)
WHEELS = (3, 7, 11, 15)

# load true pose log: t,x,y,yaw[,z,roll,pitch]  (z/roll/pitch: logger >= 2026-07-20;
# with them the body is placed at the TRUE physics pose — no ground-settle guessing)
rows = []
with open(f"{OUTDIR}/traj_true.csv") as f:
    r = csv.reader(f); next(r)
    for a in r:
        if len(a) >= 7:
            rows.append(tuple(float(v) for v in a[:7]))
        elif len(a) >= 4:
            rows.append((float(a[0]), float(a[1]), float(a[2]), float(a[3]),
                         None, 0.0, 0.0))
t0 = rows[0][0]; dur = rows[-1][0] - t0

# optional logged joints (t + 16 positions): lets the video replay the REAL
# commander-driven standup (and true wheel spin) instead of a fixed stand pose
jrows = []
try:
    with open(f"{OUTDIR}/joints.csv") as f:
        r = csv.reader(f); next(r)
        for a in r:
            if len(a) >= 17:
                jrows.append((float(a[0]), np.array([float(v) for v in a[1:17]])))
except FileNotFoundError:
    pass
# bisect samplers — the linear min() scans were O(frames*rows) and timed out
# on the 392 s patrol tour (67k rows x 7.8k frames ~ 500M ops)
import bisect
_jt = [r[0] - t0 for r in jrows]
def sample_joints(tt):
    if not jrows:
        return None
    i = min(max(bisect.bisect(_jt, tt) - 1, 0), len(jrows) - 1)
    return jrows[i][1]
# resample to real-time FPS by timestamp; FPS/TSTART/TEND env overrides let
# long tours render in memory-safe chunks (the 392 s patrol OOM-killed a
# single-pass render twice on the 6 GB VM)
FPS = int(os.environ.get("FPS", FPS if isinstance(FPS, int) else 20))
_ts = float(os.environ.get("TSTART", "0"))
_te = float(os.environ.get("TEND", str(dur)))
frames_t = np.arange(_ts, min(_te, dur), 1.0/FPS)
_rt = [r[0] - t0 for r in rows]
def sample(tt):
    # nearest logged row to time tt
    i = min(max(bisect.bisect(_rt, tt) - 1, 0), len(rows) - 1)
    return rows[i]

m = mujoco.MjModel.from_xml_path(str(M))
W, H = 960, 540
m.vis.global_.offwidth = W; m.vis.global_.offheight = H
d = mujoco.MjData(m)
# settle to standing height once
d.qpos[:] = 0; d.qpos[3] = 1.0; d.qpos[7:23] = STAND
d.qpos[2] = 0.6; mujoco.mj_forward(m, d)
z_stand = 0.6 - float(d.geom_xpos[:, 2].min()) + 0.02

rend = mujoco.Renderer(m, H, W)
cam = mujoco.MjvCamera()
cam.distance = 8.5; cam.azimuth = 120; cam.elevation = -28

def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy)

def set_pose(x, y, yaw, wheel_ang, joints=None, z=None, roll=0.0, pitch=0.0):
    d.qpos[:] = 0
    d.qpos[3:7] = euler_to_quat(roll, pitch, yaw)
    if joints is not None:
        d.qpos[7:23] = joints
        if z is not None:
            # TRUE logged physics pose — the honest placement
            d.qpos[0], d.qpos[1], d.qpos[2] = x, y, z
        else:
            # old logs without z: settle the body so the lowest geom touches ground
            # (kinematic guess — made the standup look floaty; kept for fallback)
            d.qpos[0], d.qpos[1], d.qpos[2] = x, y, 1.0
            mujoco.mj_forward(m, d)
            d.qpos[2] = 1.0 - float(d.geom_xpos[:, 2].min()) + 0.02
    else:
        d.qpos[0], d.qpos[1], d.qpos[2] = x, y, z if z is not None else z_stand
        d.qpos[7:23] = STAND
        for w in WHEELS:
            d.qpos[7+w] = wheel_ang
    mujoco.mj_forward(m, d)

def add_goal_marker():
    s = rend.scene
    if s.ngeom < s.maxgeom:
        g = s.geoms[s.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE,
                            np.array([0.3, 0.3, 0.3]),
                            np.array([GOAL[0], GOAL[1], 0.3]),
                            np.eye(3).flatten(),
                            np.array([0.9, 0.1, 0.1, 1.0], np.float32))
        s.ngeom += 1

frames = []
wheel_ang = 0.0; prev = None
for tt in frames_t:
    _, x, y, yaw, z, roll, pitch = sample(tt)
    if prev is not None:
        dxy = math.hypot(x-prev[0], y-prev[1])
        wheel_ang -= dxy / 0.10           # spin wheels by distance/rolling-radius
    prev = (x, y)
    set_pose(x, y, yaw, wheel_ang, joints=sample_joints(tt), z=z, roll=roll, pitch=pitch)
    cam.lookat = np.array([x, y, 0.3])
    rend.update_scene(d, cam)
    add_goal_marker()
    frames.append(rend.render().copy())

imageio.mimwrite(OUTMP4, frames, fps=FPS, quality=7, macro_block_size=8)
fx, fy = rows[-1][1], rows[-1][2]
print(f"wrote {OUTMP4} frames={len(frames)} dur={dur:.1f}s "
      f"final=({fx:.2f},{fy:.2f}) goal_err={math.hypot(fx-GOAL[0], fy-GOAL[1]):.2f}m")
