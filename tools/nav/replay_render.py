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

# load true pose log: t,x,y,yaw
rows = []
with open(f"{OUTDIR}/traj_true.csv") as f:
    r = csv.reader(f); next(r)
    for a in r:
        if len(a) >= 4:
            rows.append((float(a[0]), float(a[1]), float(a[2]), float(a[3])))
t0 = rows[0][0]; dur = rows[-1][0] - t0
# resample to real-time FPS by timestamp
frames_t = np.arange(0, dur, 1.0/FPS)
def sample(tt):
    # nearest logged row to time tt
    i = min(range(len(rows)), key=lambda k: abs((rows[k][0]-t0)-tt))
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

def set_pose(x, y, yaw, wheel_ang):
    d.qpos[:] = 0
    d.qpos[0], d.qpos[1], d.qpos[2] = x, y, z_stand
    d.qpos[3] = math.cos(yaw/2); d.qpos[6] = math.sin(yaw/2)  # quat w,z
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
    _, x, y, yaw = sample(tt)
    if prev is not None:
        dxy = math.hypot(x-prev[0], y-prev[1])
        wheel_ang -= dxy / 0.10           # spin wheels by distance/rolling-radius
    prev = (x, y)
    set_pose(x, y, yaw, wheel_ang)
    cam.lookat = np.array([x, y, 0.3])
    rend.update_scene(d, cam)
    add_goal_marker()
    frames.append(rend.render().copy())

imageio.mimwrite(OUTMP4, frames, fps=FPS, quality=7, macro_block_size=8)
fx, fy = rows[-1][1], rows[-1][2]
print(f"wrote {OUTMP4} frames={len(frames)} dur={dur:.1f}s "
      f"final=({fx:.2f},{fy:.2f}) goal_err={math.hypot(fx-GOAL[0], fy-GOAL[1]):.2f}m")
