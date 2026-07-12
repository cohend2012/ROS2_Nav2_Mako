#!/usr/bin/env python3
"""Compare what the CURRENT sim LiDAR sees vs the REAL M20 sensor, from the same pose.

Left  : our current sim model — single 2D ring at z=0.35 m (360 x 1 line).
Right : the real dual-RoboSense merged sensor — hemispherical 360°x90°, 96 lines
        (rendered subsampled 24 elevation x 120 azimuth), 0.5 m blind radius
        (from Deep Robotics' official faster-lio config), expressed in one body frame
        exactly like /LIDAR/POINTS.
Same robot pose in the oil & gas field; hit points colored by height.
"""
import os, math
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np, mujoco
from pathlib import Path
from PIL import Image

OUT = os.environ["OUT"]
M = Path.home()/"m20_sim/sdk_deploy/src/M20_sdk_deploy/M20_description/m20_mjcf/mjcf/oil_gas_field.xml"
STAND = np.array([0,-0.7,1.4,0, 0,-0.7,1.4,0, 0,0.7,-1.4,0, 0,0.7,-1.4,0], float)
POSE = (1.0, 0.2, math.radians(175))     # near our nav start, facing the field (-x)
BLIND, RMAX = 0.5, 12.0                  # blind radius from official config; our sim max
W, H = 1100, 640

m = mujoco.MjModel.from_xml_path(str(M))
m.vis.global_.offwidth = W; m.vis.global_.offheight = H
d = mujoco.MjData(m)
d.qpos[:] = 0
d.qpos[0], d.qpos[1] = POSE[0], POSE[1]
d.qpos[3] = math.cos(POSE[2]/2); d.qpos[6] = math.sin(POSE[2]/2)
d.qpos[7:23] = STAND
d.qpos[2] = 0.6; mujoco.mj_forward(m, d)
d.qpos[2] = 0.6 - float(d.geom_xpos[:, 2].min()) + 0.02
mujoco.mj_forward(m, d)
_gid = np.zeros(1, np.int32)
SENSOR = np.array([d.qpos[0], d.qpos[1], float(d.qpos[2]) + 0.10])  # top of body (ASSUMED site)

def cast(az, el):
    v = np.array([math.cos(az)*math.cos(el), math.sin(az)*math.cos(el), math.sin(el)])
    dist = mujoco.mj_ray(m, d, SENSOR, v, None, 1, 1, _gid)
    if BLIND < dist < RMAX:
        return SENSOR + dist*v
    return None

# current sim: 360 rays, one plane at z=0.35 (el=0 from a 0.35m origin -> use flat ring from old origin)
ring_origin = np.array([d.qpos[0], d.qpos[1], 0.35])
ring_hits = []
for k in range(360):
    a = POSE[2] + math.radians(k)
    v = np.array([math.cos(a), math.sin(a), 0.0])
    dist = mujoco.mj_ray(m, d, ring_origin, v, None, 1, 1, _gid)
    if 0.6 < dist < RMAX:
        ring_hits.append(ring_origin + dist*v)

# real sensor: hemisphere 360x90 (el 0 down to -90 covers ground; RoboSense Airy is
# hemispherical looking outward/down from top mount) — sample el in [-80, +10]
hemi_hits = []
for ei in range(24):
    el = math.radians(-80 + ei * (90/24))
    for ai in range(120):
        az = POSE[2] + 2*math.pi*ai/120
        p = cast(az, el)
        if p is not None:
            hemi_hits.append(p)

def render(hits, title_color):
    rend = mujoco.Renderer(m, H, W, max_geom=len(hits) + 200)
    cam = mujoco.MjvCamera()
    cam.lookat = np.array([d.qpos[0] - 3.5, d.qpos[1], 0.4])
    cam.distance = 10.5; cam.azimuth = 155; cam.elevation = -30
    rend.update_scene(d, cam)
    s = rend.scene
    zs = [h[2] for h in hits] or [0.0]
    zmin = min(zs)
    zmax = max(zs) if max(zs) > zmin else zmin + 1e-6
    for hpt in hits:
        if s.ngeom >= s.maxgeom: break
        g = s.geoms[s.ngeom]
        t = (hpt[2] - zmin) / (zmax - zmin)
        rgba = np.array([0.1 + 0.9*t, 0.9 - 0.5*t, 1.0 - 0.9*t, 0.95], np.float32)  # blue(low)->red(high)
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.05]*3),
                            np.asarray(hpt, float), np.eye(3).flatten(), rgba)
        s.ngeom += 1
    # sensor marker
    if s.ngeom < s.maxgeom:
        g = s.geoms[s.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.09]*3),
                            SENSOR, np.eye(3).flatten(), np.array(title_color, np.float32))
        s.ngeom += 1
    return rend.render().copy()

img_ring = render(ring_hits, [0.0, 1.0, 1.0, 1.0])
img_hemi = render(hemi_hits, [1.0, 0.3, 0.0, 1.0])

canvas = Image.new("RGB", (W, H*2 + 8), (10, 10, 12))
canvas.paste(Image.fromarray(img_ring), (0, 0))
canvas.paste(Image.fromarray(img_hemi), (0, H + 8))
canvas.save(OUT)
print(f"wrote {OUT} ring_hits={len(ring_hits)} hemi_hits={len(hemi_hits)}")
