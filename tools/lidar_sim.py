"""Simulate a 2D LiDAR scan via MuJoCo ray-casting in the test arena, and VISUALIZE it.
The M20 has dual 3D LiDAR; this is a horizontal 360-ray slice to see it working."""
import math, numpy as np, mujoco
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from pathlib import Path
SCENE = Path.home()/"m20_sim/sdk_deploy/src/M20_sdk_deploy/M20_description/m20_mjcf/mjcf/scene_test.xml"
m = mujoco.MjModel.from_xml_path(str(SCENE))
d = mujoco.MjData(m)
STANCE = np.array([0,-0.7,1.4,0, 0,-0.7,1.4,0, 0,0.7,-1.4,0, 0,0.7,-1.4,0], float)
d.qpos[:] = 0; d.qpos[3] = 1.0; d.qpos[7:23] = STANCE
d.qpos[2] = 1.0; mujoco.mj_forward(m, d)
d.qpos[2] = 1.0 - float(d.geom_xpos[:, 2].min()) + 0.03
mujoco.mj_forward(m, d)

# LiDAR at robot center, ~0.35 m high; 360 horizontal rays, 12 m range
origin = np.array([d.qpos[0], d.qpos[1], 0.35])
N, RANGE = 360, 12.0
geomid = np.zeros(1, np.int32)
pts, angs, rngs = [], [], []
for i in range(N):
    a = 2 * math.pi * i / N
    vec = np.array([math.cos(a), math.sin(a), 0.0])
    dist = mujoco.mj_ray(m, d, origin, vec, None, 1, 1, geomid)   # exclude body 1 (base)
    if 0.6 < dist < RANGE:                                        # drop self-hits (<0.6 m)
        h = origin + dist * vec
        pts.append((h[0], h[1])); angs.append(a); rngs.append(dist)
pts = np.array(pts)
print(f"rays: {N}, hits: {len(pts)}, min range: {min(rngs):.2f} m, max: {max(rngs):.2f} m")

fig, ax = plt.subplots(1, 2, figsize=(15, 7))
# left: top-down LiDAR scan (what the sensor "sees")
ax[0].plot(origin[0], origin[1], 'b^', ms=14, label='M20 (LiDAR)')
ax[0].scatter(pts[:, 0], pts[:, 1], s=8, c='r', label='LiDAR returns')
for p in pts: ax[0].plot([origin[0], p[0]], [origin[1], p[1]], color='r', alpha=0.05, lw=0.5)
ax[0].set_title(f"Top-down LiDAR scan ({len(pts)} returns)"); ax[0].axis('equal'); ax[0].grid(True); ax[0].legend()
# right: polar range plot (classic LiDAR view)
ax2 = plt.subplot(1, 2, 2, projection='polar')
ax2.scatter(angs, rngs, s=6, c='r'); ax2.set_title("Polar range (angle vs distance)"); ax2.set_ylim(0, RANGE)
out = "/mnt/c/Users/16024/AppData/Local/Temp/claude/C--GIT-StatefullXOne-m20-autonomy-ws-m20-autonomy-ws/06227379-f151-4c17-a580-0b0e4555e6cc/scratchpad/lidar_scan.png"
plt.tight_layout(); plt.savefig(out, dpi=90); print("saved", out)
