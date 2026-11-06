"""Build the site map the way the robot would: from its own LiDAR along a mapping drive,
NOT from the scene geometry. Used by sim_mission.py --map lidar.

"Map the area" on the real robot = an operator drives a loop while slam_toolbox fuses
/scan. Here: the route is a loop over the site (what an operator would drive); along it,
every `step` m, the simulated 3-D LiDAR (the env's 16 elevations, 0.5 deg azimuth) is cast
against the physics scene. A return inside the /scan height band (0.10-1.00 m above
ground, pointcloud_to_laserscan.yaml) marks its cell occupied; every ray marks the cells
it crossed free; cells never crossed stay UNKNOWN (-1). Errors a real SLAM map has:
  pose error per scan (SLAM's residual, default 3 cm / 0.7 deg), range noise, dropouts.
Unlike scene rasterization, low pipes in the band show up and the far side of big tanks
stays unknown.
"""
import math

import mujoco
import numpy as np

from m20_ai_nav.geometry import densify
from m20_ai_nav.sim.env import LIDAR_ELEVS_DEG, LIDAR_OFFSET

BAND = (0.10, 1.00)


def build(env, route, res=0.05, half=10.0, step=0.25, pose_sigma=0.03, yaw_sigma_deg=0.7,
          range_sigma=0.02, dropout=0.05, n_az=720, max_range=10.0, seed=0, log=print):
    m, d = env.m, env.d
    rng = np.random.default_rng(seed)
    n = int(2 * half / res)
    hits = np.zeros((n, n), np.int32)
    free = np.zeros((n, n), np.int32)
    q0 = d.qpos.copy()
    z0 = float(d.qpos[2])
    az = np.arange(n_az) * (2 * math.pi / n_az)
    el = np.radians(np.array(LIDAR_ELEVS_DEG, float))
    A, E = np.meshgrid(az, el)
    dirs_body = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1).reshape(-1, 3)
    nray = len(dirs_body)
    gid = np.zeros(nray, np.int32)
    dist = np.zeros(nray)
    pts = [tuple(p) for p in densify(np.asarray(route, float), step)]
    poses = []
    for (a, b) in zip(pts, pts[1:] + pts[-1:]):
        poses.append((a[0], a[1], math.atan2(b[1] - a[1], b[0] - a[0]) if b != a else 0.0))
    for k, (x, y, yaw) in enumerate(poses):
        d.qpos[:] = q0
        d.qpos[0], d.qpos[1], d.qpos[2] = x, y, z0
        d.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        mujoco.mj_forward(m, d)
        c, s_ = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
        vecs = dirs_body @ R.T
        origin = np.array([x, y, z0 + LIDAR_OFFSET])
        mujoco.mj_multiRay(m, d, origin, vecs.reshape(-1), env.geomgroup, 1, -1, gid, dist, None,
                           nray, max_range)
        r = np.where(gid >= 0, dist, max_range)
        r = r + rng.normal(0, range_sigma, nray)
        keep = rng.random(nray) > dropout
        hz = origin[2] + vecs[:, 2] * r
        horiz = r * np.hypot(vecs[:, 0], vecs[:, 1])
        # where SLAM THINKS the robot was for this scan
        bx, by = x + rng.normal(0, pose_sigma), y + rng.normal(0, pose_sigma)
        byaw = yaw + math.radians(rng.normal(0, yaw_sigma_deg))
        # = pointcloud_to_laserscan: per azimuth, the NEAREST return inside the height
        # band is the 2-D range (a beam passing over a low pipe does not clear it)
        inband = keep & (gid >= 0) & (hz > BAND[0]) & (hz < BAND[1])
        az_i = np.tile(np.arange(n_az), len(el))
        r2 = np.full(n_az, np.inf)
        np.minimum.at(r2, az_i[inband], horiz[inband])
        for a_i in range(0, n_az, 2):                    # 1 deg bins (the stack's /scan)
            rng_ = min(r2[a_i], r2[a_i + 1])
            th = az[a_i] + byaw
            L = (rng_ - 2 * res) if np.isfinite(rng_) else 8.0
            if L > 0.3:
                t = np.arange(0.3, L, res)
                fr = ((by + t * math.sin(th) + half) / res).astype(int)
                fc = ((bx + t * math.cos(th) + half) / res).astype(int)
                okf = (fr >= 0) & (fr < n) & (fc >= 0) & (fc < n)
                free[fr[okf], fc[okf]] += 1
            if np.isfinite(rng_):
                hr, hc = int((by + rng_ * math.sin(th) + half) / res), int((bx + rng_ * math.cos(th) + half) / res)
                if 0 <= hr < n and 0 <= hc < n:
                    hits[hr, hc] += 1
        if k % 50 == 0:
            log(f"[map] scan {k}/{len(poses)}")
    d.qpos[:] = q0
    mujoco.mj_forward(m, d)
    occ = (hits >= 2) & (hits * 3 >= free)
    grid = np.full((n, n), -1, np.int8)
    grid[free > 0] = 0
    grid[occ] = 100
    log(f"[map] {len(poses)} scans: {int(occ.sum())} occupied, {int((grid == 0).sum())} free, "
        f"{int((grid < 0).sum())} unknown cells")
    return grid.reshape(-1), n, n, res, -half, -half


def save_png(grid_tuple, path):
    from PIL import Image
    data, w, h, *_ = grid_tuple
    g = np.asarray(data).reshape(h, w)
    img = np.full((h, w, 3), 128, np.uint8)
    img[g == 0] = (245, 245, 240)
    img[g >= 50] = (30, 30, 30)
    Image.fromarray(img[::-1]).save(path)
