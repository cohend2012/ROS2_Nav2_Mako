"""Occupancy grid from a MuJoCo scene (shared by training, eval and the mission harness).

Cells are tested against each static obstacle geom's TRUE shape at a few heights
(what a 3D-sliced map sees). Robot geoms and mocap bodies (people) are excluded:
the map is the static world, as slam_toolbox would build it with nobody around.
"""
import mujoco
import numpy as np


def static_obstacle_geoms(m):
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    return [g for g in range(m.ngeom) if m.geom_bodyid[g] == 0 and g != floor]


def rasterize(m, d, res=0.05, half=10.0, z_probe=(0.15, 0.4, 0.8)):
    """Returns (data[int8 0/100], width, height, res, origin_x, origin_y), the
    OccupancyGrid layout GridPlanner.set_map takes."""
    n = int(2 * half / res)
    xs = -half + (np.arange(n) + 0.5) * res
    X, Y = np.meshgrid(xs, xs)                    # rows = y, cols = x
    occ = np.zeros((n, n), bool)
    for g in static_obstacle_geoms(m):
        p, R, s = d.geom_xpos[g], d.geom_xmat[g].reshape(3, 3), m.geom_size[g]
        # cheap reject: skip geoms whose bounding circle misses the grid
        if abs(p[0]) > half + m.geom_rbound[g] or abs(p[1]) > half + m.geom_rbound[g]:
            continue
        for z in z_probe:
            P = np.stack([X - p[0], Y - p[1], np.full_like(X, z - p[2])], -1) @ R   # geom frame
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
                inside = (np.abs(P[..., 0]) <= s[0]) & (np.abs(P[..., 1]) <= s[1]) & (np.abs(P[..., 2]) <= s[2])
            elif m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER:
                inside = (np.hypot(P[..., 0], P[..., 1]) <= s[0]) & (np.abs(P[..., 2]) <= s[1])
            else:
                continue
            occ |= inside
    return np.where(occ, 100, 0).astype(np.int8).reshape(-1), n, n, res, -half, -half
