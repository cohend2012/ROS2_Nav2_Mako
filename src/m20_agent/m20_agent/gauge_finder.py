"""Find gauges with NO prior positions: detect dials in camera frames, place them on the
map with camera bearing + LiDAR range, cluster sightings, and plan where to stand to read.

ROS-free and MuJoCo-free. The robot-specific parts come in through two callables:
    range_fn(bearing_body, elevation) -> (range_m, (hx, hy)) | None   # LiDAR along a ray
and the camera intrinsics below (the M20 sim camera; set from camera_info on hardware).

Pipeline (gauge_search.search_and_read = plan action find_gauges):
  survey viewpoints (coverage of the free map)  ->  at each: look around (8 headings)
  -> detect_dials(frame) -> bearing/elevation per dial -> LiDAR range -> map point
  -> GaugeClusters (sightings within 0.6 m = one candidate) -> viewing spot 1.2 m in
  front of the dial face (normal from 2 LiDAR rays) -> go there, aim, VERIFY, read.
"""
import math

import numpy as np

from m20_agent.gauge_dial import _round_components

# Sim camera (tools/mujoco_sim.py / sim_mission.py): 320x240, vertical FOV 45 deg.
IMG_W, IMG_H, FOVY_DEG = 320, 240, 45.0
FOCAL_PX = (IMG_H / 2) / math.tan(math.radians(FOVY_DEG / 2))     # ~289.7 px
CAM_FORWARD = 0.30            # camera eye ahead of base_link (m)


def detect_dials(rgb, min_px=20, min_needle=3):
    """All dial candidates in a frame: round, bright, unsaturated faces with a red
    needle inside. Returns [(cx, cy, r_px, score)], best first. The roundness test is
    gauge_dial._round_components (bbox corners empty, circle mostly filled), so tank
    shells, sky and walls are rejected. The needle is required on top of that."""
    a = np.asarray(rgb).astype(np.int16)
    R, G, B = a[..., 0], a[..., 1], a[..., 2]
    lum = (R + G + B) / 3.0
    sat = a.max(axis=2) - a.min(axis=2)
    # relative redness: a needle under side light (dial seen 30 deg off its face) is
    # ~100,40,40, which an absolute R > 110 test misses
    red = (R > 50) & (R > 1.6 * G) & (R > 1.6 * B) & (R - G > 30)
    top = np.percentile(lum, 97)
    H, W = lum.shape
    gy, gx = np.mgrid[0:H, 0:W]
    found = []
    for drop in (30.0, 45.0, 60.0, 75.0, 90.0):
        face = (sat < 30) & (lum > max(95.0, top - drop))
        for (y0, y1, x0, x1), n in _round_components(face, min_px):
            cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            r = max(y1 - y0, x1 - x0) / 2.0
            if r < 3:
                continue
            needle = int((red & (np.hypot(gx - cx, gy - cy) <= r)).sum())
            if needle < min_needle:
                continue
            if any(math.hypot(cx - f[0], cy - f[1]) < max(r, f[2]) for f in found):
                continue                      # same dial at another threshold
            found.append((cx, cy, r, float(min(1.0, n / 1500.0) * min(1.0, needle / 40.0))))
    return sorted(found, key=lambda f: -f[3])


_CX, _CY, _FY = None, None, None       # principal point / fy when set from camera_info


def set_camera(width, height, fx, fy=None, cx=None, cy=None):
    """Use the real camera's intrinsics (sensor_msgs/CameraInfo K) instead of the sim's."""
    global IMG_W, IMG_H, FOCAL_PX, _CX, _CY, _FY
    IMG_W, IMG_H, FOCAL_PX = int(width), int(height), float(fx)
    _FY, _CX, _CY = fy, cx, cy


def pixel_to_ray(cx, cy):
    """Pixel -> (bearing, elevation) in the BODY frame (bearing CCW+, i.e. left)."""
    px = IMG_W / 2.0 if _CX is None else _CX
    py = IMG_H / 2.0 if _CY is None else _CY
    return math.atan2(-(cx - px), FOCAL_PX), math.atan2(-(cy - py), _FY or FOCAL_PX)


class GaugeClusters:
    """Sightings -> gauge CANDIDATES (>= min_hits sightings within `radius`). Detection
    during the survey is deliberately permissive (a dial 3 m away is ~7 px across), so a
    candidate only becomes a gauge after VERIFICATION up close (executor / find_gauges:
    a dial must be seen near the image centre from the viewing spot)."""

    def __init__(self, radius=0.6, min_hits=1):
        self.radius, self.min_hits = radius, min_hits
        self.clusters = []            # dicts: pts [(x,y)], normals [(nx,ny)], score

    def add(self, x, y, normal=None, score=1.0):
        for c in self.clusters:
            cx, cy = np.mean(c["pts"], axis=0)
            if math.hypot(x - cx, y - cy) < self.radius:
                c["pts"].append((x, y)); c["score"] += score
                if normal is not None:
                    c["normals"].append(normal)
                return c
        c = {"pts": [(x, y)], "normals": [normal] if normal is not None else [], "score": score}
        self.clusters.append(c)
        return c

    def gauges(self):
        out = []
        for c in self.clusters:
            if len(c["pts"]) < self.min_hits:
                continue
            x, y = np.median(np.array(c["pts"]), axis=0)
            n = None
            if c["normals"]:
                v = np.mean(np.array(c["normals"]), axis=0)
                if np.hypot(*v) > 1e-6:
                    n = tuple(v / np.hypot(*v))
            out.append({"position": (float(x), float(y)), "normal": n, "hits": len(c["pts"]),
                        "score": round(c["score"], 2)})
        return sorted(out, key=lambda g: -g["hits"])


def verify_dial(rgb, max_bearing_deg=15.0):
    """Up-close confirmation: is there a dial near the image centre? -> (cx, cy, r) | None"""
    for (cx, cy, r, score) in detect_dials(rgb, min_px=150, min_needle=8):
        if abs(math.degrees(pixel_to_ray(cx, cy)[0])) <= max_bearing_deg:
            return cx, cy, r
    return None


def viewing_spot(position, normal, robot_xy, standoff=1.2):
    """Where to stand to read the dial: `standoff` m out along the face normal, facing
    the dial. Without a normal, use the direction it was first seen from."""
    px, py = position
    if normal is None:
        nx, ny = robot_xy[0] - px, robot_xy[1] - py
        L = math.hypot(nx, ny) or 1.0
        normal = (nx / L, ny / L)
    sx, sy = px + standoff * normal[0], py + standoff * normal[1]
    return sx, sy, math.atan2(py - sy, px - sx)


def survey_viewpoints(planner, start_xy, spacing=3.0, clear=0.9, bounds=None):
    """Coverage viewpoints over the planner's free space: one per `spacing` m cell (the
    clearest spot in it, if >= `clear` m), within `bounds` (xmin, xmax, ymin, ymax),
    reachable from start, in nearest-neighbour order. This is the 'explore without
    knowing where gauges are' route."""
    d = np.where(np.isinf(planner.dist), 99.0, planner.dist)   # inf = far from everything
    h, w = d.shape
    unk = getattr(planner, "unk", None)
    step = max(1, int(round(spacing / planner.res)))
    pts = []
    for r0 in range(0, h, step):
        for c0 in range(0, w, step):
            win = d[r0:r0 + step, c0:c0 + step]
            if unk is not None:                      # never survey from unmapped space
                win = np.where(unk[r0:r0 + step, c0:c0 + step], 0.0, win)
            if win.size == 0 or win.max() < clear:
                continue
            rr, cc = np.unravel_index(np.argmax(np.minimum(win, 3.0)), win.shape)
            x, y = planner.cell_to_world(r0 + rr, c0 + cc)
            if bounds and not (bounds[0] <= x <= bounds[1] and bounds[2] <= y <= bounds[3]):
                continue
            pts.append((x, y))
    pts = [p for p in pts if planner.plan(start_xy[0], start_xy[1], *p) is not None]
    order, cur = [], start_xy
    while pts:
        nxt = min(pts, key=lambda p: math.hypot(p[0] - cur[0], p[1] - cur[1]))
        order.append(nxt); pts.remove(nxt); cur = nxt
    return order


def view_plan(planner, start_xy, bounds=None, rng=(1.0, 2.8), half_fov_deg=18.0,
              max_incidence_deg=40.0, cand_step=0.5, clear=0.75, n_headings=16,
              surf_step=0.2, target=0.97, min_gain=3):
    """Views that SEE EVERY MAPPED SURFACE instead of spinning everywhere. Gauges are
    mounted on equipment, and equipment is what the map's occupied cells are. So:
      surfaces : occupied cells with free space beside them, each with an outward normal
      views    : (free spot >= `clear` from obstacles, heading) pairs
      covers   : a surface sample is covered by a view when it is `rng` m away, inside
                 the central +-half_fov of the camera, seen within `max_incidence` of
                 face-on (the detector needs that), with clear line of sight
    Greedy set cover (most new surface first) until `target` of what any view can see is
    covered, then nearest-neighbour tour from start. Returns [(x, y, yaw)]: drive there,
    ARRIVE facing yaw, take one photo. No in-place 360s."""
    dist = np.where(np.isinf(planner.dist), 9.0, planner.dist)
    h, w = dist.shape
    res = planner.res
    occ = dist <= 1e-6
    free = ~occ
    border = occ & (np.roll(free, 1, 0) | np.roll(free, -1, 0) | np.roll(free, 1, 1) | np.roll(free, -1, 1))
    gy, gx = np.gradient(np.minimum(dist, 1.0))
    sk = max(1, int(round(surf_step / res)))
    rr, cc = np.nonzero(border)
    keep = (rr % sk == 0) | (cc % sk == 0)
    rr, cc = rr[keep], cc[keep]
    S, N = [], []
    for r, c in zip(rr, cc):
        x, y = planner.cell_to_world(r, c)
        if bounds and not (bounds[0] - 1 <= x <= bounds[1] + 1 and bounds[2] - 1 <= y <= bounds[3] + 1):
            continue
        # outward normal: mean gradient over the free neighbours
        nb = [(r + dr, c + dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1)
              if 0 <= r + dr < h and 0 <= c + dc < w and free[r + dr, c + dc]]
        vx = sum(gx[a, b] for a, b in nb); vy = sum(gy[a, b] for a, b in nb)
        L = math.hypot(vx, vy)
        if L > 0.3 * len(nb) * 0.5:
            S.append((x, y)); N.append((vx / L, vy / L))
            continue
        # thin object (a rack beam, a post): free space on both sides and the gradient
        # cancels. Give it one face per open side instead of dropping it.
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            if 0 <= r + dr < h and 0 <= c + dc < w and free[r + dr, c + dc]:
                S.append((x, y)); N.append((float(dc), float(dr)))
    if not S:
        return []
    S, N = np.array(S), np.array(N)
    # thin surface samples to ~surf_step spacing
    order, kept = np.lexsort((S[:, 1], S[:, 0])), []
    for i in order:
        # a duplicate is close AND facing the same way: the two faces of a thin rack beam
        # are 0.1 m apart but opposite (the gauge is on one of them)
        if all(abs(S[i, 0] - S[j, 0]) > surf_step * 0.9 or abs(S[i, 1] - S[j, 1]) > surf_step * 0.9
               or N[i] @ N[j] < 0.5 for j in kept[-80:]):
            kept.append(i)
    S, N = S[kept], N[kept]

    def los(x0, y0, x1, y1):
        n = max(2, int(math.hypot(x1 - x0, y1 - y0) / (res * 0.7)))
        t = np.linspace(0.0, 1.0, n)[:-1]
        xs, ys = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
        r_ = ((ys - planner.oy) / res).astype(int); c_ = ((xs - planner.ox) / res).astype(int)
        ok = (r_ >= 0) & (r_ < h) & (c_ >= 0) & (c_ < w)
        # stop checking in the last 0.25 m (the surface cell itself and its neighbours)
        tail = np.hypot(x1 - xs, y1 - ys) < 0.25
        return not np.any(occ[r_[ok & ~tail], c_[ok & ~tail]])

    step = max(1, int(round(cand_step / res)))
    hfov, minc = math.radians(half_fov_deg), math.cos(math.radians(max_incidence_deg))
    views = []                                           # (x, y, yaw, covered index set)
    for r0 in range(step // 2, h, step):
        for c0 in range(step // 2, w, step):
            if dist[r0, c0] < clear:
                continue
            x, y = planner.cell_to_world(r0, c0)
            if bounds and not (bounds[0] <= x <= bounds[1] and bounds[2] <= y <= bounds[3]):
                continue
            dx, dy = S[:, 0] - x, S[:, 1] - y
            d = np.hypot(dx, dy)
            facing = -(dx * N[:, 0] + dy * N[:, 1]) / np.maximum(d, 1e-6)   # cos incidence
            near = np.nonzero((d >= rng[0]) & (d <= rng[1]) & (facing >= minc))[0]
            near = [i for i in near if los(x, y, S[i, 0], S[i, 1])]
            if not near:
                continue
            ang = np.arctan2(dy[near], dx[near])
            for k in range(n_headings):
                yaw = -math.pi + 2 * math.pi * k / n_headings
                rel = np.abs(np.arctan2(np.sin(ang - yaw), np.cos(ang - yaw)))
                cov = {near[j] for j in np.nonzero(rel <= hfov)[0]}
                if len(cov) >= min_gain:
                    views.append((x, y, yaw, cov))
    if not views:
        return []
    coverable = set().union(*(v[3] for v in views))
    reach = {}

    def reachable(v):
        key = (round(v[0], 2), round(v[1], 2))
        if key not in reach:
            reach[key] = planner.plan(start_xy[0], start_xy[1], v[0], v[1]) is not None
        return reach[key]
    chosen, seen = [], set()
    while len(seen) < target * len(coverable):
        best = max(views, key=lambda v: len(v[3] - seen))
        gain = len(best[3] - seen)
        if gain < min_gain:
            break
        views.remove(best)
        if not reachable(best):
            continue
        chosen.append(best[:3]); seen |= best[3]
    tour, cur = [], start_xy
    while chosen:
        nxt = min(chosen, key=lambda v: math.hypot(v[0] - cur[0], v[1] - cur[1]))
        tour.append(nxt); chosen.remove(nxt); cur = nxt[:2]
    return tour
