"""Our own GLOBAL planner for standalone mode: A* on an inflated occupancy grid.

ROS-free; takes the raw OccupancyGrid data array. numpy-only (no scipy in the image).
Plans on a grid downsampled to `plan_res` (default 0.10 m — the costmap resolution
Nav2 already uses here) so a 40x40 m site plans in well under a second.

Cost model (mirrors the Nav2 inflation layer):
  lethal   : occupied cells dilated by robot_radius. 0.55 m, NOT Nav2's 0.45: the
             M20's collision geoms reach 0.50 m from base_link when it pivots in
             place (measured from the vendor MJCF); 0.45 let A* hug obstacles and a
             skid-steer pivot clipped skid2 in the sim mission (2026-11-04).
  soft     : + cost_scale * (1 - d/inflation_radius) out to inflation_radius
  unknown  : free with a small penalty (open pad; same policy as the costmap config)
"""
import heapq
import math

import numpy as np

OCC_THRESH = 65


class GridPlanner:
    def __init__(self, robot_radius=0.55, inflation_radius=1.2, cost_scale=8.0,
                 unknown_cost=0.5, plan_res=0.10, pivot_clear=None, straight_ext=0.6):
        self.rr, self.ir = robot_radius, inflation_radius
        self.cost_scale, self.unknown_cost, self.plan_res = cost_scale, unknown_cost, plan_res
        self.cost = None
        # pivot_clear (office profile: 0.6 m): a path may only TURN where the robot can
        # pivot. Narrow stretches (doors) are crossed on a straight line that extends
        # straight_ext beyond them on both sides; see _pivot_aware().
        self.pivot_clear, self.straight_ext = pivot_clear, straight_ext

    def set_map(self, data, width, height, resolution, origin_x, origin_y):
        g = np.asarray(data, np.int16).reshape(height, width)
        f = max(1, int(round(self.plan_res / resolution)))
        h2, w2 = height // f, width // f
        g = g[:h2 * f, :w2 * f].reshape(h2, f, w2, f)
        occ = (g >= OCC_THRESH).any(axis=(1, 3))
        unk = (g < 0).all(axis=(1, 3))
        self.res = resolution * f
        self.ox, self.oy = origin_x, origin_y
        self.h, self.w = h2, w2
        dist = self._distance(occ)                    # metres to nearest occupied
        cost = np.ones((h2, w2))
        cost += np.where(dist < self.ir, self.cost_scale * (1.0 - dist / self.ir), 0.0)
        cost += np.where(unk, self.unknown_cost, 0.0)
        cost[dist <= self.rr] = np.inf
        self.cost = cost
        self.dist = dist
        self.unk = unk

    def _distance(self, occ):
        """Approximate Euclidean distance transform by ring dilation (bounded at ir)."""
        n = int(math.ceil(self.ir / self.res)) + 1
        dist = np.full(occ.shape, np.inf)
        dist[occ] = 0.0
        offs = sorted(((dx, dy) for dx in range(-n, n + 1) for dy in range(-n, n + 1)
                       if 0 < dx * dx + dy * dy <= n * n), key=lambda o: o[0] ** 2 + o[1] ** 2)
        ys, xs = np.nonzero(occ)
        H, W = occ.shape
        for dx, dy in offs:
            d = math.hypot(dx, dy) * self.res
            yy, xx = ys + dy, xs + dx
            ok = (yy >= 0) & (yy < H) & (xx >= 0) & (xx < W)
            yy, xx = yy[ok], xx[ok]
            cur = dist[yy, xx]
            dist[yy, xx] = np.minimum(cur, d)
        return dist

    def world_to_cell(self, x, y):
        return int((y - self.oy) / self.res), int((x - self.ox) / self.res)

    def cell_to_world(self, r, c):
        return self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res

    def _nearest_free(self, r, c, max_r=10):
        if self.cost[r, c] < np.inf:
            return r, c
        for rad in range(1, max_r + 1):
            for dr in range(-rad, rad + 1):
                for dc in (-rad, rad) if abs(dr) != rad else range(-rad, rad + 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < self.h and 0 <= cc < self.w and self.cost[rr, cc] < np.inf:
                        return rr, cc
        return None

    def plan(self, sx, sy, gx, gy):
        """Returns a list of (x, y) world points start->goal, or None if no path."""
        path = self._astar(sx, sy, gx, gy)
        if path is None or self.pivot_clear is None:
            return path
        return self._pivot_aware(path, gx, gy)

    def plan_pose(self, sx, sy, gx, gy, gyaw, approach=(0.6, 0.45, 0.3)):
        """Plan to a POSE: arrive driving along the goal heading. Tight places (0.50 m of
        clearance, less than the 0.53 m corner sweep) leave no room to pivot at the end,
        so the path ends with a straight run of `approach` metres INTO the goal along
        gyaw. Falls back to a plain plan when no approach segment fits."""
        if gyaw is None:
            return self.plan(sx, sy, gx, gy)
        c, s_ = math.cos(gyaw), math.sin(gyaw)
        for d in approach:
            pre = (gx - c * d, gy - s_ * d)
            if self._clear(*pre) <= self.rr or not self._line_ok(pre, (gx, gy), self.rr):
                continue
            head = self.plan(sx, sy, *pre)
            if head is not None:
                return head + [(gx, gy)]
        return self.plan(sx, sy, gx, gy)

    def _clear(self, x, y):
        r, c = self.world_to_cell(x, y)
        if not (0 <= r < self.h and 0 <= c < self.w):
            return 0.0
        return float(self.dist[r, c])

    def _line_ok(self, a, b, min_clear):
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (self.res * 0.5)))
        return all(self._clear(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) > min_clear
                   for k in range(n + 1))

    def _pivot_aware(self, path, gx, gy, depth=0):
        """Cross every narrow stretch STRAIGHT. A 0.91 m-long skid-steer body cannot
        turn while it is between door jambs (corners sweep 0.53 m); plain A* turns
        right at the door exit, so the carrot pulls the robot round while it is still
        in the doorway (office forensics 2026-11-04: stuck/clipping exactly there).
        For each run of points with clearance < pivot_clear: take the run's axis, add
        straight_ext of straight line before and after it (into open space), and let
        A* reconnect start -> entry and exit -> goal."""
        clear = [self._clear(*p) for p in path]
        narrow = [c < self.pivot_clear for c in clear]
        i = next((k for k, v in enumerate(narrow) if v), None)
        if i is None or depth > 6:
            return path
        j = i
        while j + 1 < len(path) and narrow[j + 1]:
            j += 1
        a, b = path[max(0, i - 1)], path[min(len(path) - 1, j + 1)]
        ux, uy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(ux, uy)
        if L < 1e-6 or i == 0 or j == len(path) - 1:
            # starts/ends inside a narrow spot: nothing to straighten safely
            rest = self._pivot_aware(path[j + 1:], gx, gy, depth + 1) if j + 1 < len(path) else []
            return path[:j + 1] + rest
        ux, uy = ux / L, uy / L
        for ext in (self.straight_ext, 0.45, 0.3):
            entry = (a[0] - ux * ext, a[1] - uy * ext)
            exit_ = (b[0] + ux * ext, b[1] + uy * ext)
            if (self._clear(*entry) > self.pivot_clear and self._clear(*exit_) > self.pivot_clear
                    and self._line_ok(entry, exit_, self.rr)):
                head = self._astar(path[0][0], path[0][1], *entry)
                tail = self._astar(*exit_, gx, gy)
                if head is None or tail is None:
                    continue
                straight = [entry, exit_]
                tail = self._pivot_aware(tail, gx, gy, depth + 1)
                return head + straight + tail
        return path                                   # could not straighten: keep A*

    def _astar(self, sx, sy, gx, gy):
        if self.cost is None:
            raise RuntimeError("plan() before set_map(): no map received")
        s = self.world_to_cell(sx, sy); g = self.world_to_cell(gx, gy)
        for (r, c) in (s, g):
            if not (0 <= r < self.h and 0 <= c < self.w):
                return None
        # robot may start inside inflation (just after a near-pass): snap out of it
        s = self._nearest_free(*s); g = self._nearest_free(*g)
        if s is None or g is None:
            return None
        nbr = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
               (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
        gscore = {s: 0.0}
        came = {}
        h = lambda n: math.hypot(n[0] - g[0], n[1] - g[1])  # noqa: E731
        openq = [(h(s), 0.0, s)]
        cost = self.cost
        while openq:
            _, gc, cur = heapq.heappop(openq)
            if cur == g:
                break
            if gc > gscore.get(cur, np.inf):
                continue
            r, c = cur
            for dr, dc, step in nbr:
                rr, cc = r + dr, c + dc
                if not (0 <= rr < self.h and 0 <= cc < self.w):
                    continue
                k = cost[rr, cc]
                if k == np.inf:
                    continue
                ng = gc + step * k
                if ng < gscore.get((rr, cc), np.inf):
                    gscore[(rr, cc)] = ng
                    came[(rr, cc)] = cur
                    heapq.heappush(openq, (ng + h((rr, cc)), ng, (rr, cc)))
        if g not in gscore:
            return None
        cells = [g]
        while cells[-1] != s:
            cells.append(came[cells[-1]])
        cells.reverse()
        pts = [self.cell_to_world(r, c) for r, c in cells]
        pts[-1] = (gx, gy) if cost[self.world_to_cell(gx, gy)] < np.inf else pts[-1]
        return pts
