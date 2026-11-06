"""A VLM (Cosmos Reason / Claude) as the GLOBAL planner, a classical controller (Nav2) as the
local planner.

    route = VlmGlobalPlanner(vlm, planner).plan((sx, sy), (gx, gy), landmarks=reg)
    route["waypoints"]   -> [(x, y), ...] start .. goal, every leg checked
    route["source"]      -> "vlm" | "vlm+feedback" | "astar-fallback"

The model sees the robot's own occupancy map as an image (white = free, black =
obstacle, grey = unknown) with a 1 m coordinate grid, the start, the goal and named
landmarks, and answers with waypoints in metres. NOTHING it says is driven unchecked
(the same grounding rule as plan.validate):
  * every waypoint must be on mapped free floor with the robot's clearance
  * every straight leg between waypoints must keep that clearance the whole way
Failures go back to the model as plain feedback ("waypoint 3 (2.0, 1.5) is inside an
obstacle", "leg 2->3 crosses an obstacle near (1.2, 0.4)") for up to `retries` more tries;
after that the classical A* route is used and the report says so.
"""
import io as _io
import math
import time

import numpy as np

ROUTE_SYSTEM = """You are the global route planner for a wheel-legged inspection robot. \
You are given a top-down occupancy map the robot built with its LiDAR: WHITE is free floor, \
BLACK is an obstacle (tank, pipe, equipment, wall), GREY is unmapped. Grid lines are every \
1 m and labelled with map coordinates in metres (x to the right, y up). The GREEN dot is \
the robot (start), the RED dot is the goal. Plan a route as a short list of waypoints in \
metres. Every waypoint must be on WHITE floor at least {clear:.1f} m from any black or \
grey area, and the straight line between consecutive waypoints must stay that far from \
black and grey too. Go AROUND obstacles; do not cut corners. Use as few waypoints as you \
can (corners only). Start with the start point and end with the goal."""

ROUTE_INSTRUCTION = ("Start {start}, goal {goal}{lm}. Reply with ONLY a JSON object "
                     '{{"waypoints": [[x, y], ...], "rationale": "one sentence"}}.')


def render_map(planner, start, goal, landmarks=(), bounds=None, px_per_m=40):
    """PNG of the planning map for the model: free white, obstacles black, unknown grey,
    1 m grid with coordinate labels, start green, goal red, landmarks named."""
    from PIL import Image, ImageDraw, ImageFont
    occ = np.asarray(planner.dist) <= 1e-6
    unk = getattr(planner, "unk", None)
    h, w = occ.shape
    if bounds is None:                     # the equipment area + 1.5 m, and start/goal
        ys, xs = np.nonzero(occ)
        wx = planner.ox + (xs + 0.5) * planner.res; wy = planner.oy + (ys + 0.5) * planner.res
        bounds = (math.floor(min(wx.min() - 1.5, start[0] - 1, goal[0] - 1)),
                  math.ceil(max(wx.max() + 1.5, start[0] + 1, goal[0] + 1)),
                  math.floor(min(wy.min() - 1.5, start[1] - 1, goal[1] - 1)),
                  math.ceil(max(wy.max() + 1.5, start[1] + 1, goal[1] + 1)))
    x0, x1, y0, y1 = bounds
    W, H = int((x1 - x0) * px_per_m), int((y1 - y0) * px_per_m)
    img = Image.new("RGB", (W + 40, H + 30), (255, 255, 255))
    d = ImageDraw.Draw(img)
    to = lambda x, y: (40 + (x - x0) * px_per_m, (y1 - y) * px_per_m)  # noqa: E731
    cell = planner.res * px_per_m
    for r in range(h):
        for c in range(w):
            if occ[r, c] or (unk is not None and unk[r, c]):
                x, y = planner.cell_to_world(r, c)
                if x0 <= x <= x1 and y0 <= y <= y1:
                    px, py = to(x, y)
                    d.rectangle([px - cell / 2, py - cell / 2, px + cell / 2, py + cell / 2],
                                fill=(0, 0, 0) if occ[r, c] else (150, 150, 150))
    font = ImageFont.load_default(size=11)
    for gx in range(int(x0), int(x1) + 1):
        d.line([to(gx, y0), to(gx, y1)], fill=(120, 170, 230), width=1)
        d.text((to(gx, y0)[0] - 6, H + 4), str(gx), fill=(30, 60, 160), font=font)
    for gy in range(int(y0), int(y1) + 1):
        d.line([to(x0, gy), to(x1, gy)], fill=(120, 170, 230), width=1)
        d.text((4, to(x0, gy)[1] - 6), str(gy), fill=(30, 60, 160), font=font)
    for name, (lx, ly) in landmarks:
        px, py = to(lx, ly)
        d.text((px + 3, py - 12), name, fill=(150, 60, 0), font=font)
    for (x, y), col in ((start, (0, 170, 0)), (goal, (220, 0, 0))):
        px, py = to(x, y)
        d.ellipse([px - 7, py - 7, px + 7, py + 7], fill=col)
    buf = _io.BytesIO(); img.save(buf, "PNG")
    return buf.getvalue(), bounds


class VlmGlobalPlanner:
    def __init__(self, vlm, planner, clearance=None, retries=2, log=print):
        self.vlm, self.planner = vlm, planner
        self.clear = clearance if clearance is not None else planner.rr
        self.retries, self.log = retries, log

    def _check(self, wps):
        """Problems with a route, in words the model can act on ([] = drivable). The start
        and goal are given, not chosen, so only the waypoints in between are checked as
        points; every leg is checked."""
        p, errs = self.planner, []
        unk = getattr(p, "unk", None)
        for i, (x, y) in enumerate(wps):
            if i == 0 or i == len(wps) - 1:
                continue
            r, c = p.world_to_cell(x, y)
            if not (0 <= r < p.h and 0 <= c < p.w):
                errs.append(f"waypoint {i} ({x:.1f}, {y:.1f}) is off the map")
            elif unk is not None and unk[r, c]:
                errs.append(f"waypoint {i} ({x:.1f}, {y:.1f}) is in unmapped (grey) space")
            elif p._clear(x, y) < self.clear:
                errs.append(f"waypoint {i} ({x:.1f}, {y:.1f}) is {p._clear(x, y):.2f} m from an obstacle "
                            f"(needs {self.clear:.2f} m)")
        for i, (a, b) in enumerate(zip(wps, wps[1:])):
            n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (p.res * 0.5)))
            for k in range(1, n):
                x, y = a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n
                r, c = p.world_to_cell(x, y)
                bad = (not (0 <= r < p.h and 0 <= c < p.w)) or p._clear(x, y) < self.clear or \
                    (unk is not None and unk[r, c])
                if bad:
                    errs.append(f"leg {i}->{i + 1} passes too close to an obstacle near ({x:.1f}, {y:.1f})")
                    break
        return errs

    def plan(self, start, goal, landmarks=None, bounds=None):
        """Returns a dict: waypoints, source, tries, errors per try, latency, rationale."""
        lm = []
        if landmarks is not None:
            for n in landmarks.names():
                l_ = landmarks.get(n)
                if l_.type not in ("home", "gauge"):        # places, not every dial
                    lm.append((n, tuple(l_.position[:2])))
        png, bounds = render_map(self.planner, start, goal, lm, bounds)
        out = {"tries": [], "source": None, "map_png": png}
        sys_ = ROUTE_SYSTEM.format(clear=self.clear)
        fmt = lambda p: f"({p[0]:.1f}, {p[1]:.1f})"  # noqa: E731
        feedback = ""
        for t in range(1 + self.retries):
            t0 = time.time()
            try:
                ans = self.vlm.plan_route(png, sys_, ROUTE_INSTRUCTION.format(
                    start=fmt(start), goal=fmt(goal),
                    lm="" if not lm else "; landmarks are labelled on the map") + feedback)
            except Exception as e:                       # link down / refusal: classical route
                out["tries"].append({"error": str(e)[:200], "latency_s": round(time.time() - t0, 1)})
                break
            wps = [(float(a), float(b)) for a, b in (ans.get("waypoints") or [])]
            if wps and math.hypot(wps[0][0] - start[0], wps[0][1] - start[1]) > 0.3:
                wps = [tuple(start)] + wps
            if wps and math.hypot(wps[-1][0] - goal[0], wps[-1][1] - goal[1]) > 0.3:
                wps = wps + [tuple(goal)]
            errs = self._check(wps)
            out["tries"].append({"waypoints": [list(map(lambda v: round(v, 2), w)) for w in wps],
                                 "errors": errs, "latency_s": round(time.time() - t0, 1),
                                 "rationale": ans.get("rationale", "")})
            if len(wps) >= 2 and not errs:
                out.update(waypoints=wps, source="vlm" if t == 0 else "vlm+feedback",
                           rationale=ans.get("rationale", ""))
                return out
            feedback = ("\nYour previous route was rejected: " + "; ".join(errs[:6]) +
                        ". Fix those waypoints and reply again.")
        path = self.planner.plan(start[0], start[1], goal[0], goal[1])
        out.update(waypoints=[tuple(q[:2]) for q in path] if path else None, source="astar-fallback")
        return out
