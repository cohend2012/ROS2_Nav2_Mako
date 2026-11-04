"""Small 2D geometry helpers + path carrot selection (ROS-free)."""
import math

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def to_body(px, py, rx, ry, ryaw):
    """World point (px,py) into the frame of a robot at (rx,ry,ryaw)."""
    dx, dy = px - rx, py - ry
    c, s = math.cos(ryaw), math.sin(ryaw)
    return c * dx + s * dy, -s * dx + c * dy


def densify(p, step=0.10):
    """Resample a polyline to <= `step` spacing. The closest-point search works on
    VERTICES, so a sparse path (direct mode's [start, goal], a decimated Nav2 plan)
    would pin the carrot to the first vertex and the robot would orbit it — found
    by the ROS smoke test, 2026-11-04."""
    out = [p[0]]
    for a, b in zip(p[:-1], p[1:]):
        n = max(1, int(np.ceil(np.hypot(*(b - a)) / step)))
        for k in range(1, n + 1):
            out.append(a + (b - a) * (k / n))
    return np.array(out)


class CarrotTracker:
    """Pure-pursuit-style lookahead point on a path. Monotone: the closest-point
    index never moves backward, so a looping path can't snap the carrot behind us."""

    def __init__(self, lookahead=1.5, search_window=40):
        self.lookahead = lookahead
        self.window = search_window
        self.path = np.zeros((0, 2))
        self.idx = 0

    def set_path(self, xy):
        p = np.asarray(xy, float).reshape(-1, 2)
        if len(p) == 0:
            raise ValueError("empty path")
        self.path, self.idx = densify(p), 0

    def carrot(self, rx, ry):
        p = self.path
        hi = min(len(p), self.idx + self.window)
        d = np.hypot(p[self.idx:hi, 0] - rx, p[self.idx:hi, 1] - ry)
        self.idx += int(np.argmin(d))
        # walk forward from the closest point until `lookahead` path length
        acc, i = 0.0, self.idx
        while i + 1 < len(p):
            seg = float(np.hypot(*(p[i + 1] - p[i])))
            if acc + seg >= self.lookahead:
                t = (self.lookahead - acc) / seg if seg > 0 else 0.0
                return tuple(p[i] + t * (p[i + 1] - p[i]))
            acc += seg
            i += 1
        return tuple(p[-1])

    def remaining(self, rx, ry):
        p = self.path[self.idx:]
        if len(p) == 0:
            return 0.0
        d = math.hypot(p[0, 0] - rx, p[0, 1] - ry)
        if len(p) > 1:
            d += float(np.sum(np.hypot(*np.diff(p, axis=0).T)))
        return d
