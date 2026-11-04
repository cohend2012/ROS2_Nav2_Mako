"""Procedural OFFICE layouts for training + evaluation (the indoor counterpart of the
outdoor obstacle slots in env.py).

A layout is a 14 x 14 m floor with:
  * one corridor (width 1.4-2.4 m) and 2-3 rooms on each side,
  * one door per room in the corridor wall, width 0.80-1.20 m (real office doors are
    0.8-0.9 m; the M20 is 0.53 m wide, so 0.8 m leaves ~0.13 m a side),
  * furniture: desks (solid 1.4 x 0.7 m blocks), chairs (0.5 m), cabinets, kept 0.9 m
    clear of door openings,
  * people: 0-3 cylinders (r 0.25 m) walking the corridor at 0.4-1.2 m/s, some
    crossing through a door; they are mocap bodies, so the map never contains them.
Walls are 1.5 m tall: enough for the LiDAR slice (0.10-1.00 m above ground).
NOT modelled (documented gap): glass, which the real LiDAR sees through, so
glass walls must be marked in the map / keepout layer by hand.

Coordinates: x along the corridor, y across it.
"""
import math
from dataclasses import dataclass, field

import numpy as np

HALF = 7.0
WALL_T = 0.10
WALL_H = 1.5


@dataclass
class Box:
    x: float
    y: float
    hx: float
    hy: float
    hz: float
    yaw: float = 0.0
    kind: str = "wall"
    name: str = ""


@dataclass
class Person:
    path: list            # list of (x, y) waypoints, walked back and forth
    speed: float
    r: float = 0.25
    phase: float = 0.0    # metres along the path at t=0


@dataclass
class Room:
    name: str
    x0: float
    x1: float
    y0: float
    y1: float
    door: tuple           # (x, y) door centre on the corridor wall
    inside: tuple         # a free-ish point inside, for landmarks


@dataclass
class Layout:
    boxes: list = field(default_factory=list)
    people: list = field(default_factory=list)
    rooms: list = field(default_factory=list)
    corridor: tuple = (0.0, 2.0)  # (y centre, width)
    doors: list = field(default_factory=list)  # (x, y, width)


def _wall_x(boxes, x0, x1, y, gaps=(), name="wall"):
    """Horizontal wall from x0 to x1 at y, leaving `gaps` [(xc, width)] open."""
    cuts = sorted((xc - w / 2, xc + w / 2) for xc, w in gaps)
    a = x0
    for g0, g1 in cuts + [(x1, x1)]:
        if g0 - a > 0.05:
            boxes.append(Box((a + g0) / 2, y, (g0 - a) / 2, WALL_T / 2, WALL_H / 2, name=name))
        a = g1


def _wall_y(boxes, x, y0, y1, name="wall"):
    boxes.append(Box(x, (y0 + y1) / 2, WALL_T / 2, abs(y1 - y0) / 2, WALL_H / 2, name=name))


def generate(rng, n_people=None, door_range=(0.80, 1.20), corridor_range=(1.4, 2.4),
             furniture=True, room_names=None):
    L = Layout()
    cy = float(rng.uniform(-1.0, 1.0))
    cw = float(rng.uniform(*corridor_range))
    L.corridor = (cy, cw)
    yl, yh = cy - cw / 2, cy + cw / 2
    # outer walls
    _wall_x(L.boxes, -HALF, HALF, -HALF); _wall_x(L.boxes, -HALF, HALF, HALF)
    _wall_y(L.boxes, -HALF, -HALF, HALF); _wall_y(L.boxes, HALF, -HALF, HALF)
    names = iter(room_names or [])
    for side, (wy, ya, yb) in (("S", (yl, -HALF, yl)), ("N", (yh, yh, HALF))):
        n_rooms = int(rng.integers(2, 4))
        cuts = np.sort(rng.uniform(-HALF + 3.0, HALF - 3.0, n_rooms - 1))
        # enforce >= 3 m rooms
        edges = [-HALF] + [float(c) for c in cuts] + [HALF]
        edges = [edges[0]] + [e for i, e in enumerate(edges[1:-1], 1)
                              if e - edges[i - 1] >= 3.0 and edges[-1] - e >= 3.0] + [edges[-1]]
        gaps = []
        for i in range(len(edges) - 1):
            x0, x1 = edges[i], edges[i + 1]
            dw = float(rng.uniform(*door_range))
            dx = float(rng.uniform(x0 + 0.4 + dw / 2, x1 - 0.4 - dw / 2))
            gaps.append((dx, dw))
            L.doors.append((dx, wy, dw))
            if i > 0:
                _wall_y(L.boxes, x0, ya, yb, name="partition")
            name = next(names, f"room_{side}{i}")
            ins = ((x0 + x1) / 2, (ya + yb) / 2)
            L.rooms.append(Room(name, x0, x1, min(ya, yb), max(ya, yb), (dx, wy), ins))
            if furniture:
                _furnish(L, rng, x0, x1, min(ya, yb), max(ya, yb), (dx, wy))
        _wall_x(L.boxes, -HALF, HALF, wy, gaps, name="corridor_wall")
    # people
    k = int(rng.integers(0, 4)) if n_people is None else n_people
    for _ in range(k):
        speed = float(rng.uniform(0.4, 1.2))
        if rng.random() < 0.7 or not L.doors:      # walk the corridor
            lane = cy + float(rng.uniform(-cw / 2 + 0.35, cw / 2 - 0.35))
            path = [(-HALF + 1.0, lane), (HALF - 1.0, lane)]
        else:                                      # cross through a door
            dx, wy, _ = L.doors[int(rng.integers(len(L.doors)))]
            sgn = 1 if wy > cy else -1
            path = [(dx, cy - sgn * 0.3), (dx, wy + sgn * 1.5)]
        if rng.random() < 0.15:
            speed = 0.0                            # someone standing and talking
        L.people.append(Person(path, speed, phase=float(rng.uniform(0, 12))))
    return L


def _furnish(L, rng, x0, x1, y0, y1, door):
    placed = []
    def ok(x, y, r):
        if math.hypot(x - door[0], y - door[1]) < r + 0.9:
            return False
        if not (x0 + r + 0.15 < x < x1 - r - 0.15 and y0 + r + 0.15 < y < y1 - r - 0.15):
            return False
        return all(math.hypot(x - px, y - py) > r + pr + 0.75 for px, py, pr in placed)
    for kind, (hx, hy, hz), count in (("desk", (0.7, 0.35, 0.375), int(rng.integers(0, 3))),
                                      ("chair", (0.25, 0.25, 0.45), int(rng.integers(0, 4))),
                                      ("cabinet", (0.45, 0.25, 0.6), int(rng.integers(0, 2)))):
        r = math.hypot(hx, hy)
        for _ in range(count):
            for _try in range(25):
                x, y = float(rng.uniform(x0, x1)), float(rng.uniform(y0, y1))
                if ok(x, y, r):
                    yaw = float(rng.choice([0.0, math.pi / 2])) + float(rng.normal(0, 0.1))
                    L.boxes.append(Box(x, y, hx, hy, hz, yaw, kind))
                    placed.append((x, y, r)); break


def person_xy(p, t):
    """Position of a person at time t walking back and forth along p.path."""
    pts = np.asarray(p.path, float)
    seg = np.hypot(*np.diff(pts, axis=0).T)
    total = float(seg.sum())
    if total <= 0 or p.speed == 0:
        s = min(p.phase, total)
    else:
        s = (p.phase + p.speed * t) % (2 * total)
        s = s if s <= total else 2 * total - s
    for i, l in enumerate(seg):
        if s <= l or i == len(seg) - 1:
            f = s / l if l > 0 else 0.0
            return tuple(pts[i] + f * (pts[i + 1] - pts[i]))
        s -= l


def add_to_spec(spec, L, max_people=3):
    """Adds the layout's geoms + people (mocap) to an MjSpec worldbody."""
    import mujoco
    wb = spec.worldbody
    for i, b in enumerate(L.boxes):
        g = wb.add_geom()
        g.name = b.name + f"_{i}" if b.name else f"{b.kind}_{i}"
        g.type = mujoco.mjtGeom.mjGEOM_BOX
        g.size = [b.hx, b.hy, b.hz]
        g.pos = [b.x, b.y, b.hz]
        g.quat = [math.cos(b.yaw / 2), 0, 0, math.sin(b.yaw / 2)]
        g.rgba = {"wall": [0.85, 0.85, 0.82, 1], "corridor_wall": [0.85, 0.85, 0.82, 1],
                  "partition": [0.8, 0.82, 0.85, 1], "desk": [0.55, 0.4, 0.25, 1],
                  "chair": [0.2, 0.2, 0.25, 1], "cabinet": [0.5, 0.5, 0.55, 1]}.get(b.kind, [0.7, 0.7, 0.7, 1])
    for k in range(max_people):
        body = wb.add_body()
        body.name = f"person{k}"
        body.mocap = True
        body.pos = [60 + 3 * k, 60, 0.85]
        g = body.add_geom()
        g.name = f"person{k}_geom"
        g.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        g.size = [0.25, 0.85, 0]
        g.rgba = [0.2, 0.45, 0.8, 1]
