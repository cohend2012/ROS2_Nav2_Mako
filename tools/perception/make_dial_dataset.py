#!/usr/bin/env python3
"""Labelled training images for the YOLO dial detector, rendered from the MuJoCo sim.

    MUJOCO_GL=osmesa python3 tools/perception/make_dial_dataset.py --out <dir> --n 1600

The robot's own camera (320x240, 45 deg vertical FOV, posed exactly as SimRobotIO.capture)
is placed at random free spots on the oil-field site, half the time aimed near a gauge.
Each gauge plate's 4 corners are projected through the pinhole model to give an exact box;
a plate counts only when a ray from the camera reaches it (not hidden behind a tank) and
most of it is in frame. Domain randomization, so the detector learns "a dial" and not
these six faces:
  * dial faces re-rendered with random value, range, units and label every few frames
    (texture swapped on the GPU)
  * camera height / pitch jitter, robot heading jitter
  * negatives: office-scene frames (no gauges) and site frames facing away
Writes YOLO format: images/{train,val}/*.png, labels/{train,val}/*.txt (class 0 = dial),
and data.yaml.
"""
import argparse
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "src", "m20_ai_nav"))
sys.path.insert(0, os.path.join(REPO, "src", "m20_agent"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from m20_ai_nav.sim.env import M20NavEnv  # noqa: E402
from m20_ai_nav.sim.raster import rasterize  # noqa: E402
from m20_ai_nav.grid_planner import GridPlanner  # noqa: E402
from m20_agent.gauge_dial import render_dial  # noqa: E402

W, H, FOVY = 320, 240, 45.0
F = (H / 2) / math.tan(math.radians(FOVY / 2))
UNITS = ["psi", "bar", "kPa", "degC", "degF", "m3/h", "MPa", ""]


class Scene:
    def __init__(self, scene, seed):
        self.env = M20NavEnv(scene=scene, randomize=False, seed=seed, max_steps=10)
        self.env.reset(seed=seed, options={"start": (0.0, 0.0), "yaw": 0.0, "goal": (0.0, 0.0)})
        m, d = self.env.m, self.env.d
        mujoco.mj_forward(m, d)
        self.m, self.d = m, d
        self.z0 = float(d.qpos[2])
        self.q0 = d.qpos.copy()
        self.r = mujoco.Renderer(m, H, W)
        self.cam = mujoco.MjvCamera(); self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.plates = [g for g in range(m.ngeom)
                       if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith("gauge_")]
        self.tex = [t for t in range(m.ntex)
                    if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_TEXTURE, t) or "").startswith("tex_gauge")]
        self.robot_body = int(m.body("base_link").id) if mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_BODY, "base_link") >= 0 else -1
        p = GridPlanner(robot_radius=0.45, inflation_radius=0.8)
        p.set_map(*rasterize(m, d))
        self.planner = p
        free = np.argwhere(np.where(np.isinf(p.dist), 9.0, p.dist) > 0.6)
        self.free = [p.cell_to_world(int(a), int(b)) for a, b in free]

    def retexture(self, rng):
        for t in self.tex:
            lo = rng.choice([0, 0, 0, -1, -20])
            hi = rng.choice([1, 4, 6, 10, 16, 100, 150, 200, 250, 400, 600])
            if hi <= lo:
                hi = lo + 10
            v = rng.uniform(lo, hi)
            img = render_dial(v, lo, hi, rng.choice(UNITS),
                              rng.choice(["PG-", "TI-", "FL-", "PI-", "LT-"]) + str(rng.randint(100, 999)),
                              size=256)
            a = np.asarray(img.convert("RGB"), np.uint8)
            # face brightness jitter (dirt, glare, paint)
            a = np.clip(a.astype(np.float32) * rng.uniform(0.75, 1.1), 0, 255).astype(np.uint8)
            adr, n = self.m.tex_adr[t], 256 * 256 * 3
            self.m.tex_data[adr:adr + n] = a[::-1].reshape(-1)          # GL rows bottom-up
            mujoco.mjr_uploadTexture(self.m, self.r._mjr_context, t)

    def place(self, x, y, yaw, dz=0.0, pitch=0.0):
        d = self.d
        d.qpos[:] = self.q0
        d.qpos[0], d.qpos[1], d.qpos[2] = x, y, self.z0
        d.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        mujoco.mj_forward(self.m, d)
        ex, ey, ez = x + 0.30 * math.cos(yaw), y + 0.30 * math.sin(yaw), self.z0 + 0.05 + dz
        self.eye = np.array([ex, ey, ez])
        self.fwd = np.array([math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), math.sin(pitch)])
        self.left = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        self.up = np.cross(self.fwd, self.left)
        self.cam.lookat[:] = self.eye + 0.5 * self.fwd
        self.cam.distance, self.cam.azimuth, self.cam.elevation = 0.5, math.degrees(yaw), math.degrees(pitch)

    def render(self):
        self.r.update_scene(self.d, camera=self.cam)
        return self.r.render()

    def boxes(self):
        out = []
        for g in self.plates:
            c = self.d.geom_xpos[g]
            R = self.d.geom_xmat[g].reshape(3, 3)
            sx, sy = self.m.geom_size[g][:2]
            corners = [c + R @ np.array([a * sx, b * sy, 0.0]) for a in (-1, 1) for b in (-1, 1)]
            px = []
            for P in corners:
                v = P - self.eye
                fw = v @ self.fwd
                if fw < 0.3:
                    px = None
                    break
                px.append((W / 2 - F * (v @ self.left) / fw, H / 2 - F * (v @ self.up) / fw))
            if not px:
                continue
            # facing the camera? (the plate's back is a blank box)
            if (self.eye - c) @ R[:, 2] <= 0:
                continue
            xs, ys = [p[0] for p in px], [p[1] for p in px]
            x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
            area = (x1 - x0) * (y1 - y0)
            cx0, cx1, cy0, cy1 = max(0, x0), min(W, x1), max(0, y0), min(H, y1)
            if cx1 - cx0 < 4 or cy1 - cy0 < 4 or (cx1 - cx0) * (cy1 - cy0) < 0.6 * area:
                continue
            # visible: a ray from the eye to the plate centre hits the plate first
            vec = c - self.eye
            dist = float(np.linalg.norm(vec))
            gid = np.array([-1], np.int32)
            hit = mujoco.mj_ray(self.m, self.d, self.eye, vec / dist, None, 1, self.robot_body, gid)
            if gid[0] != g and not (hit > 0 and abs(hit - dist) < 0.05):
                continue
            out.append(((cx0 + cx1) / 2 / W, (cy0 + cy1) / 2 / H, (cx1 - cx0) / W, (cy1 - cy0) / H))
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--n", type=int, default=1600)
    ap.add_argument("--val", type=float, default=0.15)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    rng = random.Random(a.seed)
    for s in ("train", "val"):
        os.makedirs(os.path.join(a.out, "images", s), exist_ok=True)
        os.makedirs(os.path.join(a.out, "labels", s), exist_ok=True)
    site = Scene("oil_gas_field_gauges.xml", a.seed)
    office = Scene("office.xml", a.seed)
    gauges = [site.d.geom_xpos[g][:2].copy() for g in site.plates]
    normals = [site.d.geom_xmat[g].reshape(3, 3)[:2, 2].copy() for g in site.plates]
    stats = {"frames": 0, "boxes": 0, "neg": 0}
    for i in range(a.n):
        split = "val" if rng.random() < a.val else "train"
        if rng.random() < 0.12:                               # office negatives
            sc = office
            x, y = rng.choice(sc.free)
            yaw = rng.uniform(-math.pi, math.pi)
        else:
            sc = site
            if i % 8 == 0:
                sc.retexture(rng)
            if rng.random() < 0.7:                            # aimed near a gauge
                k = rng.randrange(len(gauges))
                d_ = rng.uniform(0.9, 6.5)
                off = math.radians(rng.uniform(-60, 60))
                n = normals[k] / (np.linalg.norm(normals[k]) or 1.0)
                base = math.atan2(n[1], n[0]) + off
                x, y = gauges[k][0] + (d_ + 0.3) * math.cos(base), gauges[k][1] + (d_ + 0.3) * math.sin(base)
                r_, c_ = sc.planner.world_to_cell(x, y)
                if not (0 <= r_ < sc.planner.h and 0 <= c_ < sc.planner.w) or sc.planner.dist[r_, c_] < 0.5:
                    x, y = rng.choice(sc.free)
                yaw = math.atan2(gauges[k][1] - y, gauges[k][0] - x) + math.radians(rng.uniform(-25, 25))
            else:
                x, y = rng.choice(sc.free)
                yaw = rng.uniform(-math.pi, math.pi)
        sc.place(x, y, yaw, dz=rng.uniform(-0.04, 0.04), pitch=math.radians(rng.uniform(-4, 4)))
        img = sc.render()
        boxes = sc.boxes() if sc is site else []
        name = f"{i:05d}"
        Image.fromarray(img).save(os.path.join(a.out, "images", split, name + ".png"))
        with open(os.path.join(a.out, "labels", split, name + ".txt"), "w") as f:
            for b in boxes:
                f.write("0 %.6f %.6f %.6f %.6f\n" % b)
        stats["frames"] += 1; stats["boxes"] += len(boxes); stats["neg"] += not boxes
        if i % 200 == 0:
            print(i, stats, flush=True)
    with open(os.path.join(a.out, "data.yaml"), "w") as f:
        f.write(f"path: {os.path.abspath(a.out)}\ntrain: images/train\nval: images/val\nnames:\n  0: dial\n")
    print("done", stats)


if __name__ == "__main__":
    main()
