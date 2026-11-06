"""Record a sim mission as an MP4, live from the physics (not a replay).

Used by sim_mission.py --video <file.mp4>. Every `every` control ticks (0.1 s each) it
renders:
  left : a chase camera behind the robot (real legs/wheels, real contacts)
  right: the robot's own camera (what the dial detector sees) + a top-down map with the
         path so far, survey points, and every gauge the robot has saved so far
  top  : sim clock, current phase, plan step
Frames go straight into ffmpeg (libx264). At every=5 and 30 fps, 1 s of video = 15 s
of robot time.
"""
import math
import subprocess

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from m20_ai_nav.sim.raster import rasterize

W, H = 1280, 720
LEFT_W = 860
MAP_BOUNDS = (-8.0, 8.0, -3.2, 7.0)


class MissionVideo:
    def __init__(self, io, registry, path, every=5, fps=30, total_points=None, driver="",
                 request="", bounds=None, indoor=False):
        self.io, self.reg, self.every, self.driver = io, registry, every, driver
        self.request, self.indoor = request, indoor
        self.bounds = bounds or MAP_BOUNDS
        env = io.env
        env.m.vis.global_.offwidth = max(env.m.vis.global_.offwidth, LEFT_W)
        env.m.vis.global_.offheight = max(env.m.vis.global_.offheight, H)
        self.chase = mujoco.Renderer(env.m, H - 60, LEFT_W)
        self.cam = mujoco.MjvCamera(); self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.az = None
        self.n = 0
        self.font = ImageFont.load_default(size=22)
        self.small = ImageFont.load_default(size=15)
        self.tiny = ImageFont.load_default(size=12)
        self._map_base(env)
        self.path_img = self.base.copy()
        self.pd = ImageDraw.Draw(self.path_img)
        self.last_px = None
        self.total_points = total_points
        self.ff = subprocess.Popen(
            ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", f"{W}x{H}", "-r", str(fps), "-i", "-", "-c:v", "libx264", "-preset", "veryfast",
             "-crf", "26", "-pix_fmt", "yuv420p", "-movflags", "+faststart", path],
            stdin=subprocess.PIPE)
        orig = io._drive

        def drive(vx, wz):
            info = orig(vx, wz)
            self.n += 1
            if self.n % self.every == 0:
                self.frame(vx, wz)
            return info
        io._drive = drive

    # ------------------------------------------------------------------ map
    def _map_base(self, env):
        mujoco.mj_forward(env.m, env.d)
        data, w, h, res, ox, oy = rasterize(env.m, env.d)
        occ = np.asarray(data).reshape(h, w) >= 50
        x0, x1, y0, y1 = self.bounds
        self.mw = W - LEFT_W - 20
        self.ppm = self.mw / (x1 - x0)
        self.mh = int((y1 - y0) * self.ppm)
        img = Image.new("RGB", (self.mw, self.mh), (245, 245, 240))
        d = ImageDraw.Draw(img)
        for gx in range(int(x0), int(x1) + 1):
            d.line([self.to(gx, y0), self.to(gx, y1)], fill=(228, 228, 222))
        for gy in range(int(y0), int(y1) + 1):
            d.line([self.to(x0, gy), self.to(x1, gy)], fill=(228, 228, 222))
        ys, xs = np.nonzero(occ)
        half = res * self.ppm / 2 + 0.5
        for r, c in zip(ys, xs):
            wx, wy = ox + (c + 0.5) * res, oy + (r + 0.5) * res
            if x0 <= wx <= x1 and y0 <= wy <= y1:
                px, py = self.to(wx, wy)
                d.rectangle([px - half, py - half, px + half, py + half], fill=(140, 140, 140))
        self.base = img

    def to(self, x, y):
        x0, x1, y0, y1 = self.bounds
        return (x - x0) * self.ppm, (y1 - y) * self.ppm

    # ---------------------------------------------------------------- phase
    def phase(self):
        lab = getattr(self.io, "cur_label", None)
        moving = getattr(self.io, "in_nav", False)
        if lab is None:
            return "Planning"
        if lab.startswith("survey_") and hasattr(self.io, "view_poses"):
            i = int(lab.split("_")[1]) + 1
            of = f"/{self.total_points}" if self.total_points else ""
            return (f"SURVEY: driving to view {i}{of}" if moving else
                    f"SURVEY: photo at view {i}{of} (one look, no spin)")
        if lab.startswith("survey_"):
            i = int(lab.split("_")[1]) + 1
            of = f"/{self.total_points}" if self.total_points else ""
            return (f"SURVEY: driving to point {i}{of}" if moving else
                    f"SURVEY: looking around at point {i}{of} (8 headings)")
        if lab.startswith("found_gauge"):
            return (f"READ: driving to {lab}" if moving else f"READ: aiming + reading {lab}")
        if lab == "home":
            return "RETURN HOME" if moving else "BACK HOME"
        if lab.endswith("_aim"):
            return f"AIMING at {lab[:-4]}"
        if moving:
            return f"GOING TO {lab.upper()}"
        if lab in [l.name for l in self.reg.of_type("gauge")]:
            return f"READING {lab}"
        return lab

    # ---------------------------------------------------------------- frame
    def frame(self, vx, wz):
        io, env = self.io, self.io.env
        d = env.d
        x, y, z = (float(v) for v in d.qpos[0:3])
        yaw = env.yaw()
        az = math.degrees(yaw)
        if self.az is None:
            self.az = az
        da = (az - self.az + 180) % 360 - 180
        self.az += 0.08 * da                                     # smooth the chase camera
        self.cam.lookat[:] = [x, y, z]
        if self.indoor:                       # over the walls, not through them
            self.cam.distance, self.cam.azimuth, self.cam.elevation = 4.0, self.az, -62
        else:
            self.cam.distance, self.cam.azimuth, self.cam.elevation = 3.2, self.az, -28
        self.chase.update_scene(d, camera=self.cam)
        left = Image.fromarray(self.chase.render())

        onboard = io.capture()
        from io import BytesIO
        raw = Image.open(BytesIO(onboard)).convert("RGB")
        det = getattr(io, "detector", None)
        if det is not None and getattr(io, "dial_tags", None) is not None:
            cd = ImageDraw.Draw(raw)                        # what the detector sees, live
            for cx, cy, r, sc in det(np.asarray(raw)):
                cd.rectangle([cx - r / 0.92, cy - r / 0.92, cx + r / 0.92, cy + r / 0.92],
                             outline=(255, 170, 0), width=2)
                cd.text((cx - r, cy - r / 0.92 - 12), f"dial {sc:.2f}", fill=(255, 170, 0), font=self.tiny)
        cam_img = raw.resize((self.mw, int(self.mw * 0.75)))

        px, py = self.to(x, y)
        if self.last_px is not None:
            self.pd.line([self.last_px, (px, py)], fill=(60, 120, 200), width=2)
        self.last_px = (px, py)
        m = self.path_img.copy()
        md = ImageDraw.Draw(m)
        home = self.reg.get("home")
        if home is not None:
            hx, hy = self.to(*home.position)
            md.rectangle([hx - 5, hy - 5, hx + 5, hy + 5], fill=(30, 30, 30))
        tags = getattr(io, "dial_tags", None)
        for c in (tags.clusters if tags is not None else []):    # asset-tagged, not yet saved
            tx, ty = np.mean(c["pts"], axis=0)
            gx, gy = self.to(tx, ty)
            col = (30, 160, 70) if c.get("read") else (240, 140, 0)
            md.ellipse([gx - 6, gy - 6, gx + 6, gy + 6], outline=col, width=3)
            md.text((gx + 8, gy + 4), c["tag"] + (" read" if c.get("read") else ""), fill=col, font=self.tiny)
        for lm in self.reg.of_type("gauge"):
            gx, gy = self.to(*lm.position)
            md.ellipse([gx - 6, gy - 6, gx + 6, gy + 6], fill=(30, 160, 70))
            md.text((gx + 8, gy - 7), lm.name.replace("found_gauge_", "G"), fill=(20, 120, 50), font=self.small)
        md.ellipse([px - 6, py - 6, px + 6, py + 6], fill=(220, 60, 40))
        md.line([(px, py), (px + 14 * math.cos(yaw), py - 14 * math.sin(yaw))], fill=(220, 60, 40), width=3)

        f = Image.new("RGB", (W, H), (22, 24, 28))
        f.paste(left, (0, 60))
        f.paste(cam_img, (LEFT_W + 10, 60 + 24))
        f.paste(m, (LEFT_W + 10, 60 + 24 + cam_img.height + 30))
        dr = ImageDraw.Draw(f)
        t = io.t
        dr.text((16, 8), f"t = {int(t // 60):02d}:{int(t % 60):02d}  robot time", fill=(255, 255, 255), font=self.font)
        dr.text((300, 8), self.phase(), fill=(255, 210, 90), font=self.font)
        n_g = len(self.reg.of_type("gauge"))
        n_t = len(tags.clusters) if tags is not None else 0
        n_r = sum(1 for c in tags.clusters if c.get("read")) if tags is not None else 0
        dr.text((16, 36), f"request: \"{self.request}\"   driver: {self.driver}   dials tagged {n_t}, read {n_r}, saved {n_g}   "
                f"cmd vx {vx:+.2f} m/s  wz {wz:+.2f} rad/s   (video 15x speed)",
                fill=(190, 190, 190), font=self.small)
        dr.text((LEFT_W + 10, 64), f"robot camera (always on, {getattr(det, 'name', '')} detector)", fill=(190, 190, 190), font=self.small)
        dr.text((LEFT_W + 10, 60 + 24 + cam_img.height + 8), "map: path, tagged dials (orange; green = read), robot (red)",
                fill=(190, 190, 190), font=self.small)
        self.ff.stdin.write(np.asarray(f, np.uint8).tobytes())

    def close(self, hold_s=3.0):
        for _ in range(int(hold_s * 30)):
            self.frame(0.0, 0.0)
        self.ff.stdin.close()
        self.ff.wait()
