"""Live asset register for found dials: <report_dir>/asset_register.json, rewritten
(atomically) every time a dial is spotted, seen again, read, or described, so the record
exists from the FIRST sighting, not at the end of the mission.

One record per asset tag (DIAL-001, ...):
  asset_tag, landmark (found_gauge_N once confirmed), status
      SPOTTED -> READ | NEEDS_REVIEW | UNREADABLE   (or REJECTED / UNREACHED)
  location   {frame, x, y, uncertainty_m, facing_deg, sightings}
  first_seen / last_seen {time, mission_time_s, robot_pose, range_m, while_driving, image}
  detector   {name, best_score}
  reading    {value, units, fraction, confidence, status, method, time, robot_pose, range_m, image}
  viewing_spot [x, y, yaw]
  mounted_on {shape, size_m, guess}          from the LiDAR map (no AI needed)
  context    {mounted_on, surroundings, labels_visible, condition, anomaly, model, image}
                                             from the VLM (Cosmos / Claude), when connected
Positions are in the map frame, where localization believed the robot was.
"""
import json
import math
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

CONTEXT_QUESTION = (
    "This photo was taken by an inspection robot that found an analog gauge (asset {tag}). "
    "Describe what the gauge is mounted on and what is around it, for a maintenance record.")


def _now():
    return time.strftime("%Y-%m-%dT%H:%M:%S")


def mount_from_map(planner, x, y, search_m=1.2):
    """What the dial is on, from the map alone: the occupied blob nearest the dial,
    measured. Works with no AI at all (and on the robot's own LiDAR map)."""
    if planner is None or not hasattr(planner, "dist"):
        return None
    occ = np.asarray(planner.dist) <= 1e-6
    h, w = occ.shape
    r0, c0 = planner.world_to_cell(x, y)
    k = int(search_m / planner.res)
    best = None
    for dr in range(-k, k + 1):
        for dc in range(-k, k + 1):
            r, c = r0 + dr, c0 + dc
            if 0 <= r < h and 0 <= c < w and occ[r, c]:
                d = math.hypot(dr, dc)
                if best is None or d < best[0]:
                    best = (d, r, c)
    if best is None:
        return {"shape": "none mapped", "size_m": None, "guess": "free-standing (nothing mapped within 1.2 m)"}
    # flood fill the blob (bounded, 8-connected)
    seen, stack, cells = {best[1:]}, [best[1:]], []
    lim = int(4.0 / planner.res)
    while stack and len(cells) < 20000:
        r, c = stack.pop()
        cells.append((r, c))
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                n = (r + dr, c + dc)
                if n not in seen and 0 <= n[0] < h and 0 <= n[1] < w and occ[n] \
                        and abs(n[0] - best[1]) < lim and abs(n[1] - best[2]) < lim:
                    seen.add(n); stack.append(n)
    cells = np.array(cells, float) * planner.res
    ext = cells.max(axis=0) - cells.min(axis=0) + planner.res
    big, small = float(max(ext)), float(min(ext))
    if big < 0.35:
        shape, guess = "post", "post or small support"
    elif big > 1.0 and (small < 0.3 or big / max(small, 1e-6) >= 3.0):   # LiDAR blur thickens thin things
        shape, guess = "linear", "pipe, rack beam or wall"
    elif big > 1.6 and abs(ext[0] - ext[1]) < 0.3 * big:
        shape, guess = "large round/square", "tank or vessel"
    else:
        shape, guess = "box", "skid, enclosure or wellhead base"
    return {"shape": shape, "size_m": [round(float(ext[1]), 2), round(float(ext[0]), 2)], "guess": guess}


class AssetRegister:
    def __init__(self, path=None, request="", detector_name="", planner=None, describe=None,
                 save_png=None, clock=None, log=print):
        self.path, self.request, self.det_name = path, request, detector_name
        self.planner, self.describe, self.save_png = planner, describe, save_png
        self.clock, self.log = clock, log
        self.assets = {}
        self._lock = threading.Lock()
        self._wlock = threading.Lock()       # the describer thread writes too
        self._pool = ThreadPoolExecutor(max_workers=2) if describe else None
        self._pending = []
        self.write()

    def _mtime(self):
        return round(self.clock(), 1) if self.clock else None

    def spotted(self, cluster, pose, range_m, score, moving, png=None):
        tag = cluster["tag"]
        x, y = np.mean(cluster["pts"], axis=0)
        sd = float(np.mean(np.std(np.array(cluster["pts"]), axis=0))) if len(cluster["pts"]) > 1 else None
        seen = {"time": _now(), "mission_time_s": self._mtime(),
                "robot_pose": [round(float(v), 3) for v in pose] if pose else None,
                "range_m": round(float(range_m), 2), "while_driving": bool(moving)}
        with self._lock:
            a = self.assets.get(tag)
            if a is None:
                if png is not None and self.save_png:
                    seen["image"] = self.save_png(png, f"{tag}_first_seen")
                a = self.assets[tag] = {"asset_tag": tag, "type": "analog dial gauge", "landmark": None,
                                        "status": "SPOTTED", "first_seen": seen,
                                        "detector": {"name": self.det_name, "best_score": round(float(score), 2)},
                                        "mission": self.request}
                a["mounted_on"] = mount_from_map(self.planner, x, y)
            a["last_seen"] = seen
            a["detector"]["best_score"] = round(max(a["detector"]["best_score"], float(score)), 2)
            nrm = None
            if cluster.get("normals"):
                v = np.mean(np.array(cluster["normals"]), axis=0)
                if np.hypot(*v) > 1e-6:
                    # the face normal points OUT of the dial (towards whoever can read it)
                    nrm = round(math.degrees(math.atan2(v[1], v[0])), 1)
            a["location"] = {"frame": "map", "x": round(float(x), 3), "y": round(float(y), 3),
                             "uncertainty_m": None if sd is None else round(sd, 3),
                             "facing_deg": nrm, "sightings": len(cluster["pts"])}
        self.write()

    def read(self, tag, reading, pose, range_m, png=None, method="survey", context_png=None):
        status = ("UNREADABLE" if not reading.get("readable") else
                  "NEEDS_REVIEW" if reading.get("confidence", 0) < 0.7 else "READ")
        rec = {k: reading.get(k) for k in ("value", "units", "confidence", "notes")}
        rec.update(status=status, method=method, time=_now(), mission_time_s=self._mtime(),
                   robot_pose=[round(float(v), 3) for v in pose] if pose else None,
                   range_m=None if range_m is None else round(float(range_m), 2))
        if png is not None and self.save_png:
            rec["image"] = self.save_png(png, f"{tag}_read")
        with self._lock:
            a = self.assets.get(tag)
            if a is None:
                return
            a["reading"], a["status"] = rec, status
        self.write()
        if self._pool and context_png is not None and "context" not in a:
            self._pending.append(self._pool.submit(self._describe, tag, context_png))

    def _describe(self, tag, png):
        t0 = time.time()
        try:
            ctx = self.describe(png, CONTEXT_QUESTION.format(tag=tag))
        except Exception as e:                       # the mission never waits on / dies of the AI
            ctx = {"error": str(e)[:200]}
        if ctx is None:
            return
        ctx["latency_s"] = round(time.time() - t0, 1)
        if self.save_png:
            ctx["image"] = self.save_png(png, f"{tag}_context")
        with self._lock:
            self.assets[tag]["context"] = ctx
        self.log(f"[find] {tag} context: {str(ctx.get('mounted_on') or ctx.get('error'))[:80]}")
        self.write()

    def set(self, tag, **kw):
        with self._lock:
            if tag in self.assets:
                self.assets[tag].update(kw)
        self.write()

    def finish(self, timeout_s=120):
        for f in self._pending:
            try:
                f.result(timeout=timeout_s)
            except Exception:
                pass
        if self._pool:
            self._pool.shutdown(wait=False)
        self.write()
        return self.doc()

    def doc(self):
        with self._lock:
            return {"updated": _now(), "mission": self.request, "frame": "map",
                    "assets": [self.assets[k] for k in sorted(self.assets)]}

    def write(self):
        if not self.path:
            return
        with self._wlock:
            d = self.doc()
            os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
            tmp = self.path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(d, f, indent=1)
            os.replace(tmp, self.path)          # atomic: readers never see half a file
