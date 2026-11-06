#!/usr/bin/env python3
"""FIND THE GAUGES with no prior positions, in MuJoCo (vendor M20 physics, the real
oil-field gauge scene). Gate G6.

    MUJOCO_GL=osmesa python3 tools/agent/find_gauges.py --out docs/ai_nav/find_gauges/run1 [--seed 1]

The agent's landmark file is loaded WITH ALL GAUGES REMOVED. The robot then:
  1. survey: drives coverage viewpoints over the site map (every ~3 m of free space)
  2. looks around at each: 8 headings, a camera frame each
  3. detects dials per frame (gauge_finder.detect_dials), turns pixel -> bearing,
     takes the LiDAR range along that ray -> a map point; the face normal comes from
     2 more rays at +-1.5 deg
  4. clusters sightings within 0.6 m into candidates; goes to a viewing spot 1.2 m in
     front of each face and VERIFIES a dial is there (else REJECTED, nothing saved)
  5. reads it (retry at 0.9 m if not confident) and saves it as a new gauge landmark
     (fraction of scale: with no prior spec the offline reader can't know the units;
     a VLM backend reads the scale text)
This runs the agent's own code (m20_agent.gauge_search.search_and_read, the executor's
find_gauges action); only the robot interface (FinderIO) is the simulator's.
Scored against tools/sim/gauges/truth.json (never seen by the robot):
  found = a saved gauge within 0.6 m of a true plate; false positive = a saved gauge
  matching none; reading error = |read fraction - true fraction| (percent of full scale).
Navigation is real: every move goes through A* + NavCore + the learned (or --policy
baseline) controller on the vendor physics, and failed legs are skipped, not faked.
"""
import argparse
import io
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "src", "m20_ai_nav"))
sys.path.insert(0, os.path.join(REPO, "src", "m20_agent"))
sys.path.insert(0, os.path.join(REPO, "tools", "ai_nav"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from m20_ai_nav.sim.env import M20NavEnv  # noqa: E402
from m20_agent.gauge_finder import CAM_FORWARD, survey_viewpoints  # noqa: E402
from m20_agent.gauge_search import search_and_read  # noqa: E402
from m20_agent.landmarks import LandmarkRegistry  # noqa: E402
from m20_agent.vlm_offline import OfflineBackend  # noqa: E402
from sim_mission import LANDMARKS, SimRobotIO, TRUTH  # noqa: E402

BOUNDS = (-7.5, 7.5, -2.6, 6.5)      # the site pad north of the ground pipelines


class FinderIO(SimRobotIO):
    """SimRobotIO + a LiDAR-equivalent range query along a camera ray."""

    def range_along(self, bearing, elev):
        """Range along the camera ray, measured in the TRUE scene, but placed on the map
        from where localization says the robot is (pose()), with optional range noise and
        dropouts (range_noise): what the robot really gets."""
        d = self.env.d
        x, y, z = d.qpos[0:3]
        yaw = self.env.yaw()
        eye = np.array([x + CAM_FORWARD * math.cos(yaw), y + CAM_FORWARD * math.sin(yaw), z + 0.05])
        h = yaw + bearing
        vec = np.array([math.cos(elev) * math.cos(h), math.cos(elev) * math.sin(h), math.sin(elev)])
        gid = np.array([-1], np.int32)
        dist = mujoco.mj_ray(self.env.m, d, eye, vec, np.array([1, 0, 0, 0, 0, 0], np.uint8), 1, -1, gid)
        if gid[0] < 0 or dist <= 0 or dist > 12.0:
            return None
        rn = getattr(self, "range_noise", None)
        if rn:
            if self._prng.random() < rn["dropout"]:
                return None
            dist = dist + self._prng.normal(0, rn["sigma_m"])
        bx, by, byaw = self.pose()                     # believed pose
        bh = byaw + bearing
        bvec = np.array([math.cos(elev) * math.cos(bh), math.cos(elev) * math.sin(bh), math.sin(elev)])
        beye = np.array([bx + CAM_FORWARD * math.cos(byaw), by + CAM_FORWARD * math.sin(byaw), z + 0.05])
        hit = beye + dist * bvec
        return dist, (float(hit[0]), float(hit[1]))

    def turn_to(self, yaw, timeout_s=15.0):
        """Rotate in place to `yaw` (NavCore's shielded align: blocked turns stop)."""
        from m20_ai_nav.core import SUCCEEDED, STUCK, BLOCKED
        x, y, _ = self.pose()
        self.core.set_path([(x, y), (x, y)], goal_yaw=yaw)
        t_end, blocked = self.t + timeout_s, 0
        while self.t < t_end:
            px, py, pyaw = self.pose()
            v = self.env.d.qvel
            cmd = self.core.tick(self.t, px, py, pyaw, self.env.scan,
                                 math.cos(pyaw) * v[0] + math.sin(pyaw) * v[1], float(v[5]))
            if cmd.state in (SUCCEEDED, STUCK):
                return cmd.state == SUCCEEDED
            blocked += cmd.state == BLOCKED
            if blocked > 20:
                return False                 # no room to turn here: skip this heading
            self._drive(cmd.vx, cmd.wz)
        return False

    def face_normal(self, bearing, elev):
        a = self.range_along(bearing + math.radians(1.5), elev)
        b = self.range_along(bearing - math.radians(1.5), elev)
        if not a or not b:
            return None
        (ax, ay), (bx, by) = a[1], b[1]
        tx, ty = ax - bx, ay - by
        L = math.hypot(tx, ty)
        if L < 1e-3:
            return None
        nx, ny = -ty / L, tx / L
        x, y = self.pose()[:2]
        mx, my = (ax + bx) / 2, (ay + by) / 2
        if (x - mx) * nx + (y - my) * ny < 0:       # point the normal at the robot
            nx, ny = -nx, -ny
        return (nx, ny)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--model", default=os.path.join(REPO, "src", "m20_ai_nav", "models", "m20_nav_policy.onnx"))
    ap.add_argument("--policy", default="learned", choices=["learned", "baseline"])
    ap.add_argument("--spacing", type=float, default=2.0)
    ap.add_argument("--start", default="0,0,0")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    t_wall = time.time()
    sx, sy, syaw = (float(v) for v in a.start.split(","))
    env = M20NavEnv(scene="oil_gas_field_gauges.xml", randomize=True, seed=a.seed, max_steps=10 ** 9)
    env.reset(seed=a.seed, options={"start": (sx, sy), "yaw": syaw, "goal": (sx, sy)})
    io_ = FinderIO(env, a.model)
    if a.policy == "baseline":
        from evaluate import Baseline
        io_.core.policy = Baseline()
    log = []
    say = lambda s: (log.append(s), print(s, flush=True))  # noqa: E731

    # The agent's OWN search (m20_agent.gauge_search, = plan action find_gauges), on a
    # registry with every gauge removed, offline reader (fraction of scale).
    reg = LandmarkRegistry.load(LANDMARKS, os.path.join(a.out, "taught.yaml"))
    for n in [n for n in reg.names() if reg.get(n).type == "gauge"]:
        reg._lm.pop(n)
    io_.survey_viewpoints = lambda: survey_viewpoints(io_.planner, (sx, sy), spacing=a.spacing,
                                                      clear=0.9, bounds=BOUNDS)
    sightings, raw_range = [], io_.range_along

    def range_logged(b, e):
        hit = raw_range(b, e)
        if hit is not None:
            rx, ry, ryaw = io_.pose()
            sightings.append({"robot": [round(rx, 2), round(ry, 2), round(ryaw, 2)],
                              "bearing_deg": round(math.degrees(b), 1), "range": round(hit[0], 2),
                              "map_xy": [round(hit[1][0], 2), round(hit[1][1], 2)]})
        return hit
    io_.range_along = range_logged

    def save_png(png, name):
        path = os.path.join(a.out, f"{name}.png")
        Image.open(io.BytesIO(png)).save(path)
        return path
    res_s = search_and_read(io_, OfflineBackend(reg), reg, save_png=save_png, log=say)
    report = [dict(g, fraction=g["reading"].get("value") if g["reading"].get("readable") else None,
                   confidence=g["reading"].get("confidence")) for g in res_s["gauges"]]
    unreached = [r for r in res_s["rejected"] if r["status"] == "UNREACHED"]
    rejected = [r for r in res_s["rejected"] if r["status"] == "REJECTED"]
    vps, nav_fail, found = (range(res_s["survey_points"]), res_s["unreached_survey_points"],
                            range(res_s["candidates"]))

    # score vs hidden truth
    truth = json.load(open(TRUTH))
    located = report + unreached         # localized (unreached ones only not read)
    matched, rows = set(), []
    for t_name, t in truth.items():
        tx, ty = t["plate"][:2]
        best = min(located, key=lambda r: math.hypot(r["position"][0] - tx, r["position"][1] - ty), default=None)
        dmin = math.hypot(best["position"][0] - tx, best["position"][1] - ty) if best else None
        hit = best is not None and dmin < 0.6 and best["name"] not in matched
        if hit:
            matched.add(best["name"])
        tf = (t["value"] - t["min"]) / (t["max"] - t["min"])
        err = (abs(best["fraction"] - tf) * 100 if hit and best.get("fraction") is not None else None)
        rows.append({"gauge": t_name, "found": bool(hit), "as": best["name"] if hit else None,
                     "pos_err_m": round(dmin, 2) if hit else None, "true_frac": round(tf, 4),
                     "read_frac": best.get("fraction") if hit else None,
                     "err_pct_fs": round(err, 2) if err is not None else None,
                     "status": best.get("status") if hit else "NOT FOUND"})
    n_found = sum(r["found"] for r in rows)
    read_ok = sum(1 for r in rows if r["err_pct_fs"] is not None and r["err_pct_fs"] <= 5.0)
    res = {"seed": a.seed, "policy": a.policy, "viewpoints": len(vps), "viewpoints_unreachable": nav_fail,
           "sightings": len(sightings), "candidates": len(found), "rejected_up_close": len(rejected),
           "gauges_saved": len(report), "unreached": len(unreached), "found": n_found, "of": len(truth),
           "false_positives": len(located) - len(matched), "read_within_5pct": read_ok,
           "confident_wrong": sum(1 for r in rows if r["status"] == "OK" and r["err_pct_fs"] is not None
                                  and r["err_pct_fs"] > 5.0),
           "sim_time_s": round(io_.t, 1), "wall_s": round(time.time() - t_wall, 1),
           "rows": rows, "saved": report, "rejected": rejected, "unreached_list": unreached, "log": log}
    json.dump(res, open(os.path.join(a.out, "find_gauges.json"), "w"), indent=2)
    json.dump(sightings, open(os.path.join(a.out, "sightings.json"), "w"))
    json.dump(io_.trace, open(os.path.join(a.out, "trace.json"), "w"))
    print(json.dumps({k: res[k] for k in ("found", "of", "false_positives", "read_within_5pct",
                                          "confident_wrong", "viewpoints", "viewpoints_unreachable",
                                          "sim_time_s")}))
    for r in rows:
        print(r)


if __name__ == "__main__":
    main()
