"""Plan executor: validated plan -> robot actions -> inspection report. ROS-free.

The robot is reached through a RobotIO object (agent_node.RosRobotIO on the bus,
a fake in tests, the MuJoCo harness in tools/agent/gauge_gate.py):

    io.pose() -> (x, y, yaw) | None
    io.navigate(x, y, yaw, label) -> (ok: bool, why: str)   # via /m20/mission/run
    io.capture() -> PNG bytes | None                          # /camera/image_raw
    io.wait(seconds) -> (ok, why)
  for the find_gauges action, also (see gauge_search.py):
    io.survey_viewpoints() -> [(x, y)]
    io.range_along(bearing, elevation) -> (range_m, (x, y)) | None
    io.turn_to(yaw) -> bool            (optional: falls back to navigate in place)

Policy rules that make it safe to put a language model on top of a robot:
  * the plan is validated against the registry before ANY motion (plan.validate)
  * home is recorded from the robot's real pose at mission start
  * any failed motion step stops the mission, unless `continue_on_fail`; return_home
    is still attempted if the plan contained it (the robot should come back)
  * gauge readings below review_threshold confidence are flagged NEEDS_REVIEW,
    never silently accepted; the frame is saved with every reading
"""
import json
import math
import os
import time

from m20_agent.plan import validate, PlanError

REVIEW_THRESHOLD = 0.7


def _tag(lm):
    """A found gauge's asset tag (DIAL-NNN alias), if it has one."""
    return next((a for a in lm.aliases if a.upper().startswith("DIAL-")), None)


class Executor:
    def __init__(self, registry, backend, io, report_dir=None, continue_on_fail=False,
                 log=print, read_retries=1):
        self.reg, self.vlm, self.io = registry, backend, io
        self.report_dir = report_dir
        self.continue_on_fail = continue_on_fail
        self.read_retries = read_retries
        self.stop_requested = False
        self.log = log

    def run(self, request):
        t0 = time.time()
        report = {"request": request, "backend": self.vlm.name,
                  "started": time.strftime("%Y-%m-%dT%H:%M:%S"), "steps": [], "readings": []}
        pose = self.io.pose()
        if pose is None:
            return self._finish(report, False, "no robot pose: refusing to plan a mission", t0)
        self.reg.set_home(*pose)
        report["home"] = list(pose)
        try:
            raw = self.vlm.plan(request, self.reg.describe())
            steps = validate(raw, self.reg)
        except PlanError as e:
            return self._finish(report, False, f"plan rejected: {e}", t0)
        except RuntimeError as e:                       # BackendError and friends
            return self._finish(report, False, f"planner unavailable: {e}", t0)
        report["plan"] = steps
        report["rationale"] = raw.get("rationale", "")
        self.log(f"[agent] plan ({self.vlm.name}): {json.dumps(steps)}")
        wants_home = any(s["action"] == "return_home" for s in steps)
        ok_all = True
        self.stop_requested = False
        for i, s in enumerate(steps):
            if self.stop_requested:
                report["steps"].append({**s, "status": "CANCELLED"})
                ok_all = False
                continue
            if s["action"] == "return_home" and not ok_all:
                pass                                     # always try to come back
            elif not ok_all and not self.continue_on_fail:
                report["steps"].append({**s, "status": "SKIPPED"})
                continue
            ok, detail = self._step(s, report)
            report["steps"].append({**s, "status": "OK" if ok else "FAILED", "detail": detail})
            self.log(f"[agent] step {i} {s['action']} {s.get('target', '')}: "
                     f"{'OK' if ok else 'FAILED'} {detail}")
            ok_all &= ok
        if self.stop_requested:
            return self._finish(report, False, "stopped by operator", t0)
        if report.get("flagged"):
            ok_all = False
        if wants_home and not any(r["action"] == "return_home" and r["status"] == "OK"
                                  for r in report["steps"]):
            ok_all = False
        why = ("unreadable: " + ", ".join(report["flagged"])) if report.get("flagged") and ok_all is False \
            and all(st["status"] == "OK" for st in report["steps"]) else "one or more steps failed"
        return self._finish(report, ok_all, "ok" if ok_all else why, t0)

    def _goto(self, name):
        lm = self.reg.get(name)
        x, y, yaw = lm.approach
        return self.io.navigate(x, y, yaw, name)

    def stop(self):
        """Operator STOP: cancel the motion in flight and skip every remaining step
        (including return_home — stop means stop)."""
        self.stop_requested = True
        cancel = getattr(self.io, "cancel", None)
        if cancel:
            cancel()

    def _step(self, s, report):
        a = s["action"]
        if a == "save_place":
            p = self.io.pose()
            if p is None:
                return False, "no pose: cannot remember this place"
            try:
                lm = self.reg.teach(s["target"], *p, aliases=s.get("aliases", []))
            except ValueError as e:
                return False, str(e)
            return True, f"remembered {lm.name} at ({p[0]:.2f}, {p[1]:.2f})"
        if a == "set_gauge_scale":
            try:
                lm = self.reg.set_gauge_scale(s["target"], s["min"], s["max"], s["units"])
            except ValueError as e:
                return False, str(e)
            return True, f"{lm.name} scale set to {s['min']:g}-{s['max']:g} {s['units']}"
        if a == "wait":
            return self.io.wait(s["seconds"])
        if a == "return_home":
            return self._goto("home")
        if a == "find_gauges":
            return self._find_gauges(report)
        ok, why = self._goto(s["target"])
        if not ok or a == "goto":
            return ok, why
        lm = self.reg.get(s["target"])
        try:
            if a == "read_gauge":
                r, img_path = None, None
                for attempt in range(1 + self.read_retries):
                    self._aim(lm)                       # face the object, not the pose
                    png = self.io.capture()
                    if png is None:
                        return False, "no camera frame"
                    img_path = self._save_png(png, s["target"])
                    r = self.vlm.read_gauge(png, lm.name, lm.gauge)
                    if not r.get("readable"):
                        r = self._read_crop(png, lm) or r      # whole frame failed: the dial alone
                    if r.get("readable") and r.get("confidence", 0) >= REVIEW_THRESHOLD:
                        break
                r["attempts"] = attempt + 1
                status = ("UNREADABLE" if not r.get("readable") else
                          "NEEDS_REVIEW" if r.get("confidence", 0) < REVIEW_THRESHOLD else
                          "OK")
                rec = {"gauge": lm.name, "asset_tag": _tag(lm), **r, "status": status,
                       "image": img_path, "position": [round(float(v), 2) for v in lm.position]}
                report["readings"].append(rec)
                self._log_reading(rec, report)
                if status == "UNREADABLE":
                    # flagged, and the mission fails at the end, but the round carries on:
                    # one dirty dial must not leave every gauge after it unread
                    report.setdefault("flagged", []).append(lm.name)
                return True, f"{r.get('value')} {r.get('units')} (conf {r.get('confidence')}, {status})"
            self._aim(lm)
            png = self.io.capture()
            if png is None:
                return False, "no camera frame"
            img_path = self._save_png(png, s["target"])
            r = self.vlm.describe(png, s["question"])
            report["readings"].append({"inspect": lm.name, **r, "image": img_path})
            return True, r.get("answer", "")[:120]
        except RuntimeError as e:
            return False, f"VLM failed: {e}"

    def _find_gauges(self, report):
        from m20_agent.gauge_search import search_and_read
        if not all(hasattr(self.io, m) for m in ("survey_viewpoints", "range_along")):
            return False, "this robot interface cannot search (no survey_viewpoints/range_along)"
        try:
            res = search_and_read(self.io, self.vlm, self.reg, save_png=self._save_png,
                                  log=self.log, should_stop=lambda: self.stop_requested)
        except RuntimeError as e:
            return False, f"search failed: {e}"
        for g in res["gauges"]:
            rec = {"gauge": g["name"], "asset_tag": g.get("asset_tag"), **g["reading"],
                   "status": g["status"], "image": g.get("image"), "position": g["position"],
                   "viewing_spot": g.get("viewing_spot"), "found": True}
            report["readings"].append(rec)
            self._log_reading(rec, report)
        report["find_gauges"] = {k: v for k, v in res.items() if k != "gauges"}
        if res["survey_points"] and res["unreached_survey_points"] == res["survey_points"]:
            return False, "could not reach any survey point"
        n = {st: sum(g["status"] == st for g in res["gauges"])
             for st in ("OK", "NEEDS_REVIEW", "UNREADABLE")}
        return True, (f"found {len(res['gauges'])} gauge(s): {n['OK']} OK, {n['NEEDS_REVIEW']} "
                      f"NEEDS_REVIEW, {n['UNREADABLE']} UNREADABLE; {len(res['rejected'])} "
                      f"candidate(s) rejected/unreached; {res['unreached_survey_points']}/"
                      f"{res['survey_points']} survey points unreachable")

    def _read_crop(self, png, lm):
        """Find the dial nearest the image centre (we just aimed at it) and read only it."""
        try:
            from m20_agent.dial_detector import ClassicalDetector
            from m20_agent.gauge_search import crop_dial, _rgb
            rgb = _rgb(png)
            det = getattr(self.io, "detector", None) or ClassicalDetector()
            dets = det(rgb)
            if not dets:
                return None
            d = min(dets, key=lambda o: abs(o[0] - rgb.shape[1] / 2) + abs(o[1] - rgb.shape[0] / 2))
            _, cpng = crop_dial(rgb, d)
            r = self.vlm.read_gauge(cpng, lm.name, lm.gauge)
            return r if r.get("readable") else None
        except Exception:
            return None

    LOG_FIELDS = ("time", "request", "gauge", "asset_tag", "x", "y", "value", "units",
                  "confidence", "status", "image")

    def _log_reading(self, rec, report):
        """Append one row per reading to <report_dir>/inspection_log.csv: the running
        record of every gauge's location and value across missions."""
        if not self.report_dir:
            return
        import csv
        os.makedirs(self.report_dir, exist_ok=True)
        path = os.path.join(self.report_dir, "inspection_log.csv")
        new = not os.path.exists(path)
        pos = rec.get("position") or (None, None)
        row = {"time": time.strftime("%Y-%m-%dT%H:%M:%S"), "request": report.get("request", ""),
               "gauge": rec.get("gauge"), "asset_tag": rec.get("asset_tag") or "",
               "x": pos[0], "y": pos[1], "value": rec.get("value") if rec.get("readable") else "",
               "units": rec.get("units", ""), "confidence": rec.get("confidence"),
               "status": rec.get("status"), "image": rec.get("image") or ""}
        with open(path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=self.LOG_FIELDS)
            if new:
                w.writeheader()
            w.writerow(row)

    def _aim(self, lm):
        """Turn in place to the TRUE bearing of the object from where the robot
        actually stopped (arrival error of 0.2 m at 1.2 m standoff is ~10 deg; a
        45 deg camera can't afford that plus the goal checker's yaw slack)."""
        p = self.io.pose()
        if p is None:
            return
        bearing = math.atan2(lm.position[1] - p[1], lm.position[0] - p[0])
        err = math.atan2(math.sin(bearing - p[2]), math.cos(bearing - p[2]))
        if abs(err) > 0.05:
            self.io.navigate(p[0], p[1], bearing, lm.name + "_aim")

    def _save_png(self, png, name):
        if not self.report_dir:
            return None
        os.makedirs(self.report_dir, exist_ok=True)
        p = os.path.join(self.report_dir, f"{time.strftime('%H%M%S')}_{name}.png")
        with open(p, "wb") as f:
            f.write(png)
        return p

    def _finish(self, report, ok, detail, t0):
        report.update(success=bool(ok), detail=detail, duration_s=round(time.time() - t0, 1))
        if self.report_dir:
            os.makedirs(self.report_dir, exist_ok=True)
            p = os.path.join(self.report_dir, f"report_{time.strftime('%Y%m%d_%H%M%S')}.json")
            with open(p, "w") as f:
                json.dump(report, f, indent=2)
            report["report_path"] = p
        self.log(f"[agent] mission {'SUCCEEDED' if ok else 'FAILED'}: {detail}")
        return report


class FallbackBackend:
    """Primary VLM with the offline backend behind it: if the primary raises (link
    down, endpoint gone, refusal), the call is retried offline and the report says so."""

    def __init__(self, primary, fallback, log=print):
        self.p, self.f, self.log = primary, fallback, log
        self.name = f"{primary.name}+{fallback.name}"

    def _try(self, method, *args):
        try:
            return getattr(self.p, method)(*args)
        except RuntimeError as e:
            self.log(f"[agent] {self.p.name}.{method} failed ({e}); using {self.f.name}")
            out = getattr(self.f, method)(*args)
            if isinstance(out, dict):
                out = {**out, "fallback_reason": str(e)[:200]}
            return out

    def plan(self, *a): return self._try("plan", *a)
    def read_gauge(self, *a): return self._try("read_gauge", *a)
    def describe(self, *a): return self._try("describe", *a)


def make_backend(name, registry, offline_fallback=True):
    """Factory used by the node and the CLI. name: claude | cosmos | offline.
    Network backends get the offline backend as automatic fallback by default."""
    if name in ("claude", "cosmos") and offline_fallback:
        from m20_agent.vlm_offline import OfflineBackend
        return FallbackBackend(make_backend(name, registry, False), OfflineBackend(registry))
    if name == "claude":
        from m20_agent.vlm_claude import ClaudeBackend
        return ClaudeBackend()
    if name == "cosmos":
        from m20_agent.vlm_openai_compat import OpenAICompatBackend
        return OpenAICompatBackend()
    if name == "offline":
        from m20_agent.vlm_offline import OfflineBackend
        return OfflineBackend(registry)
    raise ValueError(f"unknown backend {name!r} (claude | cosmos | offline)")
