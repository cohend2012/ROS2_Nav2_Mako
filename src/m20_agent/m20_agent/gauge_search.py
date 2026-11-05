"""The 'find the gauges' mission: survey -> look around -> detect -> localize ->
verify up close -> read -> save. Used by the executor (plan action `find_gauges`) and by
tools/agent/find_gauges.py (the MuJoCo gate, which scores it against hidden truth).

The robot is reached through the RobotIO object, which needs four extras on top of
pose / navigate / capture:
    io.survey_viewpoints()            -> [(x, y)]   coverage of the map's free space
    io.turn_to(yaw)                   -> bool       rotate in place (optional: falls
                                                    back to navigate(x, y, yaw))
    io.range_along(bearing, elev)     -> (range_m, (x, y)) | None   LiDAR along a camera ray
    io.face_normal(bearing, elev)     -> (nx, ny) | None            optional
"""
import io as _io
import math

import numpy as np

from m20_agent.gauge_dial import read_dial
from m20_agent.gauge_finder import (GaugeClusters, detect_dials, pixel_to_ray,
                                    verify_dial, viewing_spot)

UNKNOWN_SPEC = {"units": "fraction of scale", "min": 0.0, "max": 1.0, "unknown": True}


def _rgb(png):
    from PIL import Image
    return np.asarray(Image.open(_io.BytesIO(png)).convert("RGB"))


def _turn(io, yaw):
    if hasattr(io, "turn_to"):
        return io.turn_to(yaw)
    p = io.pose()
    return bool(p) and io.navigate(p[0], p[1], yaw, "look")[0]


HALF_FOV = math.radians(28)          # horizontal half-FOV of the 320x240, 45 deg-vertical camera


def _look_headings(io, n, look_range):
    """The headings worth turning to from here, in ONE sweep starting at the current
    heading (no back-and-forth). A heading is skipped when the LiDAR sees nothing within
    `look_range` across the camera's view there: no surface, no gauge to spot."""
    p = io.pose()
    yaw0 = p[2] if p else 0.0
    step = 2 * math.pi / n
    out = []
    for k in range(n):
        h = math.atan2(math.sin(yaw0 + k * step), math.cos(yaw0 + k * step))
        if look_range:
            seen = False
            for j in range(-6, 7):
                hit = io.range_along(h - yaw0 + j * HALF_FOV / 6, 0.0)
                if hit is not None and hit[0] <= look_range:
                    seen = True
                    break
            if not seen:
                continue
        out.append(h)
    return out


def search_and_read(io, vlm, registry, save_png=None, log=print, teach=True, headings=6,
                    should_stop=lambda: False, look_range=4.5):
    """Returns {"candidates", "sightings", "gauges": [record], "rejected", "unreached"}.
    Each saved record: name, position, viewing_spot, hits, reading (vlm.read_gauge dict)
    and status OK / NEEDS_REVIEW / UNREADABLE. Nothing that wasn't confirmed up close is
    saved as a gauge."""
    vps = io.survey_viewpoints()
    log(f"[find] {len(vps)} survey viewpoints")
    clusters, sightings, unreached = GaugeClusters(radius=0.6, min_hits=1), 0, 0
    for i, (vx, vy) in enumerate(vps):
        if should_stop():
            break
        ok, why = io.navigate(vx, vy, None, f"survey_{i}")
        if not ok:
            unreached += 1
            log(f"[find] survey point {i} unreachable: {why}")
            continue
        for yaw_k in _look_headings(io, headings, look_range):
            _turn(io, yaw_k)
            png = io.capture()
            if png is None:
                continue
            for (cx, cy, r, score) in detect_dials(_rgb(png)):
                b, e = pixel_to_ray(cx, cy)
                hit = io.range_along(b, e)
                if hit is None:
                    continue
                n = io.face_normal(b, e) if hasattr(io, "face_normal") else None
                clusters.add(hit[1][0], hit[1][1], n, score)
                sightings += 1
    cands = clusters.gauges()
    log(f"[find] survey done: {sightings} sightings -> {len(cands)} candidates")
    gauges, rejected = [], []
    n_existing = len([n for n in registry.names() if n.startswith("found_gauge_")])
    for j, g in enumerate(cands):
        if should_stop():
            break
        name = f"found_gauge_{n_existing + j + 1}"      # unique per candidate, saved or not
        rec = {"name": name, "position": [round(v, 2) for v in g["position"]], "hits": g["hits"]}
        best = None
        for standoff in (1.2, 0.9):                        # second try closer
            vx, vy, vyaw = viewing_spot(g["position"], g["normal"], io.pose()[:2], standoff)
            ok, why = io.navigate(vx, vy, vyaw, name)
            if not ok:
                rec["detail"] = why
                continue
            x, y, _ = io.pose()
            _turn(io, math.atan2(g["position"][1] - y, g["position"][0] - x))
            png = io.capture()
            if png is None or verify_dial(_rgb(png)) is None:
                continue
            r = vlm.read_gauge(png, name, UNKNOWN_SPEC)
            if best is None or r.get("confidence", 0) > best[0].get("confidence", 0):
                best = (r, png, (vx, vy, vyaw))
            if r.get("readable") and r.get("confidence", 0) >= 0.7:
                break
        if best is None:
            rec.update(status="REJECTED" if "detail" not in rec else "UNREACHED")
            rejected.append(rec)
            log(f"[find] {name}: {rec['status']} ({rec.get('detail', 'no dial seen up close')})")
            continue
        r, png, spot = best
        rec.update(viewing_spot=[round(v, 3) for v in spot], reading=r,
                   status=("UNREADABLE" if not r.get("readable") else
                           "NEEDS_REVIEW" if r.get("confidence", 0) < 0.7 else "OK"))
        if save_png:
            rec["image"] = save_png(png, name)
        if teach:
            try:
                registry.teach(name, *spot, aliases=[], type_="gauge",
                               gauge={k: v for k, v in UNKNOWN_SPEC.items() if k != "unknown"},
                               position=tuple(g["position"]))
            except ValueError as e:
                log(f"[find] {name}: not saved ({e})")
        gauges.append(rec)
        log(f"[find] {name} at {rec['position']}: {rec['status']} "
            f"{r.get('value')} {r.get('units')} (conf {r.get('confidence')})")
    return {"candidates": len(cands), "sightings": sightings, "gauges": gauges,
            "rejected": rejected, "unreached_survey_points": unreached, "survey_points": len(vps)}
