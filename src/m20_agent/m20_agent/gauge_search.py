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


def _read_from_survey(io, vlm, cluster, rgb, det, range_m, log, max_range=2.8):
    """A survey photo taken by a planned view is already close and nearly head-on. If the
    dial passes the same up-close test the reading visit uses (near the image centre, big
    enough) and reads confidently, keep it: no trip back. Only the crop around that dial
    is read, so a second dial in the frame can't be read by mistake."""
    from PIL import Image
    cx, cy, r = det
    if range_m > max_range or abs(math.degrees(pixel_to_ray(cx, cy)[0])) > 15.0:
        log(f"[find]   survey read skipped: {range_m:.1f} m, {math.degrees(pixel_to_ray(cx, cy)[0]):+.0f} deg, r={r:.0f}px")
        return
    m = int(1.6 * r) + 4
    H, W = rgb.shape[:2]
    crop = rgb[max(0, int(cy) - m):min(H, int(cy) + m), max(0, int(cx) - m):min(W, int(cx) + m)]
    if verify_dial(np.ascontiguousarray(crop), max_bearing_deg=90.0) is None:
        log(f"[find]   survey read: dial too small to confirm ({range_m:.1f} m, r={r:.0f}px)")
        return
    buf = _io.BytesIO(); Image.fromarray(crop).resize((crop.shape[1] * 2, crop.shape[0] * 2)).save(buf, "PNG")
    png = buf.getvalue()
    rd = vlm.read_gauge(png, "candidate", UNKNOWN_SPEC)
    if not (rd.get("readable") and rd.get("confidence", 0) >= 0.7):
        log(f"[find]   survey read not confident ({range_m:.1f} m, r={r:.0f}px): "
            f"{rd.get('confidence')} {rd.get('notes', '')[:60]}")
        return
    prev = cluster.get("read")
    p = io.pose()
    if p is not None and (prev is None or rd.get("confidence", 0) > prev[0].get("confidence", 0)):
        cluster["read"] = (rd, png, tuple(p))


def search_and_read(io, vlm, registry, save_png=None, log=print, teach=True, headings=8,
                    should_stop=lambda: False, look_range=4.5, read_on_survey=True):
    """Returns {"candidates", "sightings", "gauges": [record], "rejected", "unreached"}.
    Each saved record: name, position, viewing_spot, hits, reading (vlm.read_gauge dict)
    and status OK / NEEDS_REVIEW / UNREADABLE. Nothing that wasn't confirmed up close is
    saved as a gauge."""
    # Preferred: planned VIEWS of every mapped surface (arrive facing it, one photo each).
    # Fallback: grid survey points with a look-around sweep at each.
    views = io.view_poses() if hasattr(io, "view_poses") else None
    vps = views if views else io.survey_viewpoints()
    log(f"[find] {len(vps)} {'planned views' if views else 'survey viewpoints'}")
    clusters, sightings, unreached = GaugeClusters(radius=0.6, min_hits=1), 0, 0
    for i, vp in enumerate(vps):
        if should_stop():
            break
        # Drive to the spot (any heading), THEN one short turn to the view heading: arriving
        # at a fixed heading made the planner loop in for a straight approach (2x the path).
        ok, why = io.navigate(vp[0], vp[1], None, f"survey_{i}")
        if not ok:
            unreached += 1
            log(f"[find] survey point {i} unreachable: {why}")
            continue
        looks = [vp[2]] if views else _look_headings(io, headings, look_range)
        for yaw_k in looks:
            if yaw_k is not None:
                _turn(io, yaw_k)
            png = io.capture()
            if png is None:
                continue
            rgb = _rgb(png)
            close = []
            for (cx, cy, r, score) in detect_dials(rgb):
                b, e = pixel_to_ray(cx, cy)
                hit = io.range_along(b, e)
                if hit is None:
                    continue
                n = io.face_normal(b, e) if hasattr(io, "face_normal") else None
                c = clusters.add(hit[1][0], hit[1][1], n, score)
                sightings += 1
                if read_on_survey and views:
                    _read_from_survey(io, vlm, c, rgb, (cx, cy, r), hit[0], log)
                    if c.get("read") is None and hit[0] <= 3.0:
                        close.append((c, hit[1]))
            # a dial close by but off-centre: turn to it here (a few seconds) rather than
            # drive back for it later
            for c, (hx, hy) in close[:2]:
                p = io.pose()
                if p is None or c.get("read") is not None:
                    continue
                _turn(io, math.atan2(hy - p[1], hx - p[0]))
                png2 = io.capture()
                if png2 is None:
                    continue
                rgb2 = _rgb(png2)
                for (cx, cy, r, score) in detect_dials(rgb2):
                    b, e = pixel_to_ray(cx, cy)
                    hit = io.range_along(b, e)
                    if hit is not None and math.hypot(hit[1][0] - hx, hit[1][1] - hy) < 0.6:
                        _read_from_survey(io, vlm, c, rgb2, (cx, cy, r), hit[0], log)
                        break
    cands = clusters.gauges()
    log(f"[find] survey done: {sightings} sightings -> {len(cands)} candidates")
    gauges, rejected = [], []
    n_existing = len([n for n in registry.names() if n.startswith("found_gauge_")])
    # read-on-survey hits first (no driving), then the rest nearest-first from here
    done = [g for g in cands if g.get("read")]
    todo, cur = [g for g in cands if not g.get("read")], (io.pose() or (0.0, 0.0))[:2]
    order = list(done)
    while todo:
        nxt = min(todo, key=lambda g: math.hypot(g["position"][0] - cur[0], g["position"][1] - cur[1]))
        order.append(nxt); todo.remove(nxt); cur = nxt["position"]
    for j, g in enumerate(order):
        if should_stop():
            break
        name = f"found_gauge_{n_existing + j + 1}"      # unique per candidate, saved or not
        rec = {"name": name, "position": [round(v, 2) for v in g["position"]], "hits": g["hits"]}
        best = g.get("read")
        if best is not None:
            rec["read_on_survey"] = True
        for standoff in (() if best is not None else (1.2, 0.9)):   # second try closer
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
