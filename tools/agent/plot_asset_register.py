#!/usr/bin/env python3
"""The asset map: every tagged dial from asset_register.json drawn on the map the robot
used (its own LiDAR map when run with --map lidar), with the path it drove.

    python3 tools/agent/plot_asset_register.py <mission out dir>

Per dial: a marker at its logged map position (colour = status), an arrow the way the
dial faces, its tag, reading, and what it is mounted on; plus a table underneath.
Writes <dir>/asset_map.png.
"""
import json
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

COL = {"READ": (20, 150, 60), "NEEDS_REVIEW": (230, 150, 0), "UNREADABLE": (210, 40, 40),
       "SPOTTED": (90, 90, 220), "REJECTED": (140, 140, 140), "UNREACHED": (140, 140, 140)}


def main():
    out = sys.argv[1]
    reg = json.load(open(os.path.join(out, "asset_register.json")))
    trace = json.load(open(os.path.join(out, "trace.json")))
    mp = os.path.join(out, "lidar_map.png")
    half, res = 10.0, 0.05                               # lidar_map.build defaults
    S = 4                                                # px per cell
    if os.path.exists(mp):
        base = Image.open(mp).convert("RGB")             # already flipped: row 0 = +y
        title = "robot's own LiDAR map"
    else:
        base = Image.new("RGB", (400, 400), (245, 245, 240))
        title = "map"
    x0, x1, y0, y1 = -8.5, 8.5, -4.0, 7.0                # site area
    c0, c1 = int((x0 + half) / res), int((x1 + half) / res)
    r0, r1 = int((half - y1) / res), int((half - y0) / res)
    img = base.crop((c0, r0, c1, r1)).resize(((c1 - c0) * S, (r1 - r0) * S), Image.NEAREST)
    W, H = img.size
    to = lambda x, y: ((x - x0) / res * S, (y1 - y) / res * S)  # noqa: E731
    d = ImageDraw.Draw(img)
    f = ImageFont.load_default(size=15)
    fs = ImageFont.load_default(size=13)
    for gx in range(int(x0) + 1, int(x1) + 1):
        d.line([to(gx, y0), to(gx, y1)], fill=(215, 225, 240))
    for gy in range(int(y0) + 1, int(y1) + 1):
        d.line([to(x0, gy), to(x1, gy)], fill=(215, 225, 240))
    d.line([to(t[1], t[2]) for t in trace[::3]], fill=(70, 130, 200), width=2)
    hx, hy = trace[0][1], trace[0][2]
    px, py = to(hx, hy)
    d.rectangle([px - 7, py - 7, px + 7, py + 7], fill=(20, 20, 20))
    d.text((px + 10, py + 4), "start / home", fill=(20, 20, 20), font=fs)
    rows = []
    for a in reg["assets"]:
        L = a["location"]
        col = COL.get(a["status"], (0, 0, 0))
        px, py = to(L["x"], L["y"])
        unc = (L.get("uncertainty_m") or 0.0) / res * S
        d.ellipse([px - max(9, unc), py - max(9, unc), px + max(9, unc), py + max(9, unc)], outline=col, width=3)
        if L.get("facing_deg") is not None:
            th = math.radians(L["facing_deg"])
            d.line([(px, py), (px + 34 * math.cos(th), py - 34 * math.sin(th))], fill=col, width=3)
        rd = a.get("reading") or {}
        val = f"{rd.get('value')} {rd.get('units', '')}".replace("fraction of scale", "of scale") if rd else "-"
        mount = (a.get("mounted_on") or {}).get("guess", "")
        for (tx, ty, txt, fnt, c_) in ((px + 14, py - 30, a["asset_tag"], f, col),
                                       (px + 14, py - 13, val, fs, (30, 30, 30))):
            bb = d.textbbox((tx, ty), txt, font=fnt)
            d.rectangle([bb[0] - 2, bb[1] - 1, bb[2] + 2, bb[3] + 1], fill=(255, 255, 255))
            d.text((tx, ty), txt, fill=c_, font=fnt)
        rows.append((a["asset_tag"], a["status"], f"({L['x']:.2f}, {L['y']:.2f})",
                     f"±{L['uncertainty_m']:.2f} m" if L.get("uncertainty_m") is not None else "",
                     f"{L['facing_deg']:.0f}°" if L.get("facing_deg") is not None else "",
                     str(L.get("sightings")), val, f"{rd.get('confidence', '')}",
                     f"{a['first_seen']['mission_time_s']} s, {a['first_seen']['range_m']} m"
                     + (" driving" if a["first_seen"].get("while_driving") else ""),
                     rd.get("method", ""), mount,
                     ((a.get("context") or {}).get("mounted_on") or "")[:40]))
    hdr = ("tag", "status", "map x, y", "uncert.", "faces", "seen", "reading", "conf",
           "first seen", "read", "mounted on (LiDAR)", "context (VLM)")
    widths = [80, 70, 115, 70, 50, 45, 120, 45, 150, 120, 220, 220]
    th_ = 26 * (len(rows) + 2) + 40
    sheet = Image.new("RGB", (max(W, sum(widths) + 20), H + th_), (255, 255, 255))
    sheet.paste(img, (0, 0))
    d2 = ImageDraw.Draw(sheet)
    d2.text((10, H + 8), f"Asset register: {len(rows)} dials | {title} | mission: \"{reg['mission']}\" | "
            f"updated {reg['updated']}", fill=(0, 0, 0), font=f)
    y = H + 34
    for row in [hdr] + rows:
        x = 10
        for cell, w_ in zip(row, widths):
            d2.text((x, y), str(cell), fill=(0, 0, 0) if row is not hdr else (60, 60, 140), font=fs)
            x += w_
        y += 26
    sheet.save(os.path.join(out, "asset_map.png"))
    print("wrote", os.path.join(out, "asset_map.png"))


if __name__ == "__main__":
    main()
