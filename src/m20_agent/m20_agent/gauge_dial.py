"""Analog gauge dial convention: renderer (sim textures) + classical reader (offline).

Convention (single source for both sides): 270 deg sweep, MIN at the 7:30 position,
increasing CLOCKWISE through 12 o'clock to MAX at 4:30. The needle is red with a
dark hub, the face is white, and the numbers are black.

read_dial() is the no-model fallback reader. It finds the bright dial face in the
frame, finds red needle pixels inside it, and fits the needle direction from the hub
outward. It is deliberately simple: it is the CI and offline baseline, and the
reference that VLM readings are checked against in sim (G5). On real gauges, prefer
a VLM or the ETH pipeline (Reitsma et al., ICRA 2024).
"""
import math

import numpy as np

SWEEP_DEG = 270.0
START_DEG = 225.0          # math angle (CCW from +x) of MIN: 7:30 position


def value_to_angle(value, vmin, vmax):
    """Math angle (deg, CCW from +x, image y up) of the needle for `value`."""
    f = (min(max(value, vmin), vmax) - vmin) / (vmax - vmin)
    return START_DEG - f * SWEEP_DEG


def angle_to_value(angle_deg, vmin, vmax):
    a = (START_DEG - angle_deg) % 360.0
    if a > SWEEP_DEG:                      # in the dead zone at the bottom: snap to nearest end
        a = SWEEP_DEG if a - SWEEP_DEG < (360.0 - a) else 0.0
    return vmin + a / SWEEP_DEG * (vmax - vmin)


def render_dial(value, vmin, vmax, units, label, size=256):
    """PIL RGB image of a gauge face showing `value`."""
    from PIL import Image, ImageDraw, ImageFont
    img = Image.new("RGB", (size, size), (40, 40, 44))
    d = ImageDraw.Draw(img)
    c = size / 2; r = size * 0.46
    d.ellipse([c - r, c - r, c + r, c + r], fill=(245, 245, 240), outline=(20, 20, 20),
              width=max(2, size // 64))
    try:
        font = ImageFont.load_default(size=max(10, size // 14))
        small = ImageFont.load_default(size=max(8, size // 18))
    except TypeError:                      # Pillow < 10.1
        font = small = ImageFont.load_default()
    for k in range(11):                    # major ticks + labels
        v = vmin + k * (vmax - vmin) / 10
        a = math.radians(value_to_angle(v, vmin, vmax))
        x0, y0 = c + 0.80 * r * math.cos(a), c - 0.80 * r * math.sin(a)
        x1, y1 = c + 0.95 * r * math.cos(a), c - 0.95 * r * math.sin(a)
        d.line([x0, y0, x1, y1], fill=(10, 10, 10), width=max(2, size // 80))
        if k % 2 == 0:
            tx, ty = c + 0.64 * r * math.cos(a), c - 0.64 * r * math.sin(a)
            txt = f"{v:g}"
            d.text((tx, ty), txt, fill=(10, 10, 10), font=small, anchor="mm")
    d.text((c, c + 0.42 * r), units, fill=(10, 10, 10), font=small, anchor="mm")
    d.text((c, c - 0.30 * r), label, fill=(10, 10, 10), font=small, anchor="mm")
    a = math.radians(value_to_angle(value, vmin, vmax))
    d.line([c - 0.15 * r * math.cos(a), c + 0.15 * r * math.sin(a),
            c + 0.78 * r * math.cos(a), c - 0.78 * r * math.sin(a)],
           fill=(210, 20, 20), width=max(3, size // 40))
    d.ellipse([c - r * 0.07, c - r * 0.07, c + r * 0.07, c + r * 0.07], fill=(30, 30, 30))
    return img


def _round_components(mask, min_px=150):
    """4-connected components of `mask` that look like a dial seen ~face-on: a
    roughly square bbox, the circle inscribed in it mostly filled, and almost NO
    pixels in the bbox corners (a round face never reaches them; a tank shell or
    a sky-merged blob does). Yields ((y0, y1, x0, x1), n_px)."""
    H, W = mask.shape
    seen = np.zeros_like(mask, bool)
    for sy, sx in zip(*np.nonzero(mask)):
        if seen[sy, sx]:
            continue
        stack = [(sy, sx)]; seen[sy, sx] = True
        ys, xs = [], []
        while stack:
            y, x = stack.pop(); ys.append(y); xs.append(x)
            for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
                if 0 <= ny < H and 0 <= nx < W and mask[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True; stack.append((ny, nx))
        n = len(ys)
        if n < min_px:
            continue
        ys, xs = np.array(ys), np.array(xs)
        y0, y1, x0, x1 = ys.min(), ys.max(), xs.min(), xs.max()
        h, w = y1 - y0 + 1, x1 - x0 + 1
        if not 0.6 < h / w < 1.6:
            continue
        cy, cx, r = (y0 + y1) / 2.0, (x0 + x1) / 2.0, max(h, w) / 2.0
        outside = np.mean(np.hypot((xs - cx) / (w / 2.0), (ys - cy) / (h / 2.0)) > 1.08)
        filled = n / (math.pi * (h / 2.0) * (w / 2.0))
        if outside < 0.03 and filled > 0.55:
            yield (y0, y1, x0, x1), n


def _largest_round_component(mask, min_px=150):
    best = None
    for comp in _round_components(mask, min_px):
        if best is None or comp[1] > best[1]:
            best = comp
    return best


def read_dial(rgb):
    """rgb: HxWx3 uint8 frame containing ONE dial. Returns (fraction in [0,1],
    confidence in [0,1], detail) where fraction = (value - min) / (max - min)."""
    a = np.asarray(rgb).astype(np.int16)
    R, G, B = a[..., 0], a[..., 1], a[..., 2]
    # face = bright, unsaturated pixels; threshold RELATIVE to the frame (scene
    # lighting greys the white face: measured ~190-210 in the sim camera)
    lum = (R + G + B) / 3.0
    sat = a.max(axis=2) - a.min(axis=2)
    # threshold LADDER: lighting varies across the face and the frame, so try
    # bands from strict to loose and keep the largest component that passes the
    # roundness test (a merged blob fails it, so loosening can't create a false dial)
    top = np.percentile(lum, 97)
    comp = None
    for drop in (30.0, 45.0, 60.0, 75.0, 90.0):
        face = (sat < 30) & (lum > max(95.0, top - drop))
        c = _largest_round_component(face)
        if c is not None and (comp is None or c[1] > comp[1]):
            comp = c
    if comp is None:
        return None, 0.0, f"no dial face found ({int(face.sum())} bright px, none dial-shaped)"
    (y0, y1, x0, x1), n_face = comp
    cy, cx = (y0 + y1) / 2.0, (x0 + x1) / 2.0
    rad = max(y1 - y0, x1 - x0) / 2.0
    H, W = face.shape
    gy, gx = np.mgrid[0:H, 0:W]
    in_dial = np.hypot(gx - cx, gy - cy) <= rad * 1.02
    # relative redness (as gauge_finder.detect_dials): a needle under side light or
    # seen off-axis is ~100,40,40, which an absolute R > 110 test misses
    red = (R > 50) & (R > 1.6 * G) & (R > 1.6 * B) & (R - G > 30) & in_dial
    yy, xx = np.nonzero(red)
    if len(yy) < 8:
        return None, 0.0, f"no needle found ({len(yy)} red px in dial)"
    # needle direction = principal axis of the red pixels (independent of where we
    # think the centre is); the coarse face centre only decides which end is the tip
    px_, py_ = xx.astype(float), -yy.astype(float)          # math y up
    mx, my = px_.mean(), py_.mean()
    cov = np.cov(np.stack([px_ - mx, py_ - my]))
    evals, evecs = np.linalg.eigh(cov)
    u = evecs[:, 1]                                          # major axis
    proj = (px_ - mx) * u[0] + (py_ - my) * u[1]
    end_hi = np.array([mx, my]) + proj.max() * u
    end_lo = np.array([mx, my]) + proj.min() * u
    c = np.array([cx, -cy])
    if np.hypot(*(end_lo - c)) > np.hypot(*(end_hi - c)):
        u = -u
    ang = math.degrees(math.atan2(u[1], u[0]))
    length = float(proj.max() - proj.min())
    thin = float(math.sqrt(max(evals[0], 1e-9) / max(evals[1], 1e-9)))   # 0 = a perfect line
    spread = math.degrees(math.atan(thin))
    conf = float(max(0.0, min(1.0, 1.0 - thin / 0.35))
                 * min(1.0, length / (0.5 * rad))
                 * min(1.0, n_face / 1500.0))
    frac = angle_to_value(ang, 0.0, 1.0)
    return frac, conf, f"needle {ang:.1f} deg, spread {spread:.1f} deg, face r={rad:.0f}px"
