#!/usr/bin/env python3
"""Map quality gate: fit circles to known landmarks in a saved .pgm map and
compare against the TRUE model geometry (tools/sim/oil_gas_field.xml).

The 2026-07-20 finding this guards against: the mapping run's odometry drift
froze into the posegraph region-by-region (tank1 +0.27 m, wellhead −0.61 m) —
locally-precise localization against a warped map reproduces the warp in every
nav run. A map only replaces maps/oil_gas_field.* if every landmark fits
within TOL of truth.

Usage:  python3 tools/map/check_map_landmarks.py [map.pgm map.yaml]
        (defaults to maps/oil_gas_field.pgm/.yaml)
Exit 0 = PASS (all offsets < TOL), 1 = FAIL.
"""
import sys
from pathlib import Path

import numpy as np
import imageio.v2 as iio
import yaml

TOL = 0.30            # m per landmark — Phase A acceptance ties to this
LANDMARKS = [         # name, true x, true y (from oil_gas_field.xml geoms)
    ("tank1",    -5.0,  4.0),
    ("tank2",    -1.0,  4.5),
    ("wellhead",  6.0, -1.0),
]

repo = Path(__file__).resolve().parents[2]
pgm = Path(sys.argv[1]) if len(sys.argv) > 2 else repo / "maps/oil_gas_field.pgm"
meta = Path(sys.argv[2]) if len(sys.argv) > 2 else repo / "maps/oil_gas_field.yaml"

m = yaml.safe_load(open(meta))
res = float(m["resolution"])
ox, oy = float(m["origin"][0]), float(m["origin"][1])
img = iio.imread(pgm)
H = img.shape[0]
occ = np.argwhere(img < 100)                 # occupied cells (row, col)
ys = oy + (H - 1 - occ[:, 0]) * res          # pgm row 0 = top = max y
xs = ox + occ[:, 1] * res


def fit_circle_near(cx, cy, win=2.5):
    sel = (np.abs(xs - cx) < win) & (np.abs(ys - cy) < win)
    px, py = xs[sel], ys[sel]
    if len(px) < 20:
        return None
    A = np.c_[px, py, np.ones(len(px))]      # algebraic circle fit
    b = px ** 2 + py ** 2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    fx, fy = sol[0] / 2, sol[1] / 2
    return fx, fy, len(px)


fail = False
for name, tx, ty in LANDMARKS:
    f = fit_circle_near(tx, ty)
    if f is None:
        print(f"{name}: NOT FOUND in map (<20 occupied cells near truth) — FAIL")
        fail = True
        continue
    fx, fy, n = f
    off = float(np.hypot(fx - tx, fy - ty))
    ok = off < TOL
    fail |= not ok
    print(f"{name}: true ({tx:+.2f},{ty:+.2f})  map ({fx:+.2f},{fy:+.2f})  "
          f"offset {off:.2f} m  n={n}  {'ok' if ok else 'FAIL (>' + str(TOL) + ')'}")

print(f"LANDMARK GATE: {'FAIL' if fail else 'PASS'} (tol {TOL} m)")
sys.exit(1 if fail else 0)
