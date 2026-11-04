#!/usr/bin/env python3
"""Generate the gauge-inspection scene: oil_gas_field + 6 textured analog gauges.

    python3 tools/sim/make_gauge_scene.py [--out-dir <dir with M20.xml>]

Writes, next to M20.xml (default: the tools/setup_sim.sh model dir):
  oil_gas_field_gauges.xml      scene (includes oil_gas_field.xml)
  gauges/<name>.png             dial textures (render_dial convention)
and into the repo:
  tools/sim/gauges/truth.json   GROUND-TRUTH needle values, for the G5 gate ONLY.
                                The agent never sees this file; its landmark file
                                (src/m20_agent/config/) carries only scale + units.

Gauges are mounted ON existing equipment faces (tanks, skids, wellhead, rack post),
so the saved site map stays valid. Each plate is a thin box whose local +z is the
face normal (MuJoCo projects 2D textures along local z). Face centre is 0.55 m above
ground, the height of the first-person camera at stance.
"""
import argparse
import json
import math
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "src", "m20_agent"))
from m20_agent.gauge_dial import render_dial  # noqa: E402

DEFAULT_DIR = os.path.expanduser(
    "~/m20_sim/sdk_deploy/src/M20_sdk_deploy/M20_description/m20_mjcf/mjcf")
Z = 0.55
HALF = 0.16          # plate half-width (0.32 m dial)
# name, label, plate (x, y), outward normal yaw, units, min, max, TRUE value
GAUGES = [
    ("gauge_pg101", "PG-101", (-5.0, 2.59), -math.pi / 2, "psi", 0, 100, 72.5),
    ("gauge_pg102", "PG-102", (-1.3, 3.12), -math.pi / 2, "psi", 0, 100, 31.0),
    ("gauge_ti201", "TI-201", (5.59, -1.0), math.pi, "degC", 0, 150, 85.0),
    ("gauge_pg301", "PG-301", (-3.49, -1.0), 0.0, "psi", 0, 200, 140.0),
    ("gauge_pg302", "PG-302", (0.0, 1.09), -math.pi / 2, "bar", 0, 16, 12.0),
    ("gauge_fl401", "FL-401", (5.0, 1.89), -math.pi / 2, "bar", 0, 10, 3.3),
]
STANDOFF = 1.2       # base_link distance from the plate at the approach pose


def plate_quat(n_yaw):
    """Quaternion taking local z -> horizontal outward normal (yaw n_yaw) and
    local y -> world up, so the dial texture stands upright facing the robot."""
    # rotation matrix columns = images of local x, y, z
    nz = (math.cos(n_yaw), math.sin(n_yaw), 0.0)
    ny = (0.0, 0.0, 1.0)
    nx = (ny[1] * nz[2] - ny[2] * nz[1], ny[2] * nz[0] - ny[0] * nz[2],
          ny[0] * nz[1] - ny[1] * nz[0])
    m = [[nx[0], ny[0], nz[0]], [nx[1], ny[1], nz[1]], [nx[2], ny[2], nz[2]]]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return (0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return ((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s)


def approach(g):
    _, _, (px, py), n, *_ = g
    return (round(px + STANDOFF * math.cos(n), 3), round(py + STANDOFF * math.sin(n), 3),
            round(math.atan2(-math.sin(n), -math.cos(n)), 4))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default=DEFAULT_DIR)
    a = ap.parse_args()
    if not os.path.exists(os.path.join(a.out_dir, "M20.xml")):
        sys.exit(f"{a.out_dir}/M20.xml missing — run tools/setup_sim.sh first")
    shutil.copy(os.path.join(HERE, "oil_gas_field.xml"), a.out_dir)
    tex_dir = os.path.join(a.out_dir, "gauges"); os.makedirs(tex_dir, exist_ok=True)
    repo_dir = os.path.join(HERE, "gauges"); os.makedirs(repo_dir, exist_ok=True)
    assets, bodies, truth = [], [], {}
    for g in GAUGES:
        name, label, (px, py), n, units, vmin, vmax, val = g
        # MuJoCo samples 2D textures with image-row 0 at texcoord v=1 -> flip
        # vertically so the dial reads upright (verified by the G5 render check)
        img = render_dial(val, vmin, vmax, units, label, size=256)
        img.save(os.path.join(tex_dir, f"{name}.png")); img.save(os.path.join(repo_dir, f"{name}.png"))
        assets.append(f'    <texture name="tex_{name}" type="2d" file="gauges/{name}.png"/>\n'
                      f'    <material name="mat_{name}" texture="tex_{name}" texrepeat="1 1"/>')
        q = plate_quat(n)
        bodies.append(f'    <geom name="{name}" type="box" size="{HALF} {HALF} 0.01" '
                      f'pos="{px:.3f} {py:.3f} {Z}" quat="{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}" '
                      f'material="mat_{name}"/>')
        truth[name] = {"value": val, "min": vmin, "max": vmax, "units": units,
                       "plate": [px, py, Z], "approach": approach(g)}
    xml = ("<!-- GENERATED by tools/sim/make_gauge_scene.py — do not edit by hand. -->\n"
           '<mujoco model="m20_oil_gas_field_gauges">\n  <include file="oil_gas_field.xml"/>\n'
           "  <asset>\n" + "\n".join(assets) + "\n  </asset>\n  <worldbody>\n"
           + "\n".join(bodies) + "\n  </worldbody>\n</mujoco>\n")
    for d in (a.out_dir, HERE):
        with open(os.path.join(d, "oil_gas_field_gauges.xml"), "w") as f:
            f.write(xml)
    with open(os.path.join(repo_dir, "truth.json"), "w") as f:
        json.dump(truth, f, indent=2)
    print(f"wrote {len(GAUGES)} gauges -> {a.out_dir}/oil_gas_field_gauges.xml")
    for k, v in truth.items():
        print(f"  {k}: approach {v['approach']}")


if __name__ == "__main__":
    main()
