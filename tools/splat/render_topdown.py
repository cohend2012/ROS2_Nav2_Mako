#!/usr/bin/env python3
"""Render a PHOTOREAL top-down map of a splat world (gsplat, orthographic camera).

The ceiling hides the room from above, so Gaussians above a cut height are dropped first.
The camera looks straight down (north = world +y is image up) with an exact scale of
`px_per_m`, so map pixels <-> world (= sim = Nav2 map) metres is a fixed affine map that
the Viser mini-map (tools/splat/splat_viewer.py) uses to draw the robot, trail and tour.

Run inside m20_splat (GPU):
  tuning:  python3 render_topdown.py --scene /scene/indoor_splat.scene.yaml --cuts 1.2,1.6,2.0,2.4
           -> /out/<name>_topdown_cuts.png (labelled contact sheet; pick a cut)
  final:   python3 render_topdown.py --scene /scene/indoor_splat.scene.yaml --cut 2.0 --write
           -> <scene dir>/<name>.topdown.png + `topdown:` block in the scene yaml(s)
Env/args: SPLAT_PLY (default /splat/<splat.file from yaml>), --px-per-m (50 = 2 cm/px).
"""
import argparse, math, os, re, sys
import numpy as np
import torch
import yaml
import cv2
from gsplat import rasterization

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from splat_camera_node import load_splat   # one splat loader (world frame via splat_to_world)

BG = (24, 24, 24)


def render(g, deg, cut, bounds, ppm, dev, margin=0.5, max_scale=None, min_opacity=None):
    """Orthographic straight-down render of Gaussians with z <= cut inside bounds.
    max_scale / min_opacity drop oversized and faint Gaussians: the capture was ground-level,
    so object TOPS are poorly constrained and render from above as smears and streaks.
    Returns (rgb uint8 HxWx3, meta dict)."""
    (x0, y0), (x1, y1) = bounds
    x0, y0, x1, y1 = x0 - margin, y0 - margin, x1 + margin, y1 + margin
    m = g["means"]
    keep = (m[:, 2] <= cut) & (m[:, 2] >= -0.3) & (m[:, 0] >= x0) & (m[:, 0] <= x1) \
        & (m[:, 1] >= y0) & (m[:, 1] <= y1)
    if max_scale is not None:
        keep &= g["scales"].max(dim=1).values <= max_scale
    if min_opacity is not None:
        keep &= g["opacities"] >= min_opacity
    sub = {k: v[keep] for k, v in g.items()}
    W, H = int(math.ceil((x1 - x0) * ppm)), int(math.ceil((y1 - y0) * ppm))
    cxw, cyw, zc = (x0 + x1) / 2, (y0 + y1) / 2, 20.0
    # camera->world: x_c = +x, y_c = -y (image down = south), z_c = -z (looking down)
    R_wc = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
    view = np.eye(4)
    view[:3, :3] = R_wc.T
    view[:3, 3] = -R_wc.T @ np.array([cxw, cyw, zc])
    K = np.array([[ppm, 0, W / 2], [0, ppm, H / 2], [0, 0, 1]], np.float32)
    with torch.no_grad():
        out, alpha, _ = rasterization(
            sub["means"], sub["quats"], sub["scales"], sub["opacities"], sub["colors"],
            torch.tensor(view, dtype=torch.float32, device=dev)[None],
            torch.tensor(K, device=dev)[None], W, H, sh_degree=deg,
            near_plane=0.01, far_plane=100.0, camera_model="ortho")
    rgb = out[0].clamp(0, 1).cpu().numpy()
    a = alpha[0].clamp(0, 1).cpu().numpy()
    img = rgb * a + (1 - a) * (np.array(BG) / 255.0)     # composite over a dark background
    img = (img * 255).astype(np.uint8)
    meta = {"px_per_m": float(ppm), "x0": float(x0), "y1": float(y1),   # world (x, y) of pixel (0, 0)
            "width": W, "height": H, "cut_z": float(cut), "gaussians": int(keep.sum().item())}
    return img, meta


def label(img, text):
    out = img.copy()
    cv2.rectangle(out, (0, 0), (out.shape[1], 46), (0, 0, 0), -1)
    cv2.putText(out, text, (12, 33), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
    return out


def write_topdown_block(yaml_path, block):
    """Replace/append the `topdown:` block in a scene yaml, keeping everything else verbatim."""
    txt = open(yaml_path).read()
    txt = re.sub(r"^topdown:\n(?:[ \t].*\n?)*", "", txt, flags=re.M).rstrip("\n") + "\n"
    txt += block
    with open(yaml_path, "w", newline="\n") as f:
        f.write(txt)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", required=True, help="scene yaml (has splat_to_world, world_bounds_xy)")
    ap.add_argument("--cuts", help="tuning: comma-separated cut heights [m] -> contact sheet")
    ap.add_argument("--cut", type=float, help="final: the chosen cut height [m]")
    ap.add_argument("--write", action="store_true", help="final: write map PNG + topdown: block(s)")
    ap.add_argument("--px-per-m", type=float, default=50.0)
    ap.add_argument("--max-scale", type=float, help="drop Gaussians larger than this [m] (de-smear)")
    ap.add_argument("--min-opacity", type=float, help="drop Gaussians fainter than this (de-streak)")
    ap.add_argument("--out", default="/out", help="where the contact sheet goes")
    a = ap.parse_args()

    scene = yaml.safe_load(open(a.scene))
    ply = os.environ.get("SPLAT_PLY", os.path.join("/splat", scene["splat"]["file"]))
    dev = torch.device("cuda")
    g, deg = load_splat(ply, scene["splat_to_world"], dev)
    bounds = scene["world_bounds_xy"]
    name = scene["name"]
    print(f"[topdown] {len(g['means'])} Gaussians, bounds {bounds}, {a.px_per_m:.0f} px/m", flush=True)

    if a.cuts:
        tiles = []
        for c in [float(v) for v in a.cuts.split(",")]:
            img, meta = render(g, deg, c, bounds, a.px_per_m, dev, max_scale=a.max_scale, min_opacity=a.min_opacity)
            tag = "" if a.max_scale is None and a.min_opacity is None else f"  clean(s<={a.max_scale}, o>={a.min_opacity})"
            tiles.append(label(img, f"cut {c:.1f} m{tag}"))
            print(f"[topdown] cut {c:.1f} m: {meta['gaussians']} Gaussians, {meta['width']}x{meta['height']}", flush=True)
        sep = np.full((tiles[0].shape[0], 8, 3), 255, np.uint8)
        sheet = np.concatenate(sum([[t, sep] for t in tiles], [])[:-1], axis=1)
        suffix = "" if a.max_scale is None and a.min_opacity is None else "_clean"
        path = os.path.join(a.out, f"{name}_topdown_cuts{suffix}.png")
        cv2.imwrite(path, cv2.cvtColor(sheet, cv2.COLOR_RGB2BGR))
        print(f"[topdown] wrote {path}", flush=True)

    if a.cut is not None:
        img, meta = render(g, deg, a.cut, bounds, a.px_per_m, dev, max_scale=a.max_scale, min_opacity=a.min_opacity)
        out_png = os.path.join(os.path.dirname(a.scene), f"{name}.topdown.png")
        target = out_png if a.write else os.path.join(a.out, f"{name}.topdown.png")
        cv2.imwrite(target, cv2.cvtColor(img, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_PNG_COMPRESSION, 9])
        print(f"[topdown] wrote {target} ({os.path.getsize(target) / 1e6:.1f} MB, {meta['width']}x{meta['height']})", flush=True)
        if a.write:
            block = ("topdown:\n"
                     "  # photoreal top-down map (tools/splat/render_topdown.py); world(x, y) -> pixel:\n"
                     "  #   u = (x - x0) * px_per_m,  v = (y1 - y) * px_per_m   (north = image up)\n"
                     f"  file: {os.path.basename(out_png)}\n"
                     f"  cut_z: {meta['cut_z']}\n  px_per_m: {meta['px_per_m']}\n"
                     f"  x0: {meta['x0']:.4f}\n  y1: {meta['y1']:.4f}\n"
                     f"  width: {meta['width']}\n  height: {meta['height']}\n")
            sdir = os.path.dirname(a.scene)
            for y in sorted(os.listdir(sdir)):
                # every scene yaml built from the same splat + transform shares the map
                if y.endswith(".scene.yaml"):
                    s2 = yaml.safe_load(open(os.path.join(sdir, y)))
                    if s2.get("splat", {}).get("sha256") == scene["splat"]["sha256"] \
                            and s2.get("splat_to_world") == scene["splat_to_world"]:
                        write_topdown_block(os.path.join(sdir, y), block)
                        print(f"[topdown] topdown: block -> {y}", flush=True)


if __name__ == "__main__":
    main()
