#!/usr/bin/env python3
"""Extract a MuJoCo world from a 3D Gaussian splat (2.5D box geometry).

MuJoCo cannot render or collide with Gaussians, so the splat world is split in two:
  * THIS script: physics + LiDAR geometry. Opaque Gaussians -> 2D occupancy grid ->
    greedy-merged boxes extruded floor-to-top. Boxes (not a mesh) because MuJoCo
    collides against a mesh's CONVEX HULL, and the LiDAR's mj_multiRay stays cheap
    on primitives (mujoco_sim.py:36-41). All boxes are geom group 0 (LiDAR-visible).
  * tools/splat/splat_camera_node.py: photoreal camera, rendered with gsplat at the
    sim's ground-truth pose using the transform this script writes.

Frame contract: world = splat + t (pure translation; the splat must already be
z-up and metric). t puts the floor at z=0 (M20.xml ground plane) and the chosen
spawn at the sim origin (mujoco_sim.py spawns at 0,0 and bringup seeds
/initialpose at identity), so no sim code needs a spawn parameter.

Usage (WSL):
  python3 tools/splat/extract_geometry.py ~/m20_sim/splats/gaussians_indoor.ply
  -> tools/sim/<name>.xml, tools/sim/<name>.scene.yaml, docs/media/<name>_occupancy.png
Needs numpy, scipy, pillow.
"""
import argparse, hashlib, math, os, re, sys
import numpy as np
from scipy import ndimage

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SH_C0 = 0.28209479177387814   # degree-0 SH basis constant: rgb = 0.5 + SH_C0 * f_dc


def load_ply(path):
    with open(path, "rb") as f:
        props, n = [], 0
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            if line.startswith("element vertex"):
                n = int(line.split()[-1])
            elif line.startswith("property"):
                if line.split()[1] != "float":
                    sys.exit(f"unsupported PLY property type: {line}")
                props.append(line.split()[-1])
            elif line == "end_header":
                break
            elif line.startswith("format") and "binary_little_endian" not in line:
                sys.exit(f"unsupported PLY format: {line}")
        data = np.fromfile(f, dtype=np.float32, count=n * len(props)).reshape(n, len(props))
    col = {k: i for i, k in enumerate(props)}
    for k in ("x", "y", "z", "opacity", "scale_0", "f_dc_0"):
        if k not in col:
            sys.exit(f"not a 3DGS PLY (missing '{k}')")
    xyz = data[:, [col["x"], col["y"], col["z"]]].astype(np.float64)
    opacity = 1.0 / (1.0 + np.exp(-data[:, col["opacity"]]))
    scale = np.exp(data[:, [col["scale_0"], col["scale_1"], col["scale_2"]]]).max(1)
    rgb = np.clip(0.5 + SH_C0 * data[:, [col["f_dc_0"], col["f_dc_1"], col["f_dc_2"]]], 0, 1)
    return xyz, opacity, scale, rgb


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 22), b""):
            h.update(chunk)
    return h.hexdigest()


def greedy_rects(occ, hbin):
    """Merge occupied cells with equal height bin into maximal-ish rectangles.
    Returns [(r0, c0, rows, cols, hbin)]."""
    used = np.zeros_like(occ)
    rects = []
    R, C = occ.shape
    for r in range(R):
        for c in range(C):
            if not occ[r, c] or used[r, c]:
                continue
            h = hbin[r, c]
            w = 1
            while c + w < C and occ[r, c + w] and not used[r, c + w] and hbin[r, c + w] == h:
                w += 1
            d = 1
            while r + d < R:
                row = slice(c, c + w)
                if (occ[r + d, row].all() and not used[r + d, row].any()
                        and (hbin[r + d, row] == h).all()):
                    d += 1
                else:
                    break
            used[r:r + d, c:c + w] = True
            rects.append((r, c, d, w, h))
    return rects


def geodesic(free, start):
    """4-connected BFS distance (cells) over `free` from start; -1 = unreachable."""
    from collections import deque
    dist = np.full(free.shape, -1, np.int32)
    dist[start] = 0
    q = deque([start])
    R, C = free.shape
    while q:
        r, c = q.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            rr, cc = r + dr, c + dc
            if 0 <= rr < R and 0 <= cc < C and free[rr, cc] and dist[rr, cc] < 0:
                dist[rr, cc] = dist[r, c] + 1
                q.append((rr, cc))
    return dist


def plan_tour(occ, floor, spawn, cell, clearance, spacing):
    """Exploration tour: one waypoint per spacing x spacing tile of reachable floor (the
    most open cell in the tile, >= clearance from any box), ordered greedily by WALKING
    distance from spawn, ending back at spawn. Returns [(r, c), ...] incl. final spawn."""
    clear = ndimage.distance_transform_edt(~occ) * cell
    free = floor & (clear >= clearance)
    free[spawn] = True
    reach = geodesic(free, spawn) >= 0
    tile = max(1, int(round(spacing / cell)))
    pts = []
    for r0 in range(0, occ.shape[0], tile):
        for c0 in range(0, occ.shape[1], tile):
            blk = np.where(reach[r0:r0 + tile, c0:c0 + tile], clear[r0:r0 + tile, c0:c0 + tile], -1)
            if blk.max() > 0:
                r, c = np.unravel_index(np.argmax(blk), blk.shape)
                pts.append((r0 + r, c0 + c))
    # tile-edge neighbours land within cm of each other: keep the most open of any pair
    # closer than spacing/2 (and never one on top of spawn)
    pts.sort(key=lambda p: -clear[p])
    kept, min_d = [], (spacing / 2) / cell
    for p in pts:
        if all(math.dist(p, k) >= min_d for k in kept + [spawn]):
            kept.append(p)
    # closed loop spawn -> stops -> spawn: walking-distance matrix, nearest-neighbour seed,
    # then 2-opt (reverse any segment that shortens the loop) until no move helps
    nodes = [spawn] + kept
    fields = [geodesic(free, p) for p in nodes]
    D = np.array([[f[q] if f[q] >= 0 else 10 ** 6 for q in nodes] for f in fields], float)
    route, left = [0], set(range(1, len(nodes)))
    while left:
        nxt = min(left, key=lambda j: D[route[-1], j])
        route.append(nxt); left.discard(nxt)
    route.append(0)
    improved = True
    while improved:
        improved = False
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route) - 1):
                a_, b_, c_, d_ = route[i - 1], route[i], route[j], route[j + 1]
                if D[a_, c_] + D[b_, d_] < D[a_, b_] + D[c_, d_] - 1e-9:
                    route[i:j + 1] = route[i:j + 1][::-1]
                    improved = True
    # Via-points: Nav2 takes the SHORTEST path between consecutive stops, which can thread
    # a squeeze the wide (>= clearance) route avoids — measured 2026-10-05: the stop9->home
    # leg cut through a ~0.5 m gap and grazed boxes every lap. Where the straight segment
    # leaves the wide corridor, follow the wide geodesic and add string-pulled via-points.
    out = []                                   # (cell, is_via)
    for a_i, b_i in zip(route[:-1], route[1:]):
        a, b = nodes[a_i], nodes[b_i]
        if not line_of_sight(free, a, b):
            path = descend(fields[b_i], a)
            k = 0
            while k < len(path) - 1:          # string pulling: farthest visible path cell
                j = len(path) - 1
                while j > k + 1 and not line_of_sight(free, path[k], path[j]):
                    j -= 1
                if j < len(path) - 1:
                    out.append((path[j], True))
                k = j
        out.append((b, False))
    # drop via-points within 1 m of a neighbour: such short hops don't change Nav2's route,
    # they just add stop-and-turn jitter
    min_hop = 1.0 / cell
    keep = []
    for i, (p, via) in enumerate(out):
        prev = keep[-1][0] if keep else spawn
        nxt = out[i + 1][0] if i + 1 < len(out) else None
        if via and (math.dist(p, prev) < min_hop or (nxt is not None and math.dist(p, nxt) < min_hop)):
            continue
        keep.append((p, via))
    return [p for p, _ in keep]


def line_of_sight(free, a, b):
    """True if the straight segment a->b (grid cells) stays inside `free`."""
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
    rr = np.rint(np.linspace(a[0], b[0], n)).astype(int)
    cc = np.rint(np.linspace(a[1], b[1], n)).astype(int)
    return bool(free[rr, cc].all())


def descend(field, start):
    """Walk downhill on a BFS distance field from start to its source (0). Cell path."""
    path, cur = [start], start
    R, C = field.shape
    while field[cur] > 0:
        r, c = cur
        nbrs = [(r + dr, c + dc) for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))
                if 0 <= r + dr < R and 0 <= c + dc < C and field[r + dr, c + dc] >= 0]
        cur = min(nbrs, key=lambda p: field[p])
        path.append(cur)
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ply")
    ap.add_argument("--name", default="indoor_splat", help="scene name (output file stem)")
    ap.add_argument("--cell", type=float, default=0.10, help="grid cell size [m]")
    ap.add_argument("--min-opacity", type=float, default=0.5)
    ap.add_argument("--max-scale", type=float, default=0.3, help="drop Gaussians larger than this [m]")
    ap.add_argument("--z-min", type=float, default=0.15, help="obstacle band bottom above floor [m] "
                    "(the floor layer is ~0.1 m thick in splats; a quadruped steps lower things)")
    ap.add_argument("--z-max", type=float, default=2.2, help="obstacle band top (ceiling cut) [m]")
    ap.add_argument("--layer", type=float, default=0.10, help="vertical evidence layer height [m]")
    # Splats are floor/ceiling-heavy (walls get few Gaussians), so per-voxel density is too
    # sparse: a cell is solid when its column has enough Gaussians spread over enough heights.
    ap.add_argument("--min-count", type=int, default=4, help="Gaussians per column to mark a cell")
    ap.add_argument("--min-layers", type=int, default=2, help="distinct occupied layers per column")
    ap.add_argument("--min-blob", type=int, default=3, help="drop obstacle blobs smaller than N cells")
    ap.add_argument("--close", type=int, default=2, help="morphological closing radius [cells]")
    ap.add_argument("--boundary-wall", type=float, default=2.0,
                    help="height of the wall around the reconstructed floor [m]; 0 = none")
    ap.add_argument("--height-step", type=float, default=0.4, help="box top quantization [m]")
    ap.add_argument("--clearance", type=float, default=0.8, help="min spawn clearance radius [m]")
    ap.add_argument("--max-boxes", type=int, default=3000)
    ap.add_argument("--tour-spacing", type=float, default=4.5, help="exploration waypoint spacing [m]")
    ap.add_argument("--tour-clearance", type=float, default=0.7,
                    help="min waypoint distance to any box [m] (M20 half-length + inflation margin)")
    ap.add_argument("--spawn", help="override spawn 'x,y' in SPLAT coordinates")
    a = ap.parse_args()

    print(f"[extract] loading {a.ply}")
    xyz, op, sc, rgb = load_ply(a.ply)
    n0 = len(xyz)
    keep = (op > a.min_opacity) & (sc < a.max_scale)
    xyz, rgb = xyz[keep], rgb[keep]
    lo, hi = np.quantile(xyz, 0.01, axis=0) - 0.5, np.quantile(xyz, 0.99, axis=0) + 0.5
    inb = np.all((xyz >= lo) & (xyz <= hi), axis=1)
    xyz, rgb = xyz[inb], rgb[inb]
    print(f"[extract] {n0} Gaussians -> {len(xyz)} opaque, in-bounds")

    # Floor = sharpest z peak (the flat floor carries a large share of the Gaussians).
    zs = xyz[:, 2]
    hist, edges = np.histogram(zs, bins=np.arange(lo[2], min(hi[2], lo[2] + 3.0), 0.01))
    floor_z = float(edges[hist.argmax()] + 0.005)
    print(f"[extract] floor at z={floor_z:+.3f} in splat frame ({hist.max() / len(zs):.1%} of Gaussians in its 1 cm bin)")
    z = zs - floor_z

    # Column evidence: Gaussian count + distinct height layers per 2D cell.
    cell = a.cell
    x0, y0 = lo[0], lo[1]
    ix = ((xyz[:, 0] - x0) / cell).astype(int)
    iy = ((xyz[:, 1] - y0) / cell).astype(int)
    C, R = ix.max() + 1, iy.max() + 1
    band = (z >= a.z_min) & (z <= a.z_max)
    iz = ((z[band] - a.z_min) / a.layer).astype(int)
    L = iz.max() + 1
    vox = np.zeros((R, C, L), np.int32)
    np.add.at(vox, (iy[band], ix[band], iz), 1)
    hit = vox > 0
    occ = (vox.sum(2) >= a.min_count) & (hit.sum(2) >= a.min_layers)
    occ = ndimage.binary_closing(occ, iterations=a.close)    # seal gaps in walls (a.close cells)
    lab, nl = ndimage.label(occ)
    sizes = ndimage.sum(occ, lab, range(1, nl + 1))
    occ = np.isin(lab, 1 + np.flatnonzero(sizes >= a.min_blob))
    top_idx = np.where(hit.any(2), L - 1 - np.argmax(hit[:, :, ::-1], axis=2), 0)
    top = a.z_min + (top_idx + 1) * a.layer
    top = np.where(occ & ~hit.any(2), ndimage.maximum_filter(top, 3), top)  # closing-filled cells
    hbin = np.clip(np.ceil(top / a.height_step), 1, None).astype(int)
    hbin[~occ] = 0

    # Floor mask (where the room actually has floor) -> interior free space -> spawn.
    fl = (z > -0.05) & (z < a.z_min)
    floor = np.zeros((R, C), bool)
    floor[iy[fl], ix[fl]] = True
    floor = ndimage.binary_closing(floor, iterations=3) & ~occ
    floor = ndimage.binary_fill_holes(floor | occ) & ~occ
    # Boundary wall one cell outside the reconstructed floor: beyond it the splat has no
    # ground, so keep the robot (and the LiDAR/SLAM) inside a closed room.
    wall = np.zeros_like(occ)
    if a.boundary_wall > 0:
        # smooth the outline first: a jagged ring merges into hundreds of 1-cell boxes
        room = ndimage.binary_opening(ndimage.binary_closing(floor | occ, iterations=6), iterations=4)
        room = ndimage.binary_fill_holes(room | occ)
        floor &= room
        wall = ndimage.binary_dilation(room) & ~room
        occ = occ | wall
        hbin[wall] = int(np.ceil(a.boundary_wall / a.height_step))
    if a.spawn:
        sx, sy = (float(v) for v in a.spawn.split(","))
        sr, sc_ = int((sy - y0) / cell), int((sx - x0) / cell)
        clear = ndimage.distance_transform_edt(~occ)[sr, sc_] * cell
    else:
        dist = ndimage.distance_transform_edt(floor) * cell
        sr, sc_ = np.unravel_index(np.argmax(dist), dist.shape)
        clear = dist[sr, sc_]
        sx, sy = x0 + (sc_ + 0.5) * cell, y0 + (sr + 0.5) * cell
    if clear < a.clearance:
        sys.exit(f"[extract] spawn ({sx:.2f},{sy:.2f}) clearance {clear:.2f} m < {a.clearance} m — pass --spawn")
    t = np.array([-sx, -sy, -floor_z])
    print(f"[extract] spawn at splat ({sx:.2f},{sy:.2f}), clearance {clear:.2f} m -> world origin")

    tour_cells = plan_tour(occ, floor, (sr, sc_), cell, a.tour_clearance, a.tour_spacing)
    tour = [(x0 + (c + 0.5) * cell + t[0], y0 + (r + 0.5) * cell + t[1]) for r, c in tour_cells]
    print(f"[extract] exploration tour: {len(tour) - 1} waypoints (spacing {a.tour_spacing} m, "
          f"clearance >= {a.tour_clearance} m), ends at spawn")

    rects = greedy_rects(occ, hbin)
    print(f"[extract] {int(occ.sum())} occupied cells -> {len(rects)} boxes")
    if len(rects) > a.max_boxes:
        sys.exit(f"[extract] {len(rects)} boxes > --max-boxes {a.max_boxes}: raise --cell or --height-step")

    # Per-cell mean colour (all band Gaussians) -> per-box colour.
    csum = np.zeros((R, C, 3)); ccnt = np.zeros((R, C))
    np.add.at(csum, (iy[band], ix[band]), rgb[band]); np.add.at(ccnt, (iy[band], ix[band]), 1)

    geoms = []
    for k, (r, c, d, w, h) in enumerate(rects):
        height = h * a.height_step
        cx = x0 + (c + w / 2) * cell + t[0]
        cy = y0 + (r + d / 2) * cell + t[1]
        n = ccnt[r:r + d, c:c + w].sum()
        col = csum[r:r + d, c:c + w].sum((0, 1)) / n if n else np.array([0.6, 0.6, 0.6])
        geoms.append(f'    <geom name="splat_{k}" type="box" pos="{cx:.3f} {cy:.3f} {height / 2:.3f}" '
                     f'size="{w * cell / 2:.3f} {d * cell / 2:.3f} {height / 2:.3f}" '
                     f'rgba="{col[0]:.2f} {col[1]:.2f} {col[2]:.2f} 1"/>')

    out_xml = os.path.join(REPO, "tools", "sim", f"{a.name}.xml")
    src = os.path.basename(a.ply)
    with open(out_xml, "w", newline="\n") as f:
        f.write(f'<mujoco model="m20_{a.name}">\n'
                f'  <!-- GENERATED by tools/splat/extract_geometry.py from {src} — do not hand-edit.\n'
                f'       2.5D box geometry of the splat world (group 0 = LiDAR-visible), world = splat + t,\n'
                f'       see {a.name}.scene.yaml. Sits next to M20.xml (setup_sim.sh installs it). -->\n'
                f'  <include file="M20.xml"/>\n  <worldbody>\n' + "\n".join(geoms) + "\n  </worldbody>\n</mujoco>\n")

    bmin = np.array([x0, y0]) + t[:2]
    bmax = np.array([x0 + C * cell, y0 + R * cell]) + t[:2]
    out_yaml = os.path.join(REPO, "tools", "sim", f"{a.name}.scene.yaml")
    # Keep the photoreal top-down map block (tools/splat/render_topdown.py) across re-extraction —
    # but only if the frame is unchanged: its x0/y1 are WORLD coordinates of the map pixels.
    keep_topdown = ""
    if os.path.exists(out_yaml):
        old = open(out_yaml).read()
        m_td = re.search(r"^topdown:\n(?:[ \t].*\n?)*", old, re.M)
        m_tr = re.search(r"^splat_to_world:\n((?:  - .*\n){4})", old, re.M)
        new_tr = (f"  - [1.0, 0.0, 0.0, {t[0]:.4f}]\n  - [0.0, 1.0, 0.0, {t[1]:.4f}]\n"
                  f"  - [0.0, 0.0, 1.0, {t[2]:.4f}]\n  - [0.0, 0.0, 0.0, 1.0]\n")
        if m_td and m_tr and m_tr.group(1) == new_tr:
            keep_topdown = m_td.group(0).rstrip("\n") + "\n"
        elif m_td:
            print("[extract] WARNING: frame changed — dropped the topdown: map block; re-run render_topdown.py")
    with open(out_yaml, "w", newline="\n") as f:
        f.write(f"# GENERATED by tools/splat/extract_geometry.py — do not hand-edit.\n"
                f"# world = splat_to_world @ splat (row-major 4x4). Consumed by splat_camera_node.py.\n"
                f"name: {a.name}\n"
                f"mjcf: {a.name}.xml\n"
                f"splat:\n  file: {src}\n  sha256: {sha256(a.ply)}\n  gaussians: {n0}\n"
                f"splat_to_world:\n"
                f"  - [1.0, 0.0, 0.0, {t[0]:.4f}]\n  - [0.0, 1.0, 0.0, {t[1]:.4f}]\n"
                f"  - [0.0, 0.0, 1.0, {t[2]:.4f}]\n  - [0.0, 0.0, 0.0, 1.0]\n"
                f"floor_z_splat: {floor_z:.4f}\n"
                f"spawn_splat: [{sx:.3f}, {sy:.3f}]\n"
                f"spawn_clearance_m: {clear:.2f}\n"
                f"world_bounds_xy: [[{bmin[0]:.2f}, {bmin[1]:.2f}], [{bmax[0]:.2f}, {bmax[1]:.2f}]]\n"
                f"boxes: {len(rects)}\n"
                f"# exploration tour (world/map frame, metres), visited in order; last = spawn\n"
                f"tour: [{', '.join(f'[{x:.2f}, {y:.2f}]' for x, y in tour)}]\n"
                f"params: {{cell: {cell}, min_opacity: {a.min_opacity}, max_scale: {a.max_scale}, "
                f"z_min: {a.z_min}, z_max: {a.z_max}, layer: {a.layer}, min_count: {a.min_count}, min_layers: {a.min_layers}, "
                f"min_blob: {a.min_blob}, close: {a.close}, boundary_wall: {a.boundary_wall}, height_step: {a.height_step}}}\n"
                + keep_topdown)

    # Check image: left = top-down splat colour, right = extracted boxes (+ floor, spawn, 1 m grid).
    from PIL import Image, ImageDraw
    topc = np.where(ccnt[..., None] > 0, csum / np.maximum(ccnt, 1)[..., None], 0.1)
    left = (topc * 255).astype(np.uint8)
    right = np.full((R, C, 3), 25, np.uint8)
    right[floor] = (70, 70, 70)
    shade = np.clip(hbin * a.height_step / a.z_max, 0.3, 1.0)
    right[occ] = (np.stack([shade, 0.55 * shade, 0.2 * shade], -1)[occ] * 255).astype(np.uint8)
    right[wall] = (150, 150, 200)
    img = Image.fromarray(np.concatenate([left, np.full((R, 4, 3), 255, np.uint8), right], 1)[::-1])
    scale = max(1, int(round(900 / R)))
    img = img.resize((img.width * scale, img.height * scale), Image.NEAREST)
    dr = ImageDraw.Draw(img)
    px = lambda x, y, off=0: ((off + (x - x0) / cell) * scale, (R - (y - y0) / cell) * scale)
    for gx in np.arange(np.ceil(x0), x0 + C * cell):
        for off in (0, C + 4):
            dr.line([px(gx, y0, off), px(gx, y0 + R * cell, off)], fill=(90, 90, 140))
    for gy in np.arange(np.ceil(y0), y0 + R * cell):
        for off in (0, C + 4):
            dr.line([px(x0, gy, off), px(x0 + C * cell, gy, off)], fill=(90, 90, 140))
    for off in (0, C + 4):
        cx_, cy_ = px(sx, sy, off)
        dr.ellipse([cx_ - 8, cy_ - 8, cx_ + 8, cy_ + 8], outline=(0, 255, 0), width=3)
        dr.line([(cx_, cy_), (cx_ + 20, cy_)], fill=(0, 255, 0), width=3)   # +x = spawn yaw 0
    # exploration tour (right panel): cyan path in visiting order, numbered stops
    tour_px = [px(x - t[0], y - t[1], C + 4) for x, y in [(sx + t[0], sy + t[1])] + tour]
    dr.line(tour_px, fill=(0, 220, 255), width=2)
    for k, (ux, uy) in enumerate(tour_px[1:-1], 1):
        dr.ellipse([ux - 4, uy - 4, ux + 4, uy + 4], fill=(0, 220, 255))
        dr.text((ux + 5, uy - 12), str(k), fill=(255, 255, 255))
    out_png = os.path.join(REPO, "docs", "media", f"{a.name}_occupancy.png")
    img.save(out_png)
    print(f"[extract] wrote {os.path.relpath(out_xml, REPO)}, {os.path.relpath(out_yaml, REPO)}, "
          f"{os.path.relpath(out_png, REPO)}")


if __name__ == "__main__":
    main()
