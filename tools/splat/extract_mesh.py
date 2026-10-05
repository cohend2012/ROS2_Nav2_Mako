#!/usr/bin/env python3
"""Extract a MuJoCo world with REAL 3D collision geometry from a 3D Gaussian splat.

Sibling of extract_geometry.py (2.5D boxes extruded floor-to-top). The box world cannot
represent space under tables, overhangs or sloped surfaces; this pipeline can:

  1. views   (WSL)       free-space camera poses inside the room, picked on the box scene's
                         2D free space (so cameras never start inside furniture): a grid of
                         stations x 12 yaws x {0.3 m level, 0.9 m -15 deg, 1.6 m -35 deg,
                         1.6 m +10 deg}.
  2. render  (GPU, m20_splat image)   gsplat renders expected depth for every view in the
                         WORLD frame (scene yaml splat_to_world); low-alpha pixels and depth
                         discontinuities (Gaussian "flying pixels") are dropped; a dense GPU
                         TSDF (torch, 4 cm voxels) fuses all views, carving free space.
  3. build   (WSL)       TSDF -> solid voxels (tsdf<0, seen by >= 2 views; tiny 3D blobs =
                         floaters dropped) -> per-tile marching cubes (closed shells) ->
                         CoACD convex decomposition per connected piece (near-convex pieces
                         are hulled directly) -> hulls written INLINE as MJCF vertex-only
                         meshes (MuJoCo convex-hulls them, which is also exactly what it
                         collides against). The floor (< --z-cut) is left to M20.xml's plane;
                         hulls touching the cut are extruded to z=0 so the LiDAR cannot see
                         under walls. Ceiling (> --z-top) is dropped. A boundary wall (the
                         box scene's ring around the reconstructed floor, as a simplified polygon
                         of thin rotated boxes) keeps the robot inside the room where the
                         splat's sparse/glass walls leave holes.
  4. check   (WSL)       LiDAR cost (mj_multiRay exactly like mujoco_sim._cast_sector) and
                         spawn/standing physics for box vs mesh scene, tour-waypoint
                         clearance, free-space agreement, comparison images.

Frame contract: identical to the box scene. <name>.scene.yaml is a copy of the box
scene's yaml (same splat_to_world, spawn, tour) with name/mjcf/geometry fields replaced,
so the photoreal camera/viewer stay aligned: M20_SCENE=indoor_splat_mesh.xml is drop-in.
Why inline hulls instead of mesh files: a vertex-only <mesh> is convex-hulled by MuJoCo
at load (= the geometry it collides with anyway), so the scene is ONE self-contained XML
next to M20.xml - no meshdir coupling, nothing extra to install.

Usage (WSL; venv with numpy<2 scipy scikit-image trimesh coacd mujoco pyyaml pillow):
  python3 -m venv --system-site-packages ~/m20_sim/mesh_venv
  ~/m20_sim/mesh_venv/bin/pip install "numpy<2" scikit-image trimesh coacd
  ~/m20_sim/mesh_venv/bin/python tools/splat/extract_mesh.py all   # views+render+build+check
  (stages individually: views | render (inside m20_splat, --gpus all) | build | check)
Work files (views.npz, tsdf.npz, hulls.npz) go to --work (default ~/m20_sim/splat_mesh_work).
"""
import argparse, math, os, re, subprocess, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
SIM = os.path.join(REPO, "tools", "sim")
MEDIA = os.path.join(REPO, "docs", "media")

# M20 LiDAR model, copied from tools/mujoco_sim.py (do not drift: check times the same cast)
LIDAR_ELEVS_DEG = [15, 10, 5, 2, 0, -2, -4, -6, -8, -10, -13, -16, -20, -25, -32, -45]
LIDAR_AZIMS, LIDAR_SECTORS, LIDAR_MAX, LIDAR_OFFSET = 120, 20, 12.0, 0.10
STANCE = np.array([0.0, -0.7, 1.4, 0.0, 0.0, -0.7, 1.4, 0.0,
                   0.0, 0.7, -1.4, 0.0, 0.0, 0.7, -1.4, 0.0])
WHEELS = (3, 7, 11, 15)


def read_yaml(path):
    import yaml
    with open(path) as f:
        return yaml.safe_load(f)


def parse_boxes(xml_path):
    """[(cx, cy, cz, hx, hy, hz, yaw)] of a scene's world boxes (yaw from a z-only quat)."""
    txt = open(xml_path).read()
    out = []
    for m in re.finditer(r'type="box" pos="([^"]+)"(?: quat="([^"]+)")? size="([^"]+)"', txt):
        q = [float(v) for v in m.group(2).split()] if m.group(2) else [1, 0, 0, 0]
        out.append([float(v) for v in m.group(1).split()] + [float(v) for v in m.group(3).split()]
                   + [2 * math.atan2(q[3], q[0])])
    return np.array(out)


class Grid2D:
    def __init__(self, bounds, cell):
        (self.x0, self.y0), (x1, y1) = bounds
        self.cell = cell
        self.C, self.R = int(math.ceil((x1 - self.x0) / cell)), int(math.ceil((y1 - self.y0) / cell))

    def rc(self, x, y):
        return int((y - self.y0) / self.cell), int((x - self.x0) / self.cell)

    def xy(self, r, c):
        return self.x0 + (c + 0.5) * self.cell, self.y0 + (r + 0.5) * self.cell


def box_occ2d(boxes, g, zlo=0.0, zhi=10.0):
    occ = np.zeros((g.R, g.C), bool)
    for cx, cy, cz, hx, hy, hz, _ in boxes:            # axis-aligned (box scene)
        if cz + hz < zlo or cz - hz > zhi:
            continue
        r0, c0 = g.rc(cx - hx + 1e-6, cy - hy + 1e-6)
        r1, c1 = g.rc(cx + hx - 1e-6, cy + hy - 1e-6)
        occ[max(r0, 0):r1 + 1, max(c0, 0):c1 + 1] = True
    return occ


def room_masks(boxes, g):
    """(inside, wall_ring) from the box scene: outside = free space connected to the grid
    border; the box boundary wall is the ring of occupied cells touching it."""
    from scipy import ndimage
    occ = box_occ2d(boxes, g)
    pad = np.pad(~occ, 1, constant_values=True)
    lab, _ = ndimage.label(pad)
    outside = (lab == lab[0, 0])[1:-1, 1:-1]
    ring = occ & ndimage.binary_dilation(outside)
    return ~outside, ring


# --------------------------------------------------------------------------- views
def cmd_views(a):
    from scipy import ndimage
    sc = read_yaml(a.scene)
    boxes = parse_boxes(os.path.join(SIM, sc["mjcf"]))
    g = Grid2D(sc["world_bounds_xy"], 0.1)
    occ = box_occ2d(boxes, g)
    clear = ndimage.distance_transform_edt(~occ) * g.cell
    lab, _ = ndimage.label(~occ)
    reach = lab == lab[g.rc(0.0, 0.0)]
    cand = reach & (clear >= a.view_clearance)
    tile = int(round(a.view_spacing / g.cell))
    stations = []
    for r0 in range(0, g.R, tile):
        for c0 in range(0, g.C, tile):
            blk = np.where(cand[r0:r0 + tile, c0:c0 + tile], clear[r0:r0 + tile, c0:c0 + tile], -1)
            if blk.max() > 0:
                r, c = np.unravel_index(np.argmax(blk), blk.shape)
                stations.append(g.xy(r0 + r, c0 + c))
    rigs = [(0.3, 0.0), (0.9, -15.0), (1.6, -35.0), (1.6, 10.0)]   # (height m, pitch deg)
    c2w = []
    for x, y in stations:
        for h, pitch in rigs:
            for k in range(a.yaws):
                yaw = 2 * math.pi * (k + 0.5 * (h > 1)) / a.yaws          # stagger rigs
                th = math.radians(pitch)
                f = np.array([math.cos(th) * math.cos(yaw), math.cos(th) * math.sin(yaw), math.sin(th)])
                r = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
                M = np.eye(4)
                M[:3, :3] = np.stack([r, np.cross(f, r), f], 1)          # OpenCV cam axes in world
                M[:3, 3] = (x, y, h)
                c2w.append(M)
    W, H = a.width, int(a.width * 3 / 4)
    fx = (W / 2) / math.tan(math.radians(a.hfov) / 2)
    K = np.array([[fx, 0, W / 2], [0, fx, H / 2], [0, 0, 1]])
    os.makedirs(a.work, exist_ok=True)
    np.savez(os.path.join(a.work, "views.npz"), c2w=np.array(c2w), K=K, W=W, H=H,
             stations=np.array(stations))
    print(f"[views] {len(stations)} stations x {len(rigs)} rigs x {a.yaws} yaws = {len(c2w)} views "
          f"({W}x{H}, hfov {a.hfov} deg)")


# --------------------------------------------------------------------------- render + TSDF (GPU)
def cmd_render(a):
    import torch
    from gsplat import rasterization
    sys.path.insert(0, HERE)
    from splat_camera_node import load_splat
    dev = torch.device("cuda")
    sc = read_yaml(a.scene)
    g, deg = load_splat(a.ply, sc["splat_to_world"], dev)
    V = np.load(os.path.join(a.work, "views.npz"))
    c2w, K, W, H = V["c2w"], V["K"], int(V["W"]), int(V["H"])
    (x0, y0), (x1, y1) = sc["world_bounds_xy"]
    vox = a.voxel
    origin = np.array([x0, y0, a.z_min_vol])
    dims = np.ceil((np.array([x1, y1, a.z_max_vol]) - origin) / vox).astype(int)
    print(f"[render] {len(c2w)} views, TSDF {dims.tolist()} = {dims.prod() / 1e6:.1f} M voxels @ {vox} m", flush=True)
    ii = [torch.arange(n, device=dev, dtype=torch.float32) for n in dims]
    X, Y, Z = torch.meshgrid(*[origin[k] + (ii[k] + 0.5) * vox for k in range(3)], indexing="ij")
    P = torch.stack([X.reshape(-1), Y.reshape(-1), Z.reshape(-1)], 1)
    del X, Y, Z
    tsdf = torch.ones(P.shape[0], device=dev)
    wgt = torch.zeros(P.shape[0], device=dev)
    trunc = a.trunc_vox * vox
    Kt = torch.tensor(K, dtype=torch.float32, device=dev)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    pool = torch.nn.functional.max_pool2d
    t0, used, B = time.time(), 0, 8
    previews = {}
    for b0 in range(0, len(c2w), B):
        cw = torch.tensor(c2w[b0:b0 + B], dtype=torch.float32, device=dev)
        view = torch.linalg.inv(cw)
        with torch.no_grad():
            out, alpha, _ = rasterization(g["means"], g["quats"], g["scales"], g["opacities"], g["colors"],
                                          view, Kt[None].expand(len(cw), 3, 3), W, H, sh_degree=deg,
                                          near_plane=0.1, far_plane=100.0, render_mode="RGB+ED")
            D = out[..., 3]
            ok = (alpha[..., 0] > a.alpha_min) & (D > 0.1) & (D < a.depth_max)
            Dmax = pool(torch.where(ok, D, 0)[:, None], 3, 1, 1)[:, 0]
            Dmin = -pool(-torch.where(ok, D, 1e3)[:, None], 3, 1, 1)[:, 0]
            ok &= (Dmax - Dmin) < (a.edge_abs + a.edge_rel * D)            # flying pixels
            D = torch.where(ok, D, 0)
            for k in range(len(cw)):
                Rwc, p = cw[k, :3, :3], cw[k, :3, 3]
                Pc = (P - p) @ Rwc                                          # world -> camera
                z = Pc[:, 2]
                zs = z.clamp(min=1e-3)
                u = torch.round(Pc[:, 0] / zs * fx + cx).long()
                v = torch.round(Pc[:, 1] / zs * fy + cy).long()
                inimg = (z > 0.1) & (u >= 0) & (u < W) & (v >= 0) & (v < H)
                d = torch.zeros_like(z)
                d[inimg] = D[k][v[inimg], u[inimg]]
                sdf = d - z
                upd = inimg & (d > 0) & (sdf > -trunc)
                tv = (sdf[upd] / trunc).clamp(max=1.0)
                w0 = wgt[upd]
                tsdf[upd] = (tsdf[upd] * w0 + tv) / (w0 + 1)
                wgt[upd] = w0 + 1
                used += 1
            if b0 in (0, 2000, 4000):
                previews[b0] = ((out[0, ..., :3].clamp(0, 1) * 255).byte().cpu().numpy(), D[0].cpu().numpy())
        if (b0 // B) % 100 == 0:
            print(f"[render] {b0 + len(cw)}/{len(c2w)} views, {time.time() - t0:.0f} s", flush=True)
    seen = (wgt > 0).float().mean().item()
    print(f"[render] fused {used} views in {time.time() - t0:.0f} s; {seen:.1%} of voxels observed", flush=True)
    np.savez_compressed(os.path.join(a.work, "tsdf.npz"),
                        tsdf=tsdf.reshape(*dims).half().cpu().numpy(),
                        weight=wgt.clamp(max=65535).reshape(*dims).to(torch.int32).cpu().numpy().astype(np.uint16),
                        origin=origin, voxel=vox, trunc=trunc)
    for k, (rgb, d) in previews.items():
        np.savez_compressed(os.path.join(a.work, f"preview_{k}.npz"), rgb=rgb, depth=d)
    print(f"[render] wrote {os.path.join(a.work, 'tsdf.npz')}", flush=True)


# --------------------------------------------------------------------------- build
def _thin_hull(V, max_v):
    """Convex hull vertices of V, reduced to <= max_v by farthest-point sampling."""
    from scipy.spatial import ConvexHull
    try:
        V = V[ConvexHull(V).vertices]
    except Exception:
        return None
    if len(V) > max_v:
        sel = [int(np.argmax(np.linalg.norm(V - V.mean(0), axis=1)))]
        d = np.linalg.norm(V - V[sel[0]], axis=1)
        while len(sel) < max_v:
            sel.append(int(np.argmax(d)))
            d = np.minimum(d, np.linalg.norm(V - V[sel[-1]], axis=1))
        V = V[sel]
    return V


def _fix_hull(V, z_cut, min_thick, max_v):
    """Extrude floor-touching hulls to z=0, give degenerate hulls a minimum thickness, cap
    the vertex count. Returns hull vertices or None."""
    from scipy.spatial import ConvexHull
    low = V[:, 2] < z_cut + 0.03
    if low.any():
        V = np.vstack([V, np.c_[V[low, :2], np.zeros(low.sum())]])
    c = V.mean(0)
    _, s, vt = np.linalg.svd(V - c, full_matrices=False)
    thick = np.ptp((V - c) @ vt[2])
    if thick < min_thick:                       # flat piece: offset both ways along its normal
        n = vt[2] * (min_thick - thick) / 2
        V = np.vstack([V + n, V - n])
    V = _thin_hull(V, max_v)
    if V is None or len(V) < 4:
        return None
    try:
        if ConvexHull(V).volume < 1e-6:
            return None
    except Exception:
        return None
    return V


_CORNERS = np.array([[i, j, k] for i in (-.5, .5) for j in (-.5, .5) for k in (-.5, .5)])


def _bisect_hulls(I, org, vox, p, depth=0):
    """Fidelity fallback: convex hull of a connected voxel cluster I (int (N,3) tile coords);
    while the hull is mostly empty (solid fill < fill_min) split the cluster at the median of
    its longest axis, re-label connectivity, recurse. Guarantees no hull bridges big gaps."""
    from scipy import ndimage
    from scipy.spatial import ConvexHull
    V = (org + (I[:, None, :] + 0.5 + _CORNERS[None]) * vox).reshape(-1, 3)
    try:
        hv = ConvexHull(V).volume
    except Exception:
        return [V]
    ext = np.ptp(I, 0) + 1
    if len(I) * vox ** 3 / hv >= p["fill_min"] or ext.max() * vox < p["min_split"] or depth >= 10:
        return [V]
    ax = int(np.argmax(ext))
    cut = np.median(I[:, ax])
    if cut <= I[:, ax].min():
        cut += 0.5
    out = []
    for half in (I[I[:, ax] < cut], I[I[:, ax] >= cut]):
        if not len(half):
            continue
        lo = half.min(0)
        grid = np.zeros(np.ptp(half, 0) + 1, bool)
        grid[tuple((half - lo).T)] = True
        lab, n = ndimage.label(grid, structure=np.ones((3, 3, 3)))
        for k in range(1, n + 1):
            out += _bisect_hulls(np.argwhere(lab == k) + lo, org, vox, p, depth + 1)
    return out


def _decompose_tile(job):
    """Marching cubes on one padded TSDF tile -> pieces -> convex hulls (world coords).
    Every hull (CoACD or direct) must be >= fill_min full of solid voxels, else the solid
    voxels it covers are re-decomposed by _bisect_hulls (CoACD at a coarse threshold happily
    bridges the empty space between desks with one big hull)."""
    import trimesh, coacd
    from scipy import ndimage
    from scipy.spatial import ConvexHull
    from skimage import measure
    sub, org, vox, p = job
    coacd.set_log_level("error")
    if (sub < 0).sum() == 0:
        return [], 0
    S = np.argwhere(sub < 0)
    Cw = org + (S + 0.5) * vox

    def checked(V):
        try:
            h = ConvexHull(V)
        except Exception:
            return [V]
        lo, hi = V.min(0) - vox, V.max(0) + vox
        m = np.all((Cw >= lo) & (Cw <= hi), 1)
        inside = np.zeros(len(S), bool)
        inside[m] = (Cw[m] @ h.equations[:, :3].T + h.equations[:, 3] <= 0.5 * vox).all(1)
        if inside.sum() * vox ** 3 / h.volume >= p["fill_min"] or np.ptp(V, 0).max() < p["min_split"]:
            return [V]
        I = S[inside]
        if not len(I):
            return []
        lo_i = I.min(0)
        grid = np.zeros(np.ptp(I, 0) + 1, bool)
        grid[tuple((I - lo_i).T)] = True
        lab, n = ndimage.label(grid, structure=np.ones((3, 3, 3)))
        out = []
        for k in range(1, n + 1):
            out += _bisect_hulls(np.argwhere(lab == k) + lo_i, org, vox, p)
        return out
    vol = np.pad(sub, 1, constant_values=1.0)
    verts, faces, _, _ = measure.marching_cubes(vol, level=0.0)
    verts = org + (verts - 1 + 0.5) * vox
    mesh = trimesh.Trimesh(verts, faces, process=True)
    hulls, ncoacd = [], 0
    for comp in mesh.split(only_watertight=False):
        if comp.extents.max() < p["min_piece"]:
            continue
        try:
            ratio = abs(comp.volume) / max(comp.convex_hull.volume, 1e-9)
        except Exception:
            ratio = 0.0
        if ratio > p["convex_ratio"] or comp.extents.max() < p["hull_direct"]:
            parts = [comp.vertices]
        else:
            ncoacd += 1
            try:
                res = coacd.run_coacd(coacd.Mesh(comp.vertices, comp.faces), threshold=p["coacd_threshold"],
                                      max_convex_hull=p["max_hulls_piece"], preprocess_mode="auto",
                                      preprocess_resolution=p["coacd_prep_res"], mcts_iterations=p["coacd_iters"],
                                      max_ch_vertex=p["max_verts"], seed=0)
                parts = [np.asarray(v) for v, _ in res]
            except Exception as e:
                print(f"[build] coacd failed on a piece ({e}); using its hull", flush=True)
                parts = [comp.vertices]
        parts = [W for V in parts for W in checked(np.asarray(V, float))]
        for V in parts:
            V = _fix_hull(np.asarray(V, float), p["z_cut"], p["min_thick"], p["max_verts"])
            if V is not None:
                hulls.append(V)
    return hulls, ncoacd


def cmd_build(a):
    from scipy import ndimage
    from multiprocessing import Pool
    sc = read_yaml(a.scene)
    T = np.load(os.path.join(a.work, "tsdf.npz"))
    tsdf, wgt, origin, vox = T["tsdf"].astype(np.float32), T["weight"], T["origin"], float(T["voxel"])
    nx, ny, nz = tsdf.shape
    zc = origin[2] + (np.arange(nz) + 0.5) * vox
    solid = (tsdf < 0) & (wgt >= a.min_weight)
    solid[:, :, (zc < a.z_cut) | (zc > a.z_top)] = False
    # keep the solids inside the room: the box scene's room mask (dilated a little)
    boxes = parse_boxes(os.path.join(SIM, sc["mjcf"]))
    g = Grid2D(sc["world_bounds_xy"], 0.1)
    inside, ring = room_masks(boxes, g)
    xs = origin[0] + (np.arange(nx) + 0.5) * vox
    ys = origin[1] + (np.arange(ny) + 0.5) * vox
    ri = np.clip(((ys - g.y0) / g.cell).astype(int), 0, g.R - 1)
    ci = np.clip(((xs - g.x0) / g.cell).astype(int), 0, g.C - 1)
    inside_v = ndimage.binary_dilation(inside, iterations=3)[np.ix_(ri, ci)].T   # (nx, ny)
    solid &= inside_v[:, :, None]
    n0 = int(solid.sum())
    if a.close > 0:      # seal pinholes in thin (glass/sparse) walls; drop 1-voxel speckle
        solid = ndimage.binary_closing(solid, iterations=a.close) & inside_v[:, :, None]
        solid[:, :, (zc < a.z_cut) | (zc > a.z_top)] = False
    lab, nl = ndimage.label(solid)
    sizes = np.bincount(lab.ravel())
    keep = sizes >= a.min_blob_vox
    keep[0] = False
    solid = keep[lab]
    print(f"[build] solid voxels {n0} -> {int(solid.sum())} after dropping {int((~keep[1:]).sum())} "
          f"blobs < {a.min_blob_vox} voxels (floaters)")
    vol = np.where(solid, np.minimum(tsdf, -1e-3), np.maximum(tsdf, 1e-3)).astype(np.float32)
    vol[~solid & (wgt < a.min_weight)] = 1.0
    vol[:, :, (zc < a.z_cut) | (zc > a.z_top)] = 1.0
    # A thin solid band whose free neighbours are only partly negative can vanish in MC:
    # clamp solid voxels to <= -0.5 so every kept voxel yields a surface.
    vol[solid] = np.minimum(vol[solid], -0.5)
    vol[~solid] = np.maximum(vol[~solid], 0.5)
    tn = int(round(a.tile / vox))
    p = dict(min_piece=a.min_piece, convex_ratio=a.convex_ratio, hull_direct=a.hull_direct,
             coacd_threshold=a.coacd_threshold, max_hulls_piece=a.max_hulls_piece, coacd_prep_res=a.coacd_prep_res,
             coacd_iters=a.coacd_iters, max_verts=a.max_verts, z_cut=a.z_cut, min_thick=a.min_thick,
             fill_min=a.fill_min, min_split=a.min_split)
    jobs = []
    for i0 in range(0, nx, tn):
        for j0 in range(0, ny, tn):
            sub = vol[i0:i0 + tn + 1, j0:j0 + tn + 1]
            if (sub < 0).any():
                jobs.append((sub.copy(), origin + np.array([i0, j0, 0]) * vox, vox, p))
    t0 = time.time()
    hulls, ncoacd = [], 0
    with Pool(a.jobs) as pool:
        for k, (h, nc) in enumerate(pool.imap_unordered(_decompose_tile, jobs)):
            hulls += h
            ncoacd += nc
            if (k + 1) % 20 == 0:
                print(f"[build] tiles {k + 1}/{len(jobs)}: {len(hulls)} hulls, {time.time() - t0:.0f} s", flush=True)
    print(f"[build] {len(jobs)} tiles -> {len(hulls)} hulls ({ncoacd} pieces via CoACD) in {time.time() - t0:.0f} s")
    if len(hulls) > a.max_total_hulls:
        sys.exit(f"[build] {len(hulls)} hulls > --max-total-hulls {a.max_total_hulls}: raise --coacd-threshold")
    hulls.sort(key=lambda V: (round(V[:, 0].mean(), 1), round(V[:, 1].mean(), 1), V[:, 2].mean()))

    # per-hull colour = mean colour of the opaque Gaussians nearest its vertices
    sys.path.insert(0, HERE)
    from extract_geometry import load_ply
    from scipy.spatial import cKDTree
    xyz, op, scl, rgb = load_ply(a.ply)
    S2W = np.array(sc["splat_to_world"], float)
    m = (op > 0.5) & (scl < 0.3)
    tree = cKDTree(xyz[m] @ S2W[:3, :3].T + S2W[:3, 3])
    rgb = rgb[m]
    cols = [rgb[tree.query(V, k=4)[1].ravel()].mean(0) for V in hulls]

    # Boundary wall = the box scene's ring around the reconstructed floor, but as a simplified
    # polygon of thin rotated boxes (~50) instead of the ring's ~650 staircase cells: every
    # group-0 geom costs LiDAR time, and the hulls already carry the real walls.
    from skimage import measure
    cont = max(measure.find_contours(np.pad(inside, 1).astype(float), 0.5), key=len)
    poly = measure.approximate_polygon(cont, tolerance=a.wall_tolerance / g.cell)
    pts = np.c_[g.x0 + (poly[:, 1] - 1 + 0.5) * g.cell, g.y0 + (poly[:, 0] - 1 + 0.5) * g.cell]
    wall_h, wt = a.boundary_wall, 0.12
    walls = []
    for (xa, ya), (xb, yb) in zip(pts[:-1], pts[1:]):
        L, yaw = math.hypot(xb - xa, yb - ya), math.atan2(yb - ya, xb - xa)
        if L > 1e-3:
            walls.append(((xa + xb) / 2, (ya + yb) / 2, L / 2 + wt / 2, yaw))
    geoms, meshes = [], []
    for k, (V, c) in enumerate(zip(hulls, cols)):
        meshes.append(f'    <mesh name="sm_{k}" inertia="shell" vertex="'
                      + " ".join(f"{v:.3f}" for v in V.ravel()) + '"/>')
        geoms.append(f'    <geom name="splatmesh_{k}" type="mesh" mesh="sm_{k}" '
                     f'rgba="{c[0]:.2f} {c[1]:.2f} {c[2]:.2f} 1"/>')
    for k, (cx, cy, hl, yaw) in enumerate(walls):
        geoms.append(f'    <geom name="splatwall_{k}" type="box" pos="{cx:.3f} {cy:.3f} {wall_h / 2:.3f}" '
                     f'quat="{math.cos(yaw / 2):.6f} 0 0 {math.sin(yaw / 2):.6f}" '
                     f'size="{hl:.3f} {wt / 2:.3f} {wall_h / 2:.3f}" rgba="0.59 0.59 0.78 1"/>')
    wall_rects = walls
    name = a.name
    out_xml = os.path.join(SIM, f"{name}.xml")
    with open(out_xml, "w", newline="\n") as f:
        f.write(f'<mujoco model="m20_{name}">\n'
                f'  <!-- GENERATED by tools/splat/extract_mesh.py from {os.path.basename(a.ply)} - do not hand-edit.\n'
                f'       3D collision geometry of the splat world: {len(hulls)} convex hulls (TSDF of gsplat depth\n'
                f'       renders -> marching cubes -> CoACD) + {len(wall_rects)} boundary-wall boxes, all group 0\n'
                f'       (LiDAR-visible). Vertex-only meshes: MuJoCo convex-hulls them at load. world = splat + t,\n'
                f'       see {name}.scene.yaml. Sits next to M20.xml (setup_sim.sh installs it). -->\n'
                f'  <include file="M20.xml"/>\n  <asset>\n' + "\n".join(meshes) + "\n  </asset>\n"
                f'  <worldbody>\n' + "\n".join(geoms) + "\n  </worldbody>\n</mujoco>\n")
    # scene yaml: the box scene's yaml verbatim (same transform/spawn/tour) with our fields
    src = open(a.scene).read().splitlines()
    out = [f"# GENERATED by tools/splat/extract_mesh.py from {os.path.basename(a.scene)} - do not hand-edit.",
           "# splat_to_world / spawn / tour are copied verbatim from the box scene (same frame)."]
    for line in src:
        if line.startswith("#"):
            continue
        if line.startswith("name:"):
            line = f"name: {name}"
        elif line.startswith("mjcf:"):
            line = f"mjcf: {name}.xml"
        elif line.startswith("boxes:"):
            line = f"hulls: {len(hulls)}\nboundary_wall_boxes: {len(wall_rects)}\nderived_from: {os.path.basename(a.scene)}"
        elif line.startswith("params:"):
            line = (f"params: {{voxel: {vox}, trunc: {float(T['trunc']):.3f}, min_weight: {a.min_weight}, "
                    f"z_cut: {a.z_cut}, z_top: {a.z_top}, min_blob_vox: {a.min_blob_vox}, tile: {a.tile}, "
                    f"coacd_threshold: {a.coacd_threshold}, fill_min: {a.fill_min}, max_verts: {a.max_verts}, boundary_wall: {wall_h}}}")
        elif line.startswith("tour:"):
            out.append("# exploration tour (world/map frame, metres), visited in order; last = spawn")
        out.append(line)
    with open(os.path.join(SIM, f"{name}.scene.yaml"), "w", newline="\n") as f:
        f.write("\n".join(out) + "\n")
    np.savez_compressed(os.path.join(a.work, "hulls.npz"), n=len(hulls),
                        **{f"h{k}": V for k, V in enumerate(hulls)})
    print(f"[build] wrote {os.path.relpath(out_xml, REPO)} ({os.path.getsize(out_xml) / 1e6:.2f} MB), "
          f"tools/sim/{name}.scene.yaml")


# --------------------------------------------------------------------------- check
def _load_model(xml):
    import mujoco
    mjcf_dir = os.path.expanduser("~/m20_sim/sdk_deploy/src/M20_sdk_deploy/M20_description/m20_mjcf/mjcf")
    return mujoco.MjModel.from_xml_path(os.path.join(mjcf_dir, xml))


def _world_geoms(m):
    import mujoco
    ids = [i for i in range(m.ngeom) if m.geom_bodyid[i] == 0
           and mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) != "floor"]
    return np.array(ids)


def _lidar_timing(m, pts, rng):
    import mujoco
    d = mujoco.MjData(m)
    dirs = []
    for k in range(LIDAR_AZIMS):
        az = 2 * math.pi * k / LIDAR_AZIMS
        for ed in LIDAR_ELEVS_DEG:
            el = math.radians(ed)
            dirs.append([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    dirs = np.array(dirs)
    n = len(dirs) // LIDAR_SECTORS
    grp = np.array([1, 0, 0, 0, 0, 0], np.uint8)
    gid, dist = np.full(n, -1, np.int32), np.zeros(n)
    times, hits = [], 0
    for x, y in pts:
        d.qpos[:] = 0
        d.qpos[:3] = (x, y, 0.46)
        yaw = rng.uniform(-math.pi, math.pi)
        d.qpos[3:7] = (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))
        d.qpos[7:23] = STANCE
        mujoco.mj_forward(m, d)
        R = np.empty(9)
        mujoco.mju_quat2Mat(R, d.qpos[3:7])
        R = R.reshape(3, 3)
        origin = (d.qpos[0:3] + R @ np.array([0, 0, LIDAR_OFFSET])).astype(np.float64)
        for s in range(LIDAR_SECTORS):
            vecs = (dirs[s * n:(s + 1) * n] @ R.T).reshape(-1)
            t0 = time.perf_counter()
            mujoco.mj_multiRay(m, d, origin, vecs, grp, True, -1, gid, dist, None, n, LIDAR_MAX)
            times.append((time.perf_counter() - t0) * 1e3)
            hits += int((gid >= 0).sum())
    return times, hits


def _stand(m):
    """Spawn exactly like mujoco_sim.Sim (STANCE, auto-drop, armature) and PD-hold 3 s."""
    import mujoco
    for w in WHEELS:
        m.dof_armature[6 + w] = 0.03
    m.opt.timestep = 1 / 500
    d = mujoco.MjData(m)
    d.qpos[:] = 0
    d.qpos[3] = 1
    d.qpos[7:23] = STANCE
    d.qpos[2] = 1
    mujoco.mj_forward(m, d)
    d.qpos[2] = 1 - float(d.geom_xpos[:, 2].min()) + 0.03
    mujoco.mj_forward(m, d)
    world = set(_world_geoms(m).tolist())
    pen0 = [(d.contact[i].geom1, d.contact[i].geom2, d.contact[i].dist) for i in range(d.ncon)
            if d.contact[i].geom1 in world or d.contact[i].geom2 in world]
    lo, hi = m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1]
    worst = 0.0
    for _ in range(1500):
        q, qd = d.qpos[7:23], d.qvel[6:22]
        d.ctrl[:] = np.clip(200 * (STANCE - q) + 4 * (0 - qd), lo, hi)
        mujoco.mj_step(m, d)
        for i in range(d.ncon):
            c = d.contact[i]
            if c.geom1 in world or c.geom2 in world:
                worst = min(worst, c.dist)
    w, x, y, z = d.qpos[3:7]
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
    return dict(spawn_world_contacts=len(pen0), z=float(d.qpos[2]), roll=math.degrees(roll),
                pitch=math.degrees(pitch), xy_drift=float(np.hypot(*d.qpos[:2])), worst_world_pen=worst)


def _rasterize(hulls, boxes, bounds, z0, z1, res):
    """3D occupancy grid (x, y, z) of hulls (point-in-hull) + boxes at `res` metres."""
    from scipy.spatial import ConvexHull
    (x0, y0), (x1, y1) = bounds
    dims = np.ceil(np.array([x1 - x0, y1 - y0, z1 - z0]) / res).astype(int)
    occ = np.zeros(dims, bool)
    o = np.array([x0, y0, z0])
    for V in hulls:
        eq = ConvexHull(V).equations
        lo = np.clip(np.floor((V.min(0) - o) / res).astype(int), 0, dims - 1)
        hi = np.clip(np.ceil((V.max(0) - o) / res).astype(int), 0, dims - 1)
        ax = [o[k] + (np.arange(lo[k], hi[k] + 1) + 0.5) * res for k in range(3)]
        G = np.stack(np.meshgrid(*ax, indexing="ij"), -1)
        inside = (G @ eq[:, :3].T + eq[:, 3] <= 1e-9).all(-1)
        occ[lo[0]:hi[0] + 1, lo[1]:hi[1] + 1, lo[2]:hi[2] + 1] |= inside
    for cx, cy, cz, hx, hy, hz, yaw in boxes:
        r = math.hypot(hx, hy) if abs(yaw) > 1e-6 else None
        ex = np.array([r or hx, r or hy, hz])
        lo = np.clip(np.floor((np.array([cx, cy, cz]) - ex - o) / res).astype(int), 0, dims - 1)
        hi = np.clip(np.ceil((np.array([cx, cy, cz]) + ex - o) / res).astype(int), 0, dims - 1)
        ax = [o[k] + (np.arange(lo[k], hi[k] + 1) + 0.5) * res for k in range(3)]
        G = np.stack(np.meshgrid(*ax, indexing="ij"), -1) - (cx, cy, cz)
        c, s_ = math.cos(yaw), math.sin(yaw)
        u, v = G[..., 0] * c + G[..., 1] * s_, -G[..., 0] * s_ + G[..., 1] * c
        inside = (abs(u) <= hx) & (abs(v) <= hy) & (abs(G[..., 2]) <= hz)
        occ[lo[0]:hi[0] + 1, lo[1]:hi[1] + 1, lo[2]:hi[2] + 1] |= inside
    return occ


def cmd_check(a):
    import mujoco
    from scipy import ndimage
    from PIL import Image, ImageDraw
    sc = read_yaml(os.path.join(SIM, f"{a.name}.scene.yaml"))
    base = read_yaml(a.scene)
    for k in ("splat_to_world", "spawn_splat", "tour"):
        assert sc[k] == base[k], f"{k} differs from the box scene"
    print("[check] frame contract: splat_to_world, spawn, tour identical to the box scene")
    rng = np.random.default_rng(0)
    mb, mm = _load_model(base["mjcf"]), _load_model(sc["mjcf"])
    print(f"[check] box scene ngeom={mb.ngeom}, mesh scene ngeom={mm.ngeom} nmesh={mm.nmesh} "
          f"(world hull faces total {int(mm.mesh_facenum.sum())})")

    # --- geometry rasters (5 cm) for both scenes
    H = np.load(os.path.join(a.work, "hulls.npz"))
    hulls = [H[f"h{k}"] for k in range(int(H["n"]))]
    boxes_b = parse_boxes(os.path.join(SIM, base["mjcf"]))
    boxes_m = parse_boxes(os.path.join(SIM, sc["mjcf"]))
    res, ztop = 0.05, 2.4
    bounds = base["world_bounds_xy"]
    ob = _rasterize([], boxes_b, bounds, 0, ztop, res)
    om = _rasterize(hulls, boxes_m, bounds, 0, ztop, res)
    zc = (np.arange(ob.shape[2]) + 0.5) * res
    band = (zc >= 0.15) & (zc <= 2.2)
    robot = (zc >= 0.10) & (zc <= 0.65)          # M20 standing body/legs envelope (+margin)
    fb, fm, fmr = ~ob[:, :, band].any(2), ~om[:, :, band].any(2), ~om[:, :, robot].any(2)
    g = Grid2D(bounds, res)
    inside, _ = room_masks(boxes_b, g)
    room = inside.T & fb                         # (x, y): box-scene free floor
    print(f"[check] box-scene free floor {room.sum() * res * res:.1f} m^2: still free in mesh scene "
          f"{(room & fm).sum() / room.sum():.1%} (0.15-2.2 m band), {(room & fmr).sum() / room.sum():.1%} "
          f"(robot band 0.10-0.65 m)")
    occ_b_robot = ob[:, :, robot].any(2) & inside.T
    gained = occ_b_robot & fmr
    print(f"[check] box-scene obstacle area (robot band) that is FREE in the mesh scene: "
          f"{gained.sum() * res * res:.1f} m^2 of {occ_b_robot.sum() * res * res:.1f} m^2 "
          f"(under tables/overhangs + box over-fill)")
    # tour clearance (2D distance to geometry in the robot band and in the full band)
    clr_r = ndimage.distance_transform_edt(fmr) * res
    clr_f = ndimage.distance_transform_edt(fm) * res
    clr_b = ndimage.distance_transform_edt(fb) * res
    ix = lambda x, y: (int((x - bounds[0][0]) / res), int((y - bounds[0][1]) / res))
    bad = []
    print("[check] tour clearance [m] (box | mesh full band | mesh robot band):")
    for k, (x, y) in enumerate(sc["tour"]):
        i, j = ix(x, y)
        flag = "" if clr_r[i, j] >= 0.7 else "  <-- < 0.7 m"
        if flag:
            bad.append(k + 1)
        print(f"    wp{k + 1:2d} ({x:6.2f},{y:6.2f}): {clr_b[i, j]:.2f} | {clr_f[i, j]:.2f} | {clr_r[i, j]:.2f}{flag}")
    print(f"[check] tour waypoints with < 0.7 m robot-band clearance: {bad or 'none'}")

    # --- LiDAR cost (identical cast to mujoco_sim._cast_sector)
    free = np.argwhere(room & fmr & (clr_r >= 0.5))
    near = free[np.hypot(*(free * res + np.array(bounds[0]) - 0).T) < 4.0] if len(free) else free
    sel = near[rng.choice(len(near), size=min(a.lidar_poses, len(near)), replace=False)]
    pts = [(bounds[0][0] + (i + 0.5) * res, bounds[0][1] + (j + 0.5) * res) for i, j in sel]
    # the live stack shares this CPU, so interleave the scenes over several rounds
    acc = {"box ": ([], 0), "mesh": ([], 0)}
    for r in range(a.lidar_rounds):
        for nm, m in (("box ", mb), ("mesh", mm)):
            t_, h_ = _lidar_timing(m, pts, np.random.default_rng(r))
            acc[nm] = (acc[nm][0] + t_, acc[nm][1] + h_)
    for nm, (t_, h_) in acc.items():
        t = np.array(t_)
        print(f"[check] LiDAR {nm} scene: sector (96 rays) median {np.median(t):.3f} ms, p99 {np.percentile(t, 99):.3f} ms, "
              f"max {t.max():.3f} ms (budget 2 ms) over {len(t)} casts; {h_ / len(t):.0f} hits/sector")
    # --- physics
    for nm, m in (("box ", _load_model(base["mjcf"])), ("mesh", _load_model(sc["mjcf"]))):
        r = _stand(m)
        print(f"[check] stand {nm}: spawn contacts with world geoms {r['spawn_world_contacts']}, after 3 s PD hold: "
              f"z {r['z']:.3f} m, roll {r['roll']:+.2f} deg, pitch {r['pitch']:+.2f} deg, drift {r['xy_drift']:.3f} m, "
              f"worst world-geom penetration {r['worst_world_pen'] * 1000:.1f} mm")

    # --- images
    def heat(occ):
        top = np.where(occ.any(2), (occ.shape[2] - np.argmax(occ[:, :, ::-1], 2)) * res, 0)
        v = np.clip(top / 2.2, 0, 1)
        img = np.stack([0.15 + 0.85 * v, 0.15 + 0.6 * v, 0.15 + 0.2 * v], -1)
        img[~occ.any(2)] = (0.12, 0.12, 0.12)
        return img
    under = om[:, :, (zc > 0.65) & (zc <= 2.2)].any(2) & fmr & inside.T   # passable overhang
    ib, im_ = heat(ob[:, :, zc <= 2.2]), heat(om[:, :, zc <= 2.2])
    im_[under] = (0.95, 0.2, 0.85)
    diff = np.full(ib.shape, 0.12)
    diff[room] = (0.3, 0.3, 0.3)
    diff[gained] = (0.2, 0.9, 0.3)
    diff[inside.T & fb & ~fmr] = (0.95, 0.25, 0.2)
    sep = np.ones((6, ib.shape[1], 3))
    W_ = np.concatenate([ib, sep, im_, sep, diff], 0)         # (x, y) panels side by side
    img = Image.fromarray((W_.transpose(1, 0, 2)[::-1] * 255).astype(np.uint8))
    img = img.resize((img.width * 2, img.height * 2), Image.NEAREST)
    dr = ImageDraw.Draw(img)
    nx_, ny = ib.shape[:2]
    for p_ in range(3):
        off = p_ * (nx_ + 6)
        for k, (x, y) in enumerate(sc["tour"]):
            i, j = ix(x, y)
            u, v = 2 * (off + i), 2 * (ny - 1 - j)
            dr.ellipse([u - 5, v - 5, u + 5, v + 5], outline=(0, 220, 255), width=2)
            dr.text((u + 6, v - 12), str(k + 1), fill=(255, 255, 255))
    for t_, x_ in (("box scene", 0), ("mesh scene (magenta: passable overhang)", nx_ + 6),
                   ("robot band 0.10-0.65 m: green freed, red newly blocked", 2 * nx_ + 12)):
        dr.text((2 * x_ + 6, 6), t_, fill=(255, 255, 255))
    out = os.path.join(MEDIA, f"{a.name}_topdown.png")
    img.save(out)
    print(f"[check] wrote {os.path.relpath(out, REPO)}  (left = box, middle = mesh [magenta = passable overhang], "
          f"right = robot-band diff: green freed by mesh, red newly blocked)")
    # Sections through the biggest "table": a blob the box scene blocks in the robot band
    # that the mesh scene leaves free underneath (solid only above 0.65 m).
    lab, n = ndimage.label(gained & under)
    if n == 0:
        print("[check] no box-blocked/mesh-passable overhang found; no section image")
        return
    big = 1 + int(np.argmax(ndimage.sum(np.ones_like(lab), lab, range(1, n + 1))))
    ci, cj = (int(round(v)) for v in ndimage.center_of_mass(lab == big))
    tx, ty = bounds[0][0] + (ci + 0.5) * res, bounds[0][1] + (cj + 0.5) * res
    half, sc_ = int(2.5 / res), 5
    panels = []
    for occ, label in ((ob, "box scene"), (om, "mesh scene")):
        for axis, an in ((0, "x"), (1, "y")):
            sl = occ[:, cj, :] if axis == 0 else occ[ci, :, :]
            c0 = ci if axis == 0 else cj
            lo_, hi_ = max(c0 - half, 0), min(c0 + half, sl.shape[0])
            sec = sl[lo_:hi_].T[::-1].astype(float)                     # z up
            rgbp = np.where(sec[..., None] > 0, (0.93, 0.68, 0.35), (0.13, 0.13, 0.13))
            zr = (zc >= 0.10) & (zc <= 0.65)
            band_rows = np.flatnonzero(zr[::-1])
            rgbp[band_rows[[0, -1]]] = np.where(sec[band_rows[[0, -1]], :, None] > 0, rgbp[band_rows[[0, -1]]],
                                                (0.35, 0.75, 1.0))
            im = Image.fromarray((rgbp * 255).astype(np.uint8)).resize(
                (rgbp.shape[1] * sc_, rgbp.shape[0] * sc_), Image.NEAREST)
            d_ = ImageDraw.Draw(im)
            d_.text((6, 4), f"{label}: {an}-z section through ({tx:.1f}, {ty:.1f}), +-2.5 m, z 0-2.4 m",
                    fill=(255, 255, 255))
            panels.append(im)
    Wp, Hp = panels[0].width + panels[1].width + 8, panels[0].height
    out_img = Image.new("RGB", (Wp, 2 * Hp + 8), (255, 255, 255))
    for k, im in enumerate(panels):
        out_img.paste(im, ((k % 2) * (panels[0].width + 8), (k // 2) * (Hp + 8)))
    out2 = os.path.join(MEDIA, f"{a.name}_section.png")
    out_img.save(out2)
    print(f"[check] wrote {os.path.relpath(out2, REPO)}  (top box / bottom mesh; x-z and y-z sections through the "
          f"largest box-blocked-but-passable overhang at ({tx:.2f}, {ty:.2f}), {(lab == big).sum() * res * res:.2f} m^2; "
          f"cyan lines = robot band 0.10/0.65 m)")


# --------------------------------------------------------------------------- all
def cmd_all(a):
    py = sys.executable
    me = os.path.abspath(__file__)
    common = ["--work", a.work, "--scene", a.scene, "--ply", a.ply]
    subprocess.check_call([py, me, "views"] + common)
    if not os.path.exists(os.path.join(a.work, "tsdf.npz")) or a.rerender:
        cmd = ["docker", "run", "--rm", "--gpus", "all", "--user", f"{os.getuid()}:{os.getgid()}", "-e", "HOME=/tmp",
               "-v", f"{REPO}:/repo:ro", "-v", f"{os.path.dirname(os.path.abspath(a.ply))}:/splat:ro",
               "-v", f"{a.work}:/work", "m20_splat:latest", "python3", "/repo/tools/splat/extract_mesh.py", "render",
               "--work", "/work", "--ply", f"/splat/{os.path.basename(a.ply)}",
               "--scene", "/repo/" + os.path.relpath(a.scene, REPO), "--voxel", str(a.voxel)]
        subprocess.check_call(cmd)
    subprocess.check_call([py, me, "build"] + common + sys.argv[sys.argv.index("all") + 1:])
    subprocess.check_call([py, me, "check"] + common + ["--name", a.name])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage", choices=["views", "render", "build", "check", "all"])
    ap.add_argument("--ply", default=os.path.expanduser("~/m20_sim/splats/gaussians_indoor.ply"))
    ap.add_argument("--scene", default=os.path.join(SIM, "indoor_splat.scene.yaml"),
                    help="box-scene yaml whose frame/spawn/tour/room outline is reused")
    ap.add_argument("--name", default="indoor_splat_mesh")
    ap.add_argument("--work", default=os.path.expanduser("~/m20_sim/splat_mesh_work"))
    ap.add_argument("--rerender", action="store_true", help="(all) re-render even if tsdf.npz exists")
    # views
    ap.add_argument("--view-spacing", type=float, default=1.2, help="camera station spacing [m]")
    ap.add_argument("--view-clearance", type=float, default=0.35, help="min station distance to a box [m]")
    ap.add_argument("--yaws", type=int, default=12)
    ap.add_argument("--width", type=int, default=480)
    ap.add_argument("--hfov", type=float, default=90.0)
    # render / TSDF
    ap.add_argument("--voxel", type=float, default=0.04)
    ap.add_argument("--trunc-vox", type=float, default=3.0, help="TSDF truncation in voxels")
    ap.add_argument("--z-min-vol", type=float, default=-0.12)
    ap.add_argument("--z-max-vol", type=float, default=2.6)
    ap.add_argument("--alpha-min", type=float, default=0.9, help="drop pixels with less accumulated opacity")
    ap.add_argument("--depth-max", type=float, default=6.0)
    ap.add_argument("--edge-abs", type=float, default=0.04, help="flying-pixel filter: 3x3 depth range <")
    ap.add_argument("--edge-rel", type=float, default=0.03, help="... edge_abs + edge_rel * depth")
    # build
    ap.add_argument("--min-weight", type=int, default=2, help="views a voxel needs to count as observed")
    ap.add_argument("--z-cut", type=float, default=0.10, help="below: floor (M20.xml plane) [m]")
    ap.add_argument("--z-top", type=float, default=2.3, help="above: ceiling, dropped [m]")
    ap.add_argument("--min-blob-vox", type=int, default=40, help="drop 3D solid blobs smaller than this")
    ap.add_argument("--close", type=int, default=1, help="3D morphological closing of solids [voxels]")
    ap.add_argument("--tile", type=float, default=3.0, help="decomposition tile size [m]")
    ap.add_argument("--min-piece", type=float, default=0.2, help="drop pieces smaller than this [m]")
    ap.add_argument("--hull-direct", type=float, default=0.3, help="pieces smaller than this are hulled directly")
    ap.add_argument("--convex-ratio", type=float, default=0.75, help="vol/hull-vol above which a piece is one hull")
    ap.add_argument("--coacd-threshold", type=float, default=0.15)
    ap.add_argument("--coacd-prep-res", type=int, default=30)
    ap.add_argument("--coacd-iters", type=int, default=40)
    ap.add_argument("--max-hulls-piece", type=int, default=12)
    ap.add_argument("--fill-min", type=float, default=0.3,
                    help="min solid fraction of a hull; emptier hulls are re-split (no gap bridging)")
    ap.add_argument("--min-split", type=float, default=0.3, help="never split hulls smaller than this [m]")
    ap.add_argument("--max-verts", type=int, default=16, help="vertices per hull")
    ap.add_argument("--min-thick", type=float, default=0.04, help="min hull thickness [m]")
    ap.add_argument("--max-total-hulls", type=int, default=4000)
    ap.add_argument("--boundary-wall", type=float, default=2.0)
    ap.add_argument("--wall-tolerance", type=float, default=0.15, help="boundary polygon simplification [m]")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    # check
    ap.add_argument("--lidar-poses", type=int, default=200)
    ap.add_argument("--lidar-rounds", type=int, default=3)
    a, _ = ap.parse_known_args()
    {"views": cmd_views, "render": cmd_render, "build": cmd_build, "check": cmd_check, "all": cmd_all}[a.stage](a)


if __name__ == "__main__":
    main()
