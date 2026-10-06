# Splat demo: M20 in a photoreal 3D Gaussian-splat world

This demo uses a 3D Gaussian splat of a real office as the M20 simulation world. MuJoCo simulates
the physics and LiDAR on geometry extracted from the splat. gsplat renders the splat
photorealistically for the robot camera and for the browser view. The full Nav2 stack drives one
complete tour of the room, detects office objects as it goes, and records the lap for replay.

## Run it

```bash
# WSL: fresh sim + Nav2 + Viser + detector + MuJoCo relay; drives one full lap automatically
M20_EXPLORE=1 bash tools/splat/splat_sim_up.sh
```
```powershell
# Windows (optional): native MuJoCo window mirroring the same run
python tools\splat\mujoco_mirror.py --follow
```

Open **Viser** at <http://localhost:8080>. It shows:
- **3D view:** a photoreal chase camera, with the sim's own M20 depth-composited into the splat.
- **Top-down map:** the photoreal floor plan with the robot, its driven trail, the tour stops, and
  detected office objects.
- **Robot camera:** the photoreal feed with detection boxes.
- **Replay:** replays the last lap at 1–8× speed, with loop and seek controls. Laps are saved to
  `~/m20_sim/laps/`.

Press Ctrl-C in the launcher terminal to stop. Relaunch for each lap; repeated laps on one stack
let state drift build up.

## One-time setup

| Step | Command |
|---|---|
| Splat file (not in git, about 213 MB) | copy `gaussians_indoor.ply` to `~/m20_sim/splats/` |
| Sim world + tour | `python3 tools/splat/extract_geometry.py ~/m20_sim/splats/gaussians_indoor.ply`, then `bash tools/setup_sim.sh` |
| 3D-mesh world (optional) | `~/m20_sim/mesh_venv/bin/python tools/splat/extract_mesh.py all` |
| Top-down map | `render_topdown.py --cut 1.5 --max-scale 0.15 --min-opacity 0.3 --write` (in `m20_splat`, see RUN_GUIDE 6a) |
| GPU image | `docker build -f docker/Dockerfile.splat -t m20_splat:latest .` (needs the NVIDIA Container Toolkit) |
| Robot model | vendor URDF in `~/m20_sim/m20_urdf/`, MJCF from `tools/setup_sim.sh` |

## What's here

| File | Role |
|---|---|
| `extract_geometry.py` | Splat → 2.5D box world (`tools/sim/indoor_splat.xml`), boundary wall, spawn, and an exploration tour that routes around narrow gaps |
| `extract_mesh.py` | Splat → 3D mesh world (TSDF + convex hulls), `indoor_splat_mesh.xml` |
| `render_topdown.py` | Orthographic photoreal top-down map (ceiling cut, de-smeared) |
| `splat_camera_node.py` | Photoreal robot camera + depth, rendered at the sim's true pose |
| `detector_node.py` | Office-object detection (OWLv2, 30 office classes), placed in 3D, object map |
| `splat_viewer.py` | Viser app: composite view, explore tour, mini-map, detections, replay |
| `mujoco_mirror_relay.py`, `mujoco_mirror.py` | Stream the sim state to a native MuJoCo window |
| `splat_sim_up.sh` | One-command demo launcher |
| `detect_probe*.py` | Offline detector comparison on tour renders |

## Demo-only settings (never use them for gates or evidence)

`splat_sim_up.sh` turns these on. They are off everywhere else:
- `M20_UPRIGHT_ASSIST=1`: a virtual anti-tip stabilizer, so the robot cannot fall.
- `M20_GT_LOC=1`: ground-truth localization, so Nav2 cannot mislocalize.
- Splat-scene-only Nav2 tuning: 0.7 m/s, the real footprint, keep-away inflation, and DWB
  anti-dither.

## Results and limits

- **Validated (box world):** full 13-stop laps in about 2 min, 0 tip-overs, 0 obstacle contacts,
  never within 19 cm of an obstacle.
- **Office detection:** desks, chairs, filing cabinets, boxes, whiteboards, doors, cones and more.
  Blurry splat regions still produce a few mislabels.
- **Splat quality:** the capture was ground-level, so object tops (for example the central office
  block) render dark from above, and views near the floor are streaky.
- **More:** `docs/RUN_GUIDE.md` sections 6a/6b.
