# Splat demo: write-up (3dgs-sim-env, 2026-09-30 to 2026-10-06)

## Summary

The M20 simulation now runs inside a **photoreal 3D Gaussian-splat scan of a real office**
(`gaussians_indoor.ply`, 900k Gaussians). The project's real Nav2 stack drives the robot through
a full 13-stop tour of the room.

- **Physics:** MuJoCo simulates the physics and LiDAR on geometry extracted from the splat.
- **Rendering:** gsplat renders the splat for the robot camera, the browser view (Viser) and a
  top-down map.
- **Detection:** an open-vocabulary detector finds office objects and places them in the room.
- **Replay:** laps are recorded and can be replayed at 1–8×.

**Result (box world, demo settings):** repeated full laps in about 2 minutes, with **0 tip-overs**
and **0 obstacle contacts**. The robot never came within 19 cm of an obstacle.

Run it with `M20_EXPLORE=1 bash tools/splat/splat_sim_up.sh` and open http://localhost:8080. See
[tools/splat/README.md](../tools/splat/README.md) and [RUN_GUIDE](RUN_GUIDE.md) §6a/6b.

---

## 1. How it works

### 1.1 Components (Docker containers in WSL, ROS 2 over CycloneDDS, domain 42)

| Container | Code | Role |
|---|---|---|
| `m20_sim_run` | `tools/mujoco_sim.py` | MuJoCo physics (500 Hz): the M20 in `tools/sim/indoor_splat.xml`; simulated LiDAR, IMU, odometry |
| `docker-commander-1` | `src/*` | The real robot stack: **Nav2**, commander, locomotion bridge, mission server, pointcloud→laserscan, Foxglove bridge |
| `m20_splat_cam` | `tools/splat/splat_camera_node.py` | Photoreal robot camera + depth, rendered from the splat at the sim's true pose |
| `m20_detector` | `tools/splat/detector_node.py` | Office-object detection on that camera; object map |
| `m20_splat_view` | `tools/splat/splat_viewer.py` | Viser browser app: composite 3D view, tour explorer, top-down mini-map, detections, replay |
| `m20_mujoco_mirror` | `tools/splat/mujoco_mirror_relay.py` | Streams sim state to a native Windows MuJoCo window (`mujoco_mirror.py`) |

### 1.2 Control loop: Nav2 drives

```
Viser explorer ─RunMission─▶ mission server ─NavigateToPose─▶ Nav2 (BT → NavFn A* → DWB, 20 Hz)
   ─/cmd_vel─▶ velocity smoother ─▶ locomotion bridge (skid-steer wheels, legs held) ─▶ MuJoCo
MuJoCo ─/LIDAR/POINTS─▶ pointcloud_to_laserscan ─/scan─▶ Nav2 costmaps      (obstacles)
MuJoCo ─/odom + TF─▶ Nav2                                                     (pose)
commander: arming, modes, failsafes (tip-over → ESTOP) — can veto everything
```

- **The explorer** (in Viser) only chooses *where* to go next. Nav2 chooses *how*: global plan,
  local control and recoveries.
- **Obstacles reach Nav2 only through the simulated LiDAR**, cast against the extracted geometry.
  Nav2 never sees the splat imagery or the detections.

### 1.3 Building a world from a splat
- **`tools/splat/extract_geometry.py`:** the default box world.
  - Opaque Gaussians are binned into a 10 cm grid. A cell counts as an obstacle when its column
    has evidence across several height layers (splats are floor- and ceiling-heavy, with sparse
    walls).
  - The obstacle cells are merged into about 2,700 group-0 boxes, which the LiDAR can see.
  - A boundary wall is added around the reconstructed floor.
  - The world is translated so the most open floor cell becomes the spawn point at the origin.
  - It also generates the **tour**: about 4.5 m spacing, at least 0.7 m clearance, ordered as a
    loop by walking distance with 2-opt, plus **via-points** wherever the straight leg between two
    stops would pass through a narrow gap.
- **`tools/splat/extract_mesh.py`:** an optional real 3D world.
  - It renders about 11k depth views from the splat, fuses them (TSDF) and splits the surface into
    about 1,200 convex hulls (CoACD).
  - The robot can drive under overhangs. The middle office block, which the scan captured poorly,
    comes out mostly solid.
- **`tools/splat/render_topdown.py`:** an orthographic gsplat render of the room from straight
  above.
  - It cuts the ceiling at 1.5 m and drops smeared or faint Gaussians.
  - Output is 2 cm/px, and the scene yaml records the world↔pixel mapping.

## 2. What was built, in order

1. **Machine setup:** Docker Engine and the NVIDIA Container Toolkit in WSL (RTX 5090 visible to
   containers). Fixed `tools/setup_sim.sh`: it now installs every scene, works with PEP 668 on
   Ubuntu 24.04, and resolves its own path correctly.
2. **The splat world:** box extraction, scene file, tour, and the `M20_SCENE` and
   `M20_SPLAT_CAMERA` bringup flags.
3. **Viewing:**
   - WSLg can't paint MuJoCo windows on this laptop. The workarounds are a native Windows MuJoCo
     viewer and the live **MuJoCo mirror**.
   - **Viser** gives a photoreal browser view.
   - The robot in Viser was first drawn from the vendor URDF. It is now the **sim's own MuJoCo
     M20, depth-composited into the splat** (about 18–20 fps, GPU EGL via WSL D3D12).
4. **Three parallel tracks** (agents):
   - **3D mesh world:** done.
   - **MuJoCo robot composite:** done.
   - **Isaac Sim feasibility:** runs on native Windows only (not WSL2), and RAM is tight
     (31.5 GB). Parked for now.
5. **Reliable full loop:** the long tail in §4. The result is the validated demo config in §3.
6. **Photoreal top-down mini-map** in Viser: robot to scale, driven trail, tour stops (done,
   current, to come).
7. **Office object detection:**
   - First COCO FasterRCNN-v2. It ran at about 55 ms per frame, but COCO has no desk, whiteboard,
     filing cabinet or similar classes, and blur gave false positives (bed, airplane, train).
   - Switched to **OWLv2**, open-vocabulary, queried with **30 office classes** drawn from the
     Objects365 and LVIS categories, at threshold 0.3. That takes about 300 ms per frame at 3 Hz.
   - Detections are placed in 3D with the camera depth and **confirmed after 3 sightings**, then
     shown on the camera panel and the mini-map.
   - One lap confirmed 64 objects: desks, chairs, filing cabinets, boxes, trash cans, doors,
     whiteboards, cones, a printer and more.
8. **Lap replay:**
   - Pose and joints are recorded at 20 Hz and interpolated (lerp + slerp; 0.1 mm error in an
     offline test).
   - Camera frames, the object map and tour progress are timestamped.
   - Playback runs at 1–8× with loop and seek.
   - Laps are saved to `~/m20_sim/laps/` and the newest one loads automatically.

## 3. Demo configuration and results

**Demo-only settings.** `splat_sim_up.sh` turns them on; they are **off everywhere else**. Never
use them for gates or sim-to-real evidence.

| Setting | Effect |
|---|---|
| `M20_UPRIGHT_ASSIST=1` (`mujoco_sim.py`) | A virtual stabilizer torques the base toward level beyond 5° of tilt, so the robot cannot fall |
| `M20_GT_LOC=1` (bringup + `M20_GT_ODOM`) | Odometry = sim truth, fixed map→odom, no SLAM, so Nav2 cannot mislocalize |
| Splat-scene Nav2 params (staged copy only; `nav2_params.yaml` and the oil & gas gates are untouched) | 0.7 m/s, accel 1.0 m/s², the real 0.86 × 0.56 m footprint, inflation 1.0 m (slow decay), DWB obstacle weight 0.15, DWB min speeds 0.10 m/s / 0.25 rad/s (anti-dither), no-progress timeout 15 s |
| Explorer (Viser) | Pass-through at gentle corners (< 45°), stop and turn at sharp ones; 2 retries per stop; progress-based stall rescue (within 1.2 m, no 10 cm of progress in 5 s); localization watchdog |

**Validation (box world, fresh sim per run):**

| Check | Result |
|---|---|
| Laps completed | every final-config run 13/13, 0 skipped, about 117–135 s |
| Tip-overs | 0, including 2 stress runs at 1.6 m/s and 2.0 m/s² (the settings that tipped it before) |
| Obstacle contacts (real M20 collision geometry) | 0 |
| Clearance | closest 19–36 cm; < 3.5 % of the lap within 30 cm (was 10–18 %) |
| Long stalls | none in the final config (was up to 160 s) |

## 4. Problems found and how they were fixed

| Problem | Cause | Fix |
|---|---|---|
| Robot tipped at 1.0–1.6 m/s | Hard acceleration (2.0 m/s²) while turning fishtails the skid-steer: uncommanded −1.2 rad/s yaw, the body lifts and rolls | 0.7 m/s at accel 1.0; turn in place at sharp corners; upright assist |
| Robot tipped mid-session | Live-SLAM drift grew to 17 m over an hour of laps, so Nav2 drove on a wrong pose | One lap per launch; localization watchdog; ground-truth localization for the demo |
| Speed patch silently did nothing | `docker exec` without `-i` drops the heredoc on stdin | Added `-i`. **The existing `M20_STATIC_MAP` patch has the same bug and is still open** (see §5) |
| Wheel scraped boxes every lap | Nav2 took the shortest route through a 0.5 m gap; the 0.45 m circular footprint understated the M20's corners; the LiDAR's 0.5 m blind radius | Real footprint, keep-away inflation, tour via-points around narrow gaps |
| 70–160 s stalls | DWB settled on "vx 0, wz ±0.04", which is below what the skid-steer can turn, just outside the goal tolerance | DWB minimum speeds (anti-dither); progress-based stall rescue |
| Two explorers raced through every stop | A second explore client preempted each goal | One explorer at a time; halt after 3 instant failures |
| Blank MuJoCo window | WSLg compositor doesn't paint on this machine | Native Windows viewer and mirror relay |
| Viewer crash on start | An unpinned pip install upgraded numpy to 2.x, which broke ROS Humble's cv2 | `numpy<2` in every pip resolve |
| OWLv2 failed to import | Ubuntu's Pillow 9.0 has no `Image.Resampling` | `pillow>=10` |
| torchvision install pulled in a new torch | Unpinned resolve | `--no-deps` and a pinned torchvision 0.26.0 |

## 5. Known limits and open items

- **The splat itself.** The scan was captured at ground level: object tops (for example the
  central office block) were never seen, views near the floor are streaky, and walls are sparse.
  This shows in the top-down map, the box geometry and some detector mislabels.
- **2.5D boxes.** Tables and overhangs are solid blocks in the default world. The mesh world fixes
  that, but it hasn't been validated at demo settings.
- **Demo crutches.** Upright assist and ground-truth localization hide real falls and real SLAM
  drift. Getting real physics and live SLAM through full laps is still to do.
- **Open bug.** The `M20_STATIC_MAP` global-costmap patch in `bringup_plan_a.sh` is a silent no-op
  (missing `-i`). Static-map runs have likely used the rolling costmap all along. Fixing it means
  re-gating the oil & gas results.
- **Detection runs at 3 Hz and is display-only.** It doesn't feed navigation.
- **Pre-existing unit test failures**, unrelated to this work: `test_behavior_engine.py`
  (`HistoryPolicy` missing from the test shims) and `test_bridge.py::test_watchdog_halts_on_timeout`.

## 6. Possible next steps

- Validate the 3D mesh world at demo settings.
- Turn the demo crutches off and harden live SLAM and the dynamics for real laps.
- Show Nav2's live planned path on the mini-map, next to the planned route and the driven trail.
- Per-leg speed limits (fast in open aisles, slow near clutter) for quicker laps without scrapes.
- Fix and re-gate the `M20_STATIC_MAP` patch.
