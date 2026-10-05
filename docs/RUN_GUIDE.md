# Run Guide — how to actually operate this stack

Everything you can run, in the order you would normally need it. Every command
assumes you are at the repo root inside WSL:

```bash
cd /mnt/c/GIT/StatefullXOne/m20_autonomy_ws/m20_autonomy_ws
```

---

## 0. Prerequisites (one time)

| Requirement | Check | Fix |
|---|---|---|
| WSL2 + Docker running | `docker ps` | start Docker Desktop |
| `docker-commander-1` container | `docker ps \| grep commander` | `cd docker && docker compose up -d commander` |
| Stack image built | `docker images \| grep m20_autonomy` | see "Rebuild the image" below |
| Sim image | `docker images \| grep m20_sim` | `docker build -f docker/Dockerfile.sim -t m20_sim:latest .` |
| Vendor MuJoCo model | `ls ~/m20_sim/sdk_deploy/.../m20_mjcf/` | `bash tools/import_vendor.sh` |
| Windows sleep disabled | — | `powercfg /change standby-timeout-ac 0` (PowerShell, admin) |

**The box matters.** A loaded machine starves the 20 Hz control loop and
produces failures that look algorithmic. Before any gate or batch:

```bash
uptime          # load average should be < 1.0
```

---

## 1. The fast path — bring the stack up and drive

```bash
# full stack on the pre-built map, no goal sent
M20_STATIC_MAP=maps/oil_gas_field M20_NO_GOAL=1 bash tools/nav/bringup_plan_a.sh

# then send it somewhere
tools/dev/m20 goto -6 -0.5

# tear everything down
bash tools/nav/kill_stack.sh
```

### Bringup flags (all optional, all guarded)

| Flag | Effect |
|---|---|
| `M20_STATIC_MAP=maps/oil_gas_field` | navigate against the QA'd map (slam in localization mode + static costmap layer). Omit for live-SLAM mapping mode |
| `M20_STANDUP=1` | spawn folded, rise via the commander-gated `standup` behavior before driving |
| `M20_CAMERA=1` | publish `/camera/image_raw` (first-person, ~3 Hz) |
| `M20_NO_GOAL=1` | bring the stack up but send no goal |
| `M20_EKF=1` | run the GPS-fused estimator, publishing `/odom_filtered` |
| `M20_GUARDIAN=0` | disable the drift-bounding anchor guardian (on by default in static-map mode) |
| `GOAL_X` / `GOAL_Y` | goal for the auto-sent goal (default `-6.0 / -0.5`) |
| `RUN_SECS=300` | how long `nav_logger` records |
| `M20_ALLOW_STALE=1` | bypass the installed-vs-repo commander check (debugging only) |

`M20_EKF_TF=1` exists for map building only and bringup **refuses** it in
combination with `M20_STATIC_MAP` — see the frame contract in SECOND_BRAIN §3.

---

## 2. Driving the robot

```bash
tools/dev/m20 goto 2 4                  # single goal via the mission server
tools/dev/m20 mission "-6,-0.5" "2,4"   # multi-waypoint mission
tools/dev/m20 status                    # mode, arming, failsafes
tools/dev/m20sh                         # interactive ROS-sourced shell in the container
```

Raw Nav2 action (bypasses the mission layer):

```bash
docker exec docker-commander-1 bash -lc \
  'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash;
   export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
   "{pose: {header: {frame_id: map}, pose: {position: {x: -6.0, y: -0.5}}}}"'
```

---

## 3. Watching it live (operator station)

The station starts with every bringup: **Foxglove WebSocket on
`ws://localhost:8765`**, rosbridge on `ws://localhost:9090`.

1. Install Foxglove Studio on Windows.
2. Open connection → choose **Foxglove WebSocket** (not Rosbridge) →
   `ws://localhost:8765`.
3. Import the layout: `src/m20_station/config/m20_operator_layout.json`.
4. In the 3D panel set **Display frame: `map`**. Image panel topic:
   `/camera/image_raw`.

Full walkthrough and troubleshooting: `docs/STATION.md`.

**Wrong connection type is the #1 confusion:** port 8765 speaks the Foxglove
protocol; 9090 speaks Rosbridge JSON. Mismatching them yields "Connection
failed" with no further explanation.

---

## 4. Building a map

```bash
bash tools/map/build_map.sh      # one build + landmark quality gate (~10 min)
bash tools/map/best_map.sh 3     # build 3, ship the best-scoring (~35 min)
```

The build runs a coverage route, serializes the pose graph, saves the `.pgm`,
then scores it:

```bash
python3 tools/map/check_map_landmarks.py    # PASS = every landmark < 0.30 m
```

A failing candidate lands in `maps/candidate/` and **never overwrites** the
committed map. The shipped map's score lives in `maps/oil_gas_field.score.txt`.

---

## 5. Tests and gates

### Fast, no ROS required
```bash
make unit          # 24 pytest tests with rclpy shims, runs anywhere in seconds
```

### Component checks (stack must be up)
```bash
python3 tools/nav/drive_check.py            # L1.2 direction/sign check
python3 tools/nav/odom_drift_check.py       # L1.3 odometry error
python3 tools/nav/lidar_realism_check.py    # L1.8 sensor contract
python3 tools/nav/tf_age_probe.py           # transform freshness
```

### Integration gates
```bash
bash tools/ci/batch_nav_test.sh 10   # L3.2: N fresh single-goal runs -> success rate + error
bash tools/ci/patrol_gate.sh         # M1a: one 5-leg site tour, per-leg true error
bash tools/ci/observer_trust.sh      # do the action result, logger, and live probe agree?
```

Results: `tools/ci/out/batch_results.csv`, `tools/ci/out/patrol_results.csv`,
per-run logs in `tools/ci/out/run_N/`.

**Run gates on a quiet box.** The scripts warn when load > 2.0; a contaminated
batch is not evidence and should be discarded, not interpreted.

---

## 6. Videos

```bash
# copy the run's logs out of the container first
docker cp docker-commander-1:/cfg/out tools/nav/out_myrun

# cinematic replay (MuJoCo, real logged trajectory)
OUTDIR=tools/nav/out_myrun OUTMP4=tools/nav/out_myrun/replay.mp4 \
  python3 tools/nav/replay_render.py

# planner view (map, global plan, DWB local plan, laser, true vs est pose)
OUTDIR=tools/nav/out_myrun OUTMP4=tools/nav/out_myrun/nav_view.mp4 \
  python3 tools/nav/render_planner_view.py

# side-by-side, the standard demo format
FF=$(python3 -c "import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())")
$FF -y -i tools/nav/out_myrun/replay.mp4 -i tools/nav/out_myrun/nav_view.mp4 \
  -filter_complex "[0:v]scale=960:540[l];[1:v]scale=960:540[r];[l][r]hstack" \
  docs/media/demo.mp4
```

Rendering is CPU-heavy — stop the stack first, and expect several minutes for
a long run. Both renderers replay **logged data**; they never re-simulate.

---

## 6a. The splat demo (one command, one full lap)

```bash
# WSL — fresh sim + Nav2 + photoreal Viser + MuJoCo mirror relay; drives one full lap automatically
M20_EXPLORE=1 bash tools/splat/splat_sim_up.sh            # add M20_SCENE=indoor_splat_mesh.xml for the 3D-mesh world
```
```powershell
# Windows — the native MuJoCo view of the SAME run (needs `pip install mujoco`)
python tools\splat\mujoco_mirror.py --follow
```
- **Viser** (photoreal splat) at `http://localhost:8080`; the **MuJoCo window** mirrors the same sim.
- **Demo guarantees (ON by default in `splat_sim_up.sh`, OFF everywhere else — never use them for gates or
  sim-to-real evidence):**
  - `M20_UPRIGHT_ASSIST=1` — virtual anti-tip stabilizer in `mujoco_sim.py`: beyond 5° of tilt it torques
    the base back toward level. The robot cannot fall.
  - `M20_GT_LOC=1` — ground-truth localization: odom = sim truth, fixed map->odom, no SLAM. Nav2 cannot
    mislocalize (live SLAM drifted 2.3 m within one lap, and 17 m over an hour of laps).
- **Validated 2026-10-05, box world, final config:** 3/3 fresh runs completed the 13-stop lap
  (117–134 s), 0 tip-overs, 0 robot-box contacts, never within 10 cm of a box (closest 19 cm, < 2.5 % of
  the lap within 30 cm). Stress: 2 runs at 1.6 m/s with 2.0 m/s² accel (the settings that tipped it
  without the assist) — 0 tip-overs, max lean < 10°.
- **How the lap stays clear of obstacles:** keep-away costmaps (inflation 1.0 m with slow decay, padded
  M20 footprint, DWB obstacle weight 0.15), and the tour inserts via-points so no leg's shortest path
  threads a narrow gap (the old home leg squeezed a 0.5 m gap every lap).
- **How it always finishes:** a failed stop is retried twice; a robot parked within 0.8 m of a stop for
  3 s counts as having reached it (DWB could otherwise sit just outside the goal tolerance for 70–98 s).
- One lap per launch. Splat-scene driving limits (staged params only, gates untouched): 0.7 m/s,
  accel 1.0 m/s². Sharp tour corners (> 45°) stop and turn in place; gentle ones are passed through.
- **Top-down photoreal mini-map** (Viser side panel "Top-down map"): the robot to scale, its driven
  trail (orange), tour stops (done = green, current = yellow ring) on a straight-down gsplat render of
  the room. The map is generated once (ceiling and smeared/faint Gaussians removed, 2 cm/px) and its
  geometry lives in the scene yaml's `topdown:` block:
  ```bash
  # tune the ceiling cut (contact sheet), then write the chosen one (in WSL, GPU)
  docker run --rm --gpus all -v ~/m20_sim/splats:/splat:ro -v $PWD/tools/sim:/scene -v $PWD/tools/splat:/splat_tools:ro \
    -v $PWD:/out m20_splat:latest python3 /splat_tools/render_topdown.py --scene /scene/indoor_splat.scene.yaml \
    --cuts 1.0,1.5,2.0,2.5 --max-scale 0.15 --min-opacity 0.3            # -> indoor_splat_topdown_cuts_clean.png
  #   ... same with:  --cut 1.5 --max-scale 0.15 --min-opacity 0.3 --write  -> tools/sim/indoor_splat.topdown.png
  ```
  The capture was ground-level, so object TOPS (e.g. the central office block) were never seen and
  render dark from above; the floor, aisles and furniture edges are accurate (box outlines line up).
- Black box of each lap: `docker cp m20_splat_view:/tmp/explore_log.csv .`
  (t, true x/y/yaw/roll/pitch/z, Nav2 x/y, cmd vx/wz).

## 6b. Splat worlds (3D Gaussian splat as the sim environment)

A splat world has two halves. MuJoCo simulates physics and LiDAR on **boxes
extracted from the splat**. A GPU node renders the **splat itself** as the
robot's camera. Both use the same transform (world = splat + t, recorded in
`tools/sim/<name>.scene.yaml`), so camera and geometry stay aligned.

One-time setup (WSL):

```bash
# 1. the splat (not committed — 200+ MB); its sha256 is in the scene yaml
mkdir -p ~/m20_sim/splats && cp /mnt/c/Users/<you>/Downloads/gaussians_indoor.ply ~/m20_sim/splats/
# 2. geometry: PLY -> tools/sim/indoor_splat.xml + .scene.yaml + docs/media/indoor_splat_occupancy.png
python3 tools/splat/extract_geometry.py ~/m20_sim/splats/gaussians_indoor.ply
bash tools/setup_sim.sh                     # installs + validates the new scene
# 3. GPU camera image (needs the NVIDIA Container Toolkit: `docker run --gpus all` must work)
docker build -f docker/Dockerfile.splat -t m20_splat:latest .
```

Run it:

```bash
M20_SCENE=indoor_splat.xml M20_SPLAT_CAMERA=1 M20_NO_GOAL=1 bash tools/nav/bringup_plan_a.sh
# arm ASSISTED (the default bringup arms TELEOP; the mission server needs 2 or 3), then drive
docker exec docker-commander-1 bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash;
  export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;
  ros2 service call /m20/set_mode m20_msgs/srv/SetMode "{mode: 2, requester: operator}"'
tools/dev/m20 goto 0 -4        # verified: reached, 0.89 m clearance at the goal
```

- **Camera:** in Foxglove, an Image panel on `/camera/image_raw` shows the
  photoreal view. `/camera/depth` is the matching depth image (32FC1, metres).
- **Offline render (no ROS):** render one frame at world pose (x, y, yaw):
  ```bash
  docker run --rm --gpus all -v ~/m20_sim/splats:/splat:ro -v $PWD/tools/sim:/scene:ro -v $PWD:/out \
    m20_splat:latest python3 /splat_camera_node.py --snapshot 0 0 0 /out/snap
  ```
- **View the geometry:** WSLg windows don't paint on some machines. The
  native Windows MuJoCo viewer can load
  `\\wsl.localhost\Ubuntu-24.04\home\<user>\m20_sim\sdk_deploy\src\M20_sdk_deploy\M20_description\m20_mjcf\mjcf\indoor_splat.xml`.
- **Re-extract after changing the splat or the parameters.** The defaults
  are tuned for a floor-heavy indoor capture. Check the occupancy PNG: left is
  the top-down splat, right is the boxes (orange), the boundary wall
  (lavender) and the spawn point (green, +x arrow).

Known limits:
- **2.5D geometry.** Each box runs from the floor to its top, so tables and
  overhangs become solid blocks. Walls the splat barely captured come out
  gappy. A boundary wall around the reconstructed floor keeps the robot
  inside the captured area.
- **No pre-built map.** Splat scenes run live SLAM only; bringup refuses
  `M20_STATIC_MAP`, and it requires `GOAL_X`/`GOAL_Y` or `M20_NO_GOAL=1`.
- **The LiDAR sees boxes.** Its scan shows the extracted geometry, not the
  splat. Sector casts cost about 0.7 ms, against the 2 ms budget, with about
  2.7k boxes.
- **`camera_scan`'s colour detector won't fire.** It looks for sim-red
  wellhead paint (`camera_scan.py:54`), which doesn't exist in photoreal
  images.
- **Oil & gas field only.** The gates (`patrol_gate.sh`, `batch_nav_test.sh`,
  `build_map.sh`) still run only on the oil & gas field.

---

## 7. Rebuild the image (after changing anything in `src/`)

```bash
docker build -f docker/Dockerfile -t m20_autonomy:latest .
docker build -f docker/Dockerfile.autonomy-cyclone -t m20_autonomy:latest .
cd docker && docker compose up -d --force-recreate commander && cd ..
```

The second build layers CycloneDDS + `pointcloud_to_laserscan` onto the base.

Bringup asserts the installed commander matches the repo and **refuses to run
stale safety code** — if you see that error, you skipped this step.

Scripts and configs under `tools/` are copied into the container at bringup,
so they do **not** need an image rebuild. Anything under `src/` does.

---

## 8. Troubleshooting

| Symptom | First thing to check |
|---|---|
| `bringup FAILED` with no detail | run bringup directly (not through a gate script) to see the error |
| "installed commander != repo" | rebuild the image (§7) |
| Robot never moves | commander armed? `tools/dev/m20 status`. Localization seeded? check `slam.log` |
| Goal aborts immediately | `grep -iE "error|abort" tools/nav/out_*/nav2.log` |
| Foxglove "connection failed" | wrong connection type — Foxglove WebSocket for 8765 |
| Topics missing / probes empty | QoS mismatch; sensor topics are BEST_EFFORT. `ros2 topic info -v <topic>` |
| WSL hangs or `Wsl/Service` errors | `wsl --shutdown`, wait, retry. Repeated failures → reboot Windows |
| Everything intermittently weird | `docker logs docker-commander-1` — look for repeated "Commander up" (silent restarts) |
| Sim container gone between commands | WSL idled the VM out; keep long tests in one session |

Every node writes a log to `/cfg/out/` in the container: `nav2.log`,
`slam.log`, `guardian.log`, `engine.log`, `logger.log`, `mission.log`. Read
them before theorizing.

---

## 9. Where things live

```
src/                    ROS 2 packages (rebuild image after editing)
  m20_msgs/             interface contracts — designed before nodes
  m20_commander/        modes, arming, failsafes (has veto over everything)
  m20_locomotion_bridge/ the ONLY node that talks to the vendor SDK
  m20_behaviors/        plugin engine + behaviors (standup, camera_scan, ...)
  m20_missions/         mission action server
  m20_navigation/       nav2_params.yaml lives here
  m20_perception/       sensor + estimator configs
  m20_station/          Foxglove/rosbridge launch + operator layout
tools/                  scripts, copied into the container at bringup
  nav/                  bringup, teardown, logging, probes, renderers
  ci/                   gates and batches
  map/                  map build + quality gate
  dev/                  m20 CLI, container deps, dev shell
docs/                   see ONBOARDING.md for the reading order
maps/                   the QA'd site map artifact + its score
docker/                 Dockerfiles + compose
tests/                  fast ROS-free unit tests (make unit)
```
