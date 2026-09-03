# m20_autonomy_ws

PX4-style autonomy stack for the Deep Robotics Lynx M20 Pro.

**Start here → [`docs/ONBOARDING.md`](docs/ONBOARDING.md)** — where the project
stands, how to work in this codebase, and the ROS 2 (Python + C++) skill path.

Then, as needed:

| Document | For |
|---|---|
| [`docs/ROADMAP.md`](docs/ROADMAP.md) | current milestone and the gate that closes it — read every session |
| [`docs/RUN_GUIDE.md`](docs/RUN_GUIDE.md) | every command: bringup, driving, maps, gates, videos, troubleshooting |
| [`docs/NAV2_ARCHITECTURE.md`](docs/NAV2_ARCHITECTURE.md) | how the navigation stack fits together (standalone explainer) |
| [`docs/LESSONS_LEARNED.md`](docs/LESSONS_LEARNED.md) | every hard-won lesson: symptom → cause → fix → rule |
| [`docs/ENGINEERING_RULES.md`](docs/ENGINEERING_RULES.md) | the four binding rules for code in this repo |
| [`docs/NAV2_SETUP_RUNBOOK.md`](docs/NAV2_SETUP_RUNBOOK.md) | standing a nav stack up from zero, on any robot |
| [`docs/SECOND_BRAIN.md`](docs/SECOND_BRAIN.md) | long-term memory: ADRs, contracts, phases, changelog |

**ROS 2 distro: Humble / Ubuntu 22.04** (ADR-007). The robot's onboard system runs
Foxy, but that stays *behind the vendor boundary* — our stack talks to it through the
`m20_locomotion_bridge` (which wraps the vendor SDK), never by joining the robot's DDS
graph. So the whole stack is Humble; cross-distro is contained at the bridge (ADR-014/015).

---

## What's in the build

One Docker image (`m20_autonomy:latest`, ~3.6 GB) is the deployable artifact (ADR-011):
CI builds it, the robot runs the same image.

| Layer | Contents |
|---|---|
| Base | `ros:humble-ros-base` (Ubuntu 22.04) |
| ROS deps | `nav2-bringup`, `robot-localization`, `foxglove-bridge`, `rosbridge-suite` |
| Our packages | all 9 `m20_*` (see Layout below), colcon-built into `/ws/install` |
| Vendor deps | pinned in `tools/vendor.repos`, **not** built by default — enable with `--build-arg IMPORT_VENDOR=true` (needed for the MuJoCo sim / RL, see Second Brain §6.6) |

Runtime: 5 services, one per node group (`docker/compose.yaml`), host networking +
`ROS_DOMAIN_ID=42` for DDS. Ports exposed by the station bridge: **8765** (Foxglove),
**9090** (rosbridge).

---

## Running the system (Windows + WSL2)

Docker runs **inside WSL2 Ubuntu**, not on Windows directly. Run these from **Windows
PowerShell** (they wrap `wsl`), or drop into `wsl -d Ubuntu` and use the part after `--`.

```powershell
# one-time: Docker Engine in WSL (see tools/ if you need to reinstall)

# build the image (first build ~5-10 min; rebuilds are cached)
wsl -d Ubuntu -- bash -c "cd /mnt/c/GIT/StatefullXOne/m20_autonomy_ws/m20_autonomy_ws && docker compose -f docker/compose.yaml build"

# bring the stack UP (detached)
wsl -d Ubuntu -- bash -c "cd /mnt/c/GIT/StatefullXOne/m20_autonomy_ws/m20_autonomy_ws && docker compose -f docker/compose.yaml up -d"

# status (look for 'healthy')
wsl -d Ubuntu -- bash -c "cd /mnt/c/GIT/StatefullXOne/m20_autonomy_ws/m20_autonomy_ws && docker compose -f docker/compose.yaml ps"

# logs for one service
wsl -d Ubuntu -- bash -c "cd /mnt/c/GIT/StatefullXOne/m20_autonomy_ws/m20_autonomy_ws && docker compose -f docker/compose.yaml logs -f commander"

# stack DOWN (stop + remove containers)
wsl -d Ubuntu -- bash -c "cd /mnt/c/GIT/StatefullXOne/m20_autonomy_ws/m20_autonomy_ws && docker compose -f docker/compose.yaml down"

# fully power off Docker + the WSL VM (free RAM/CPU)
wsl --shutdown
```

Expected healthy state: `commander`, `locomotion_bridge`, `mission_server`,
`behavior_engine` report **(healthy)**; `station_bridge` reports **Up** (it has no
ROS heartbeat, only process-level restart).

### Inspecting the running system (ros2 CLI)

ROS is **not** on your host/WSL shell — it lives in the image. Use the helper, which
runs any `ros2` command in a throwaway container on the stack's DDS domain:

```bash
# from WSL, in the repo root:
bash tools/ros2.sh topic list
bash tools/ros2.sh topic echo /m20/mode
bash tools/ros2.sh topic hz /m20/health
bash tools/ros2.sh node list
```
```powershell
# or from Windows PowerShell:
wsl -d Ubuntu -- bash /mnt/c/GIT/StatefullXOne/m20_autonomy_ws/m20_autonomy_ws/tools/ros2.sh topic list
```

### Operator view (Foxglove) — ADR-013

Install **Foxglove Studio** on Windows, choose "Open connection", and connect to
`ws://localhost:8765`. You get live 3D/plots of `/m20/health`, `/m20/mode`,
`/m20/failsafe`, etc. with zero custom UI. rosbridge is on `ws://localhost:9090`.

---

## Simulation preview — see the robot

Standalone MuJoCo view of the vendor M20 model. Physics runs but nothing controls the
16 joints yet, so the robot settles under gravity — this is a visual check, not the
full sim. Making it stand/walk + publish on the ROS bus is Phase 1, step 2.

```powershell
# one-time GUI prerequisite (mesa/OpenGL for WSLg; needs sudo)
wsl -d Ubuntu -- bash /mnt/c/git/StatefullXOne/m20_autonomy_ws/m20_autonomy_ws/tools/install_gui_deps.sh

# install mujoco + fetch the pinned M20 model (no sudo)
wsl -d Ubuntu -- bash /mnt/c/git/StatefullXOne/m20_autonomy_ws/m20_autonomy_ws/tools/setup_sim.sh

# open the viewer (drag=orbit, scroll=zoom, Space=pause; close window to exit)
wsl -d Ubuntu -- bash /mnt/c/git/StatefullXOne/m20_autonomy_ws/m20_autonomy_ws/tools/view_sim.sh
```
With `make` installed: `make gui-deps` / `make sim-setup` / `make sim-view`. The model
lands in `~/m20_sim`, pinned to the `sdk_deploy` commit in `tools/vendor.repos`.

## Test pyramid (Second Brain §6.6)

```bash
make unit    # 24 fast logic tests (pytest + rclpy shims), no ROS/Docker needed
make image   # build the container
make test    # colcon test inside the container (integration)
make up      # run the full stack
```
`make unit` runs anywhere in seconds and CI runs it first. Keep node logic testable at
this level: side-effects behind injectable objects, state machines as plain methods.

---

## Layout
- `src/m20_msgs` — interface contracts (designed first)
- `src/m20_locomotion_bridge` — sole gateway to the Deep Robotics SDK; cmd/gait + watchdog
- `src/m20_commander` — modes, arming, failsafes, control-authority leases
- `src/m20_navigation` — Nav2 config + behavior trees
- `src/m20_perception` — SLAM / EKF / costmap config
- `src/m20_missions` — mission server + AI agent interface
- `src/m20_behaviors` — behavior engine + plugins (camera_scan, self_right, …)
- `src/m20_station` — operator bridge (foxglove_bridge + rosbridge)
- `src/m20_bringup` — top-level launch
- `docker/` — Dockerfile, compose, healthcheck, entrypoint
- `tools/vendor.repos` — pinned vendor/third-party dependencies
- `tools/ros2.sh` — run ros2 CLI against the running stack
