# M20 Autonomy Stack — Second Brain

> Living source of truth for the project. Every meaningful fact, decision, contract,
> and open question lives here. If it isn't in this document, it didn't happen.
> Update the changelog at the bottom whenever you edit.

**Project codename:** `m20_autonomy`
**Last updated:** 2026-07-07 (rev 6)
**Status (2026-07-11, honest):** Phase 1 DONE + heavily polished in MuJoCo sim (stand,
drive, turn/circle/fig-8, watchdog, armature + yaw-rate feedback — all benchmarked/videoed).
Phase 2 PARTIAL: a *custom* lightweight EKF (wheel+IMU+GPS) live-verified at 0.48 m — NOT
yet robot_localization, and nav doesn't yet run on it. **Phase 3 PLAN A WORKING + HONEST
(rev 36):** the PRODUCTION stack — **Nav2** (Regulated Pure Pursuit + rotate-to-heading +
NavFn/A*, rolling costmaps) + **slam_toolbox** — autonomously navigated the oil & gas field
to a NavigateToPose goal, threading between skid1/skid2 (real costmap obstacle avoidance,
live 10 m LiDAR), SLAM map built live. **Localization is HONEST: no ground-truth TF** — the
sim feeds dead-reckoned wheel+gyro odometry that DRIFTS, slam_toolbox corrects it via
scan-matching, and the robot reaches the TRUE goal within 0.82 m (raw uncorrected odom
alone ends ~5 m off). Nothing consumes perfect info. Config:
`src/m20_navigation/config/nav2_params.yaml`; repro: `tools/nav/bringup_plan_a.sh`;
proof: `docs/media/nav2_plan_a_result.png` + `nav2_plan_a_run.gif`. Next honesty/accuracy
step: fuse GPS (robot_localization EKF) to bound residual SLAM error outdoors.
Plan B (3D perception/traversability) deferred.
Phase 1.5 (Foxglove) skipped (used rendered videos). Behavior engine + standup exist early.
Vendor SDK reality mapped from public GitHub (rev 6). **First integration run done
(rev 7):** full stack builds and runs green in docker compose on a dev box (WSL2) —
all 5 services up, 4 healthy + station bridge serving Foxglove/rosbridge. Still no
hardware/sim-robot integration.

---

## 1. Mission

Build a PX4-style autonomy stack for the Deep Robotics Lynx M20 Pro: small,
single-responsibility modules communicating over a shared message bus (ROS 2 / DDS),
with a paranoid Commander owning modes and failsafes, Nav2 as the Navigator,
GPU-accelerated perception on Jetson, and an AI/mission layer on top that can only
*request* — never override — the safety layer.

Design principles (stolen deliberately from PX4):

1. Modules communicate only through the bus. No direct function calls across module
   boundaries. Commander publishes mode; Navigator subscribes.
2. Message contracts first. Interfaces in `m20_msgs` are designed before nodes are
   written. Changing a contract requires a decision-log entry.
3. Stop at the vendor boundary — by default. Nominal navigation commands
   velocities and gait/mode requests only. Novel dynamic behaviors may cross into
   joint-level control, but ONLY through a Commander-granted, time-leased authority
   (ADR-008). We still never modify vendor firmware or onboard controllers.
4. Commander is paranoid. Every autonomy feature ships with its failsafe.
5. Sim first. Nothing runs on hardware that hasn't run in Gazebo/MuJoCo/Isaac Sim.

---

## 2. Platform facts — Deep Robotics Lynx M20 Pro

Facts gathered from vendor material and third-party coverage. Verify anything marked
(unconfirmed) directly with Deep Robotics support before depending on it.

| Property | Value |
|---|---|
| Type | Wheel-legged hybrid quadruped (wheels in feet, lockable for legged mode) |
| Dimensions / weight | ~820 × 430 × 570 mm, ~30–33 kg incl. battery |
| Speed | 5 m/s lab max; ~2–3 m/s operational (safety-limited in user mode) |
| Terrain | 45° slopes, 25 cm continuous stairs, up to 80 cm single obstacle, 50 cm corridor clearance |
| Endurance | ~3 h / 15 km unloaded, ~2.5 h / 12 km loaded; hot-swappable battery (~724 Wh) |
| Payload | 15 kg rated (figures up to 20 kg appear in some coverage — confirm for Pro) |
| Environmental | IP66, −20 °C to 55 °C |
| Sensors (built-in) | Dual 96-line LiDARs (360°×90° combined FOV), wide-angle cameras, GPS, lighting (2 bidirectional front/rear LED flashlights for low-light/dark ops; vendor does not publish a lumens rating) |
| Onboard compute | Industrial processors + NVIDIA Jetson Orin NX class module (reported ~100 TOPS, ~50 Hz onboard SLAM) (unconfirmed exact SKU for Pro) |
| Built-in autonomy | Vendor SLAM mapping/navigation, omnidirectional obstacle avoidance, point-cloud surround view |
| Connectivity / expansion | Wi-Fi image transmission, Gigabit Ethernet payload port, USB 3.0, 72 V power out, mounting rails, OTA updates |
| Control modes | Autonomous, intelligent follow, manual/remote, AI-motion mode |
| E-stop | Physical button + software e-stop |
| SDK | Open C++/Python SDK; native ROS 1 and ROS 2 support; UDP/CAN protocols |
| Simulation | Gazebo, MuJoCo, Isaac Sim supported |
| Vendor GitHub | https://github.com/DeepRoboticsLab — `rl_training` (M20 supported, ROS 2 version), M20 lidar secondary-development repo, Android SDK for M20 |

**Vendor SDK caution:** Deep Robotics' ROS 2 stack reportedly updates frequently
(weekly-ish) and is less battle-tested than mature SDKs. **Pin every vendor repo to a
commit hash** in `tools/vendor.repos` and test firmware/OTA updates on a bench robot
or in sim before fleet rollout.

---

## 3. Architecture

Rendered diagram: `docs/assets/m20_architecture.svg`

Two columns over one bus. Command path flows down; sensing path flows up.

```
        Command interfaces (operator UI, AI agent, WebSocket/rosbridge)
                              |
                          Commander            <- modes, arming, failsafes (m20_commander)
                              |
                          Navigator            <- Nav2 planning + behavior trees (m20_navigation)
                              |        ^
                              |        |  pose, costmaps
                              |   State estimator  <- EKF fusion (robot_localization)
                              |        ^
                              |    Perception      <- SLAM, nvblox/costmaps, detection (m20_perception)
                              |        ^
                       Locomotion bridge           <- cmd_vel + gait/mode requests (m20_locomotion_bridge)
                              |        |
                   [vendor boundary — do not cross]
                              |        |
                  M20 onboard motion control + sensors (Deep Robotics)
```

### PX4 concept mapping

| PX4 | This stack | Package |
|---|---|---|
| uORB message bus | ROS 2 topics/services/actions (DDS) | — |
| Commander | Mode/arming/failsafe state machine | `m20_commander` |
| Navigator | Nav2 BT navigator, missions, recoveries | `m20_navigation` |
| EKF2 | robot_localization EKF (odom + IMU + SLAM pose) | `m20_perception` (config) |
| Position/attitude controllers + mixer | Locomotion bridge → vendor onboard controller | `m20_locomotion_bridge` |
| MAVLink | ROS 2 API + rosbridge/WebSocket for UI & agent | `m20_missions` |
| Flight modes | idle / teleop / assisted / autonomous / e-stop | `m20_msgs/RobotMode` |
| QGroundControl | Operator console (TBD; rviz + web UI initially) | — |

### Module registry

| Package | Responsibility | Talks to | Status |
|---|---|---|---|
| `m20_msgs` | All custom interfaces (msg/srv). The contracts. | everyone | drafted |
| `m20_locomotion_bridge` | Wrap Deep Robotics SDK: subscribe `/cmd_vel` + `GaitRequest`, publish odom/joint/battery state. Watchdog: zero-velocity on command timeout. | vendor SDK, Commander, Navigator | stub |
| `m20_commander` | Mode state machine, arming, failsafe monitors (battery, comms, estimator health, e-stop). Publishes `RobotMode`, `FailsafeStatus`. Veto authority over all goal sources. | everyone | stub (runnable) |
| `m20_navigation` | Nav2 bringup, costmap config, BT XML incl. gait-arbitration nodes (wheeled ↔ legged on terrain class). | bridge, estimator, Commander | config placeholder |
| `m20_perception` | Sensor drivers/relays, SLAM (vendor lidar SLAM or FAST-LIO2 vs Isaac ROS VSLAM — see OQ-2), nvblox/elevation costmaps, robot_localization EKF. | Navigator, Commander | config placeholder |
| `m20_missions` | Mission/waypoint server; AI agent interface (LLM/VLM task → Nav2 goals). Requests only; Commander can veto. | Commander, Navigator | stub |
| `m20_behaviors` | Behavior Engine: plugin registry + executor for novel behaviors (jump_pipe, self_right, three_wheel, camera_scan, future unknowns). Enforces lifecycle, timeouts, authority leases. | Commander, bridge, perception | engine runnable, plugins stubbed |
| `m20_station` | Operator/monitoring bridge (QGroundControl analog): foxglove_bridge + rosbridge on the robot; Foxglove Studio day-1, custom web UI later. | all topics (read), Commander/missions (write) | launch stub |
| `m20_bringup` | Top-level launch: sim bringup, robot bringup, rviz config. | all | placeholder |

### Message contracts (v0 — see `src/m20_msgs`)

- `RobotMode.msg` — current mode + source of authority. Published by Commander only.
- `SetMode.srv` — request a mode change; Commander accepts/rejects with reason.
- `GaitRequest.msg` — WHEELED / LEGGED / HYBRID / STAND / SIT request with requester id.
- `FailsafeStatus.msg` — bitfield of active failsafes + severity + human-readable detail.
- `HealthReport.msg` — per-module heartbeat: node name, status enum, message.
- `MissionItem.msg` — waypoint/task primitive for the mission server.
- `BehaviorRequest.msg` / `BehaviorStatus.msg` — invoke a named behavior; lifecycle
  states PRECHECK → RUNNING → SUCCEEDED/ABORTED/FAILED.
- `ControlAuthority.msg` / `SetControlAuthority.srv` — which tier owns the actuators
  (TIER_VENDOR high-level vs TIER_POLICY joint-level). Commander is the only server;
  leases are time-bounded, renewal required, revoked instantly on any failsafe.

Contract change process: propose → add decision-log entry → bump `# Version:` comment
in the .msg → update all consumers in the same PR.

---

## 4. Decision log (append-only)

- **ADR-001 (2026-07-07) — No PX4 fork.** PX4's value is architectural, not code.
  Aviation-specific estimators/mixers/uORB don't transfer; ROS 2 provides the bus.
  *Consequence:* we copy the module discipline, not the repo.
- **ADR-002 (2026-07-07) — Vendor locomotion is a black box.** We command `cmd_vel`
  and gait/mode requests; Deep Robotics onboard control owns balance and joints. We
  do not train/deploy our own locomotion policies for now (their `rl_training` exists
  if that ever changes). *Consequence:* our lowest-level artifact is the bridge node.
- **ADR-003 (2026-07-07) — Own repo, vendored deps.** Our packages live in this
  workspace; vendor and third-party repos are pinned by commit in `tools/vendor.repos`
  and never modified (patch + upstream if a build fix is unavoidable).
- **ADR-004 (2026-07-07) — Contracts before code.** `m20_msgs` is designed first;
  modules build against it independently.
- **ADR-005 (2026-07-07) — Commander has veto over the AI layer.** The mission/agent
  layer publishes goals like any client; only Commander changes modes; failsafes
  preempt everything.
- **ADR-006 (2026-07-07) — Target platform switched W1 → Lynx M20 Pro.** Better
  documented SDK (native ROS 1/2, C++/Python), public GitHub with M20-specific ROS 2
  repos, built-in dual lidar + Jetson-class compute. Architecture unchanged.
- **ADR-008 (2026-07-07) — Two-tier control authority (amends ADR-002).** Novel
  behaviors (jumping, self-righting, three-wheel locomotion) require joint-level
  control that cmd_vel cannot express. We adopt the PX4 offboard-mode pattern: a
  TIER_POLICY lease, granted and revocable solely by Commander, time-bounded, killed
  instantly by any failsafe, reverting to TIER_VENDOR. Policies come from Deep
  Robotics `rl_training` / Isaac Lab, exported to ONNX, streamed by the bridge's
  low-level channel. ADR-002 still holds for all nominal navigation.
  *Consequence:* the bridge gains a joint-level channel gated on /m20/control_authority.
- **ADR-009 (2026-07-07) — Behaviors are plugins.** Every behavior implements one
  contract (`m20_behaviors/base.py`: SPEC + precheck/start/step/abort) and is
  auto-discovered from a directory. The engine owns lifecycle, single-behavior
  exclusivity, hard timeouts, and abort-on-mode-change, so behaviors can't skip
  safety. *Consequence:* behaviors "we have not thought of yet" are one new file.
- **ADR-010 (2026-07-07) — Sim gate for dynamic behaviors.** No TIER_POLICY behavior
  touches hardware before >95% success across domain-randomized sim trials, and
  first hardware trials are rigged (crash rails / harness). Non-negotiable.
- **ADR-011 (2026-07-07) — Containerized, CI-gated deployment.** Pattern inspired
  by Autoware Open AD Kit / SOAFEE: one pinned stack image built in CI on every
  commit, colcon tests run inside it, the SAME artifact deploys to the Jetson.
  Also our mitigation for the fast-moving vendor SDK — pins live in the image.
  We do NOT adopt Autoware's planning/control modules (car-shaped: lanelet maps,
  Ackermann assumptions). Do not relitigate. Its lidar detection/tracking pipeline
  is the one candidate borrow — see OQ-12.
- **ADR-012 (2026-07-07) — Orchestration: compose + systemd, not k8s.** Single-robot
  scale doesn't justify a Kubernetes control plane on the Jetson. Three watchdog
  layers instead: (1) ROS level — Commander consumes /m20/health and reacts with
  failsafes; (2) container level — Docker healthchecks probe /m20/health freshness
  (liveness = publishing, not merely running) with restart: unless-stopped;
  (3) host level — systemd unit keeps the compose stack itself up. *Upgrade path:*
  k3s for multi-robot fleets later; containerization makes that config, not rework.
- **ADR-013 (2026-07-07) — Operator station: Foxglove first, custom UI later.**
  We do not build a UI before the stack exists. Day 1: foxglove_bridge on the
  robot + Foxglove Studio on the operator laptop gives 3D lidar/costmap/TF views,
  camera streams, telemetry plots, and a teleop panel with zero custom code.
  rosbridge runs alongside as the JSON/WebSocket gateway a custom web console
  (and the AI agent) will use later — one gateway, multiple clients. Teleop
  publishing /cmd_vel is only honored in MODE_TELEOP (Commander enforces).
  Low-latency drive video is a separate problem from monitoring video: OQ-14.
- **ADR-014 (2026-07-07) — Fleet-ready now, fleet-built later.** Multi-robot
  orchestration is deferred, but three rules apply from today so it stays doable:
  (1) every launch file accepts a `robot_id` namespace argument; no node hardcodes
  a global topic; (2) all external interaction with a robot goes through exactly
  one gateway (rosbridge/WebSocket) — the future fleet server talks to N robot
  gateways over the network, never joins the robots' DDS domains (each robot keeps
  its own ROS_DOMAIN_ID); (3) `m20_missions` is the sole fleet integration point on
  each robot — the fleet layer assigns missions and reads status, it never drives.
  *Upgrade path:* fleet server + k3s (ADR-012) at Phase 10; robots unchanged.
- **ADR-007 (2026-07-07, DECIDED) — ROS 2 distro = Humble on Ubuntu 22.04.**
  Resolved via the vendor M20 deploy SDK README (see ADR-015): the vendor's own
  x86/sim path is **Humble on Ubuntu 22.04**, the robot's onboard system runs
  **Foxy**. We standardize our whole stack on Humble (matches vendor sim, widest
  Isaac ROS support, and our existing Dockerfile). Cross-distro (Humble ↔ onboard
  Foxy) is contained at the bridge boundary, not spread through the stack.
  *Consequence:* Jazzy dropped; the Humble pin in `docker/Dockerfile` is now a
  decision, not a default. Onboard Foxy interop tracked in OQ-1.
- **ADR-015 (2026-07-07) — Vendor SDK reality: joint-tier is public, velocity-tier
  is not.** Mapped from `DeepRoboticsLab/sdk_deploy` (`src/M20_sdk_deploy`). The
  public SDK is a **joint-level DDS (`drdds`) + ONNX policy** deploy path: subscribes
  `/JOINTS_CMD` (`JointsDataCmd`: pos/vel/kp/kd/torque_ff, PD+feedforward), publishes
  `/JOINTS_DATA` (16 joints) and `/IMU_DATA` at 200 Hz, ships a MuJoCo model + ROS 2
  sim node and a trained `policy.onnx`. It exposes **no `cmd_vel`, no gait/mode
  service, no odom** — the high-level velocity/SLAM/gait interface lives in the
  robot's onboard app, not on GitHub. *Consequences:* (1) ADR-008's TIER_POLICY
  joint channel is exactly this public interface — we can build/sim it now; the
  bridge's low-level channel wraps `drdds` JointsData/Cmd. (2) ADR-002's TIER_VENDOR
  `cmd_vel` interface is the genuinely-unconfirmed part — it needs on-robot/onboard
  docs (OQ-3). (3) The vendor's sim of record for low-level M20 is **MuJoCo**, not
  Gazebo (OQ-5). (4) `rl_training` (Isaac Lab 2.3.2 / Isaac Sim 5.1, ONNX export)
  is the confirmed Phase-8 training front end feeding this deploy path.

---

## 5. Roadmap

- **Phase 0 — Scaffold (this commit).** Workspace, contracts v0, stub nodes, this doc.
- **Phase 1 — Bridge in sim.** Vendor sim (Gazebo or MuJoCo) up; joystick →
  `m20_locomotion_bridge` → robot moves; gait switch works; command-timeout watchdog
  proven. *Exit test:* pulling the joystick node kills motion within 500 ms.
- **Phase 1.5 — Station online.** foxglove_bridge + rosbridge in the compose
  stack; operator layout built (config/foxglove_layout_notes.md). *Exit test:*
  operator watches sim lidar/TF/health live and teleops via Foxglove panel,
  denied unless MODE_TELEOP.
- **Phase 2 — Estimation.** robot_localization fusing sim odom + IMU; TF tree clean
  (`map → odom → base_link`); covariance sane during aggressive driving.
- **Phase 3 — Navigation.** Nav2 on a static map in sim, then live SLAM + local
  costmaps from lidar. MPPI (or vendor-tuned) controller → `/cmd_vel`. *Exit test:*
  A→B with dynamic obstacle inserted mid-run.
- **Phase 4 — Commander.** Full mode machine + failsafes wired to real monitors
  (battery topic, comms heartbeat, estimator covariance, e-stop input). Fault
  injection tests in sim. *Exit test:* every failsafe fires in a scripted scenario.
- **Phase 5 — Gait arbitration.** Terrain classification from elevation/costmap →
  BT nodes request LEGGED for stairs/rubble, WHEELED for flat. *Exit test:* sim course
  with flat + stairs traversed with autonomous mode switches.
- **Phase 5.5 — First behavior (camera_scan).** TIER_VENDOR only: detector +
  body-sweep gaze controller through the full engine pipeline (request → precheck →
  run → abort paths). Proves the Behavior Engine with zero joint-level risk.
  *Exit test:* robot finds and centers a target object from an arbitrary start yaw;
  operator abort mid-scan leaves the robot stationary and re-navigable.
- **Phase 6 — Hardware bring-up.** Phases 1–5 replayed on the real M20 Pro in a
  controlled space, tethered/e-stop supervised, one phase at a time.
- **Phase 7 — Missions + AI.** Mission server, operator UI, LLM/VLM agent issuing
  goals through `m20_missions`. Cloud API first, on-device later if offline needed.
- **Phase 8 — Dynamic behaviors (TIER_POLICY).** Order: self_right → three_wheel →
  jump_pipe (ascending dynamic risk; self-right is also the safety net for the other
  two). Per behavior: train in rl_training/Isaac Lab → ONNX export → sim gate
  (ADR-010) → rigged hardware trials → field. *Exit tests:* robot self-rights from
  both sides; drives 20 m on three wheels; clears a 0.3 m pipe in sim then hardware.
- **Phase 9 — Novel behavior pipeline.** Document the repeatable path from idea →
  plugin file → trained policy → sim gate → deployment, so future behaviors nobody
  has thought of yet are routine, not heroics.
- **Phase 10 — Fleet orchestration.** Fleet server (likely k3s-hosted, ADR-012)
  connecting to each robot's gateway: mission assignment, fleet-wide monitoring
  dashboard, multi-robot task allocation. Doable-by-design per ADR-014.
  *Exit test:* two robots (or robot + sim twin) run disjoint missions from one
  console with per-robot e-stop.

---

## 6. Open questions (resolve with Deep Robotics / by experiment)

- **OQ-1 (PARTIALLY RESOLVED — ADR-007/015):** Distro/OS confirmed from the vendor
  M20 SDK README: **Ubuntu 22.04**, **Humble** on the x86/sim side, **Foxy** onboard
  the robot (SSH to `10.21.31.103` over the robot's WiFi AP `M20*` / pw `12345678`,
  sources `/opt/ros/foxy` + `/opt/robot/scripts/setup_ros2.sh`). *Still open:* exact
  JetPack/L4T on the Jetson (OQ-13), and whether we get persistent deploy access to
  the onboard computer vs. running our stack on a payload computer over GigE.
- **OQ-2:** Use vendor onboard SLAM/nav outputs (pose, maps, obstacle avoidance) as
  inputs to our stack, or run our own SLAM (FAST-LIO2 on the dual 96-line lidars vs
  Isaac ROS VSLAM)? Can vendor SLAM pose be subscribed at 50 Hz over ROS 2?
- **OQ-3 (PARTIALLY RESOLVED — ADR-015):** Low-level (TIER_POLICY) surface is known
  from `M20_sdk_deploy`: DDS (`drdds`) `/JOINTS_CMD` (JointsDataCmd), `/JOINTS_DATA`
  (16-joint pos/vel/torque/temp), `/IMU_DATA`, all ~200 Hz; battery available on the
  DDS interface. *Still open (the real gap):* the high-level TIER_VENDOR surface —
  `cmd_vel`/Twist velocity control, programmatic gait/mode switching, odom, and
  software/physical e-stop state on the bus. These are NOT in the public SDK; confirm
  from the onboard app/robotserver_sdk or on-robot introspection (`ros2 topic list`).
- **OQ-4:** Does software e-stop via SDK exist, and does the physical e-stop state
  appear on the bus so Commander can observe it?
- **OQ-5 (RESOLVED — ADR-015):** The vendor ships a **MuJoCo** M20 model
  (`M20_sdk_deploy/M20_description/m20_mjcf`: `M20.xml`, `M20_stair.xml`, `scene.xml`)
  driven by a ROS 2 sim node (`mujoco_simulation_ros2.py`) that speaks the **same
  DDS joint interface as hardware** (`/JOINTS_CMD` in, `/JOINTS_DATA` + `/IMU_DATA`
  out) — so sim↔real parity holds at the joint tier. RL training is in **Isaac Lab /
  Isaac Sim 5.1** (`rl_training`). No vendor Gazebo model for M20. *Note:* this is
  low-level (joint) sim; a Nav2/`cmd_vel`-level sim for Phase 3 still needs a
  velocity-command wrapper (ties to OQ-3's high-level surface).
- **OQ-6:** Payload budget: if we add our own Jetson + sensors, what does that do to
  the 15 kg payload and endurance numbers?
- **OQ-7:** Confirm M20 **Pro** deltas vs base M20 (compute, sensors, payload).
- **OQ-8:** Does the SDK expose body-pose commands (pitch/roll/height) at the
  vendor tier? Needed for camera_scan without joint-level control.
- **OQ-9:** Does the vendor controller already include fall recovery / self-righting?
  If yes, self_right becomes a thin wrapper, not a trained policy.
- **OQ-10:** Joint-level API: rate, latency, torque limits, and safety envelope for
  streaming external policy outputs? Does `rl_training` sim-to-real deploy path give
  this for free, and what watchdog does the vendor run against bad joint targets?
- **OQ-12:** Dynamic-obstacle tracking: Autoware lidar perception
  (ground seg + CenterPoint + multi-object tracking) vs Isaac ROS detection for
  feeding predicted-obstacle layers into Nav2 costmaps? Evaluate in Phase 3+ only
  if missions involve moving people/vehicles.
- **OQ-13:** Jetson container runtime: which L4T/JetPack base image matches the
  robot's JetPack (ties to OQ-1)? Confirm nvidia-container-toolkit setup and GPU
  access (`runtime: nvidia`) for perception services.
- **OQ-14:** Drive-camera video path for teleop: vendor Wi-Fi image transmission
  vs our own WebRTC/RTSP relay off the GigE camera feed? Target < 200 ms
  glass-to-glass; measure both. Monitoring video via Foxglove is fine as-is.
- **OQ-15:** Comms architecture for field ops and future fleet: robot Wi-Fi AP vs
  site network vs LTE/5G router payload; bandwidth budget for lidar + video
  streaming; VPN/auth story for the rosbridge gateway (it must NOT be an open
  websocket in production).
- **OQ-11:** Actuator thermal/current headroom for jump launches — what does the
  vendor certify vs void the warranty on? Ask BEFORE the first hardware jump.
- **OQ-16 (RESOLVED, rev 17) — turning was a controller-gain bug, NOT a physical limit.**
  4-wheel skid-steer pivots fine; my wheel *velocity* gain was far too soft
  (kd≈0.6–1 → only ~6 N·m, wheels never developed pivot force). Diagnosis: at **kd=5**
  in-place pivot hits **~1.0 rad/s** (vs 0.25 at kd=1); circle and figure-8 courses
  now track properly. Also fixed a forward-direction sign error (+wheel-vel drives -x,
  so `send_velocity` negates vx). Bridge `SimVendorSDK` updated (wheel kd 0.6→5.0 +
  sign); sim_drive_test likewise. *Lesson:* benchmark + push on surprising results
  before concluding a physical limit. (Real-hardware turning path — vendor cmd_vel vs
  policy — still tracked under OQ-3.)

---

## 6.5 Deployment & runtime health (ADR-011/012)

- `docker/Dockerfile` — the pinned stack artifact (CI builds it, robot runs it).
- `docker/compose.yaml` — one service per node group, host networking for DDS,
  `restart: unless-stopped`, healthcheck = `/m20/health` freshness per node.
- `docker/healthcheck.py` — liveness probe: node considered healthy only if its
  HealthReport (status OK/WARN) arrived within HEALTH_MAX_AGE seconds. NOTE (rev 7):
  the compose healthcheck MUST source ROS itself (`bash -c 'source ...setup.bash &&
  python3 healthcheck.py'`) — Docker healthchecks run under /bin/sh and bypass the
  entrypoint, so rclpy/m20_msgs are otherwise not importable. Every m20_* node
  (incl. Commander, rev 7) publishes its own 1 Hz HealthReport or its check can't pass.
- Station bringup needs `ros-humble-foxglove-bridge` + `ros-humble-rosbridge-suite`
  baked into the image (added rev 7). Launch args are ROS-style `name:=value`; compose
  passes `robot_id` only when set (empty = single robot, ADR-014).
- `deploy/m20-stack.service` — systemd supervision of the compose stack.
- `.github/workflows/ci.yaml` — build + colcon test inside the image on every
  commit; Phase-8 behavior sim-gates get added here as a second job.
- Escalation chain when something dies: node crash → Docker restarts container;
  node alive-but-silent → healthcheck fails → restart; repeated restarts →
  Commander sees missing HealthReports → FS_MODULE_UNHEALTHY failsafe → halt;
  Docker/compose itself dies → systemd brings it back.

## 6.6 Dev environment & test pyramid

Three ways to run the project, cheapest first:
1. **`make unit`** — pure-Python logic tests under `tests/` using rclpy/message
   shims (`tests/shims/`). No ROS required; runs in seconds anywhere (24 tests
   currently: Commander transitions/failsafes/leases, bridge watchdog, behavior
   engine lifecycle). CI runs these first and fails fast.
   *Rule:* shim message mirrors in `tests/shims/m20_msgs/` must stay in sync with
   `src/m20_msgs` — update both in the same PR (contracts process, §3).
2. **Container** — `make build` / `make test` builds the ROS 2 Humble image and
   runs colcon tests inside it; `make sim` launches sim bringup; `make up` runs
   the compose stack. Same image CI builds and the robot runs (ADR-011).
   `.devcontainer/` gives the identical environment inside VS Code.
3. **Hardware** — Phase 6+, always preceded by 1 and 2.

Keep node logic testable at level 1: side-effects behind injectable objects
(e.g. `VendorSDK`), state machines as plain methods, no logic hidden in spins.

## 6.7 Simulation stack — which sim, for what (resolves the Isaac confusion)

Two simulators, two different jobs. We are **not** running Isaac to develop the stack.

- **MuJoCo — our sim of record for locomotion/bridge work (Phase 1+).** The vendor
  ships a ready M20 MuJoCo model (`sdk_deploy/src/M20_sdk_deploy/M20_description/m20_mjcf`:
  `M20.xml`, `M20_stair.xml`, `scene.xml`) plus a ROS 2 sim node
  (`mujoco_simulation_ros2.py`) that speaks the SAME joint DDS interface as hardware
  (`/JOINTS_CMD` in; `/JOINTS_DATA` + `/IMU_DATA` out). Lightweight, runs on CPU, GUI via
  WSLg (`DISPLAY=:0` confirmed on the dev box). This is how we "see the robot" and how
  the bridge gets exercised in sim.
- **Isaac Lab / Isaac Sim 5.1 — TRAINING ONLY (Phase 8).** Used by `rl_training` to
  train RL policies that export to ONNX. Heavy; needs a large NVIDIA RTX GPU. Not
  required to build, run, or develop the autonomy stack. Do not stand it up until Phase 8.
- **Nav2 / velocity-level sim (Phase 3)** is a separate concern: it needs a `cmd_vel`
  wrapper on top of the joint sim (or the vendor's high-level velocity tier once
  confirmed — OQ-3). MuJoCo alone gives joint-level; Nav planning sits above it.

Bring-up order for "see a robot": (1) **DONE** — MuJoCo viewer loads `M20.xml` (18
bodies / 16 actuators / 22 DOF) in a GUI window via WSLg; reproducible with
`tools/setup_sim.sh` + `tools/view_sim.sh` (GUI prereq: `tools/install_gui_deps.sh`,
mesa). Physics runs, no controller yet → robot settles under gravity. (2) full
`mujoco_simulation_ros2.py` on the DDS bus so our nodes see `/JOINTS_DATA` + `/IMU_DATA`
and it stands/walks under the ONNX policy; (3) bridge drives it. Steps 2–3 require the
vendor import (`--build-arg IMPORT_VENDOR=true`) + `mujoco` + the bundled onnxruntime.

**`drdds` is available and buildable (correction, rev 10).** The vendor joint
interface messages live in `sdk_deploy/src/drdds` — a pure ROS 2 **message package**
(`.msg`/`.srv`, no C++/DDS lib), so it builds standalone with `colcon build
--packages-select drdds` in our Humble image (validated). Interface: `JointsDataCmd`
(16× position/velocity/torque/kp/kd/control_word — PD+ff), `JointsData` (16×
position/velocity/torque/temps), `ImuData` (rpy + gyro + accel), plus BatteryData,
GamepadData. The vendor sim node needs only `rclpy + mujoco + scipy + numpy + drdds`
(no onnxruntime/Eigen — those are for the C++ policy runner). So both our
`m20_locomotion_bridge` (speaks drdds, per ADR-015) and the vendor MuJoCo sim node can
run off-robot. This supersedes the earlier "drdds is robot-only" assumption.

**Test arena (planned).** Build a MuJoCo test world with obstacles — boxes/barriers,
scattered "random objects", and pipes — layered on the M20 scene. Serves multiple
phases from one asset: dynamic/static obstacle avoidance (Phase 3 Nav), rough-terrain
gait arbitration (Phase 5, alongside the existing `M20_stair.xml`), and the `jump_pipe`
behavior (Phase 8, which must clear a 0.3 m pipe). Author as a custom `scene_test.xml`
that includes `M20.xml` + obstacle bodies; keep the vendor model file untouched.

## 7. Vendor & ecosystem links

- Autoware Open AD Kit (deployment pattern reference only): https://autoware.org/open-ad-kit/
- Deep Robotics Lynx product page: https://www.deeprobotics.cn/en/index/lynx.html
- Deep Robotics GitHub org: https://github.com/DeepRoboticsLab
  - `rl_training` — RL sim-to-sim/sim-to-real, M20 supported, ROS 2 version
  - M20 lidar secondary development repo (Lightning-LM reproduction example)
  - Android SDK for Lynx M20 (connection, state, remote control)
- Nav2: https://docs.nav2.org
- robot_localization: https://github.com/cra-ros-pkg/robot_localization
- Isaac ROS (VSLAM, nvblox, DNN): https://nvidia-isaac-ros.github.io
- FAST-LIO2: https://github.com/hku-mars/FAST_LIO
- PX4 architecture (design inspiration only): https://docs.px4.io/main/en/concept/architecture

---

## 8. Glossary

- **Bridge** — `m20_locomotion_bridge`, the only node allowed to talk to the vendor SDK.
- **Vendor boundary** — the line below which code is Deep Robotics' responsibility.
- **Gait arbitration** — autonomous wheeled↔legged mode selection from terrain data.
- **Failsafe** — a Commander-owned monitor + reaction pair (e.g., comms loss → hold).
- **OQ / ADR** — open question / architecture decision record, tracked above.

---

## 9. Changelog

- **2026-07-11 (rev 36) — Closed the localization honesty gap (no more perfect info).**
  Replaced the sim's ground-truth `odom→base_link` TF with DEAD-RECKONED odometry that
  drifts, exactly like a real skid-steer: forward speed from wheel encoders × a CALIBRATED
  effective radius (`ODOM_R=0.072`, since the bridge's nominal 0.10 over-commands ~38%),
  heading from the gyro (true turn-rate + fixed bias 0.002 rad/s + per-sample noise). Ground
  truth moved to `/odom_true` (eval only, never consumed). slam_toolbox now corrects REAL
  drift: robot reaches the true goal within **0.82 m**; raw uncorrected odom ends **~5 m** off
  (proof overlays true/raw-odom/SLAM-estimate in `nav2_plan_a_result.png`, animation in
  `nav2_plan_a_run.gif`). **Debug journey (kept honest):** first attempt used wheel-diff yaw →
  wrong sign (skid slip); switched to gyro but integrated body-frame `qvel[5]` → +21°/turn
  error from body tilt during pivots → SLAM couldn't recover (aborted, 5 m off); fixed to
  track true turn-rate delta + bias/noise → 3°/turn, clean. Residual 0.82 m (vs 0.30 m on the
  old perfect localization) is the honest cost of drift in a sparse open field → motivates GPS
  fusion (robot_localization) next. Files: `tools/mujoco_sim.py` (odom model),
  `tools/nav/nav_logger.py` + `render_nav.py` (log/plot true vs drift vs estimate).
- **2026-07-11 (rev 35) — CHECKPOINT-02: Plan A (Nav2 + slam_toolbox) WORKING in sim.**
  Stood up the production 2D-nav stack on the MuJoCo oil & gas field and sent a real
  `NavigateToPose` goal. Pipeline: sim (`/scan` honest ray-cast ring at z=0.35 m, `/odom`,
  TF `odom→base_link`) → **slam_toolbox** (`/map` + TF `map→odom`, scan-matching) →
  **Nav2** (`navigation_launch.py`) with `nav2_params.yaml` = Regulated Pure Pursuit
  (`use_rotate_to_heading:true` — anti-tip, matches our validated turn-then-drive) + NavFn/A*
  planner + **rolling 40 m global costmap** (unknown=free, live obstacle layer, inflation) →
  bridge `/cmd_vel` → wheels. **Result:** autonomously drove ~7 m from (1.06, 0.23) to the
  goal (−6.0, −0.5), threading BETWEEN skid1 and skid2 (real costmap obstacle avoidance),
  final error **0.30 m** (< 0.35 m tol), "Goal succeeded". slam_toolbox built a real live
  map (tanks as arcs, skids/wellhead as boxes). Proof: `docs/media/nav2_plan_a_result.png`.
  **Bugs fixed en route:** (a) image-baked bridge was STALE (old sign/`kd`, no yaw feedback)
  → run repo `bridge_node.py` with `sdk_backend:=sim`; (b) default backend `stub` → no
  `/JOINTS_CMD` (robot inert) → pass `sdk_backend:=sim`; (c) planner "goal off global
  costmap" (costmap sized to incremental SLAM map) → rolling 40 m window. **Honest limits:**
  localization is ground-truth odom TF (SLAM `map→odom` ≈ identity) — dead-reckoned-odom
  drift is the next Plan-A step; 2D scan misses ground pipes (tops 0.30 m < sensor 0.35 m)
  → Plan B (3D). Repro: `tools/nav/bringup_plan_a.sh`. Next: honest odom drift, then Plan B.
- **2026-07-10 (rev 34) — CHECKPOINT.** All autonomy *components* proven individually in
  MuJoCo sim (with videos): stand-up, drive/turn/circle/fig-8 (armature + yaw-feedback),
  watchdog, custom EKF (wheel+IMU+GPS, live 0.48 m), honest 3D-LiDAR model, real
  slam_toolbox 2D map, A* global planner + stable follower, realistic outdoor oil & gas
  field. **Gap (proven, not hidden):** the integrated *real-time* loop on limited info
  fails without scan-matching (DIY EKF-placed map smears → bad routing). **Decision:
  build the PRODUCTION stack** — real slam_toolbox (scan-matching) + Nav2 (costmaps +
  planner) + a 3D perception/traversability layer (nvblox/grid_map) + FAST-LIVO2 (3D SLAM)
  + GPS-EKF + gait layer. This is the industry-standard legged-inspection pattern (Spot/
  ANYmal-class). Nav2/slam_toolbox = Steve Macenski / Open Navigation, ROS 2 standard,
  production-grade. Next: scope + stand up Nav2 + slam_toolbox on the sim.
- **2026-07-10 (rev 33)** — Honest integrated real-time loop attempt (no perfect info) —
  and the wall it hits. Built `tools/`+scratchpad `integrated_nav.py`: **limited-range
  LiDAR (7 m) → incremental occupancy map → EKF noisy pose (NOT ground truth) → A* replan
  on the partial map → follower**, with a robot's-eye-view render. Fixed real bugs found
  by tracing: (1) downward LiDAR beams marked the FLOOR as obstacles (fenced the robot in)
  → filter hits by height (z>0.12); (2) follower **spun in place** (target re-picked every
  tick + replan every 0.3 s) → **monotonic carrot + replan every 2 s**; robot then moves,
  upright (z=0.47), no tip. BUT it routes the **wrong way** and doesn't reach — root cause:
  the DIY loop places scans at the **noisy EKF pose with NO scan-matching**, so the map
  **smears** (obstacles in wrong cells → phantom barriers → bad A*). **Conclusion: real-time
  SLAM-nav needs the mature stack — slam_toolbox (scan-matching → clean map, already shown
  working rev 30) + Nav2 (costmaps + tuned planner) — not a DIY A*+EKF-placement.** That
  integration is the real Phase-3 path.
- **2026-07-10 (rev 32)** — Realistic outdoor field + autonomous nav (#3), honest sensor.
  User caught fabricated sensor: real M20 = **dual 96-line 3D LiDAR (360°×90°)**, not my
  flat 360-ray 2D. Modeled a **reduced-beam 3D LiDAR** (front fan × downward channels —
  sees GROUND PIPES a 2D slice misses; explicitly fewer beams than the 96-line). Built
  `tools/sim/oil_gas_field.xml` — outdoor pad: ground pipelines, storage tanks, pipe rack,
  wellhead, skids (no room). Autonomous nav: reactive potential-field got stuck at the
  long barrier pipe (local minimum) → built **A* global planner** on an occupancy grid
  (top-down raycast, inflated) + a follower. Two fixes: (1) robot **tipped** turning-while-
  driving (tall/narrow) → **decouple: turn-in-place then drive straight** (z stayed 0.47,
  no tip); (2) inflation rad=2 (rad=3 closed the route). Result: **reaches goal, final
  0.50 m, upright.** Honest scope: lightweight A*+follower = essence of Nav2; production
  = real Nav2 (costmaps/MPPI) + 3D SLAM (FAST-LIVO2). `run_sim.sh` M20_SCENE=oil_gas_field.xml.
- **2026-07-10 (rev 31)** — Richer mapping course + fuller SLAM, honest coverage limit.
  Added `tools/sim/mapping_arena.xml` — a bounded room (perimeter walls + interior divider
  + boxes + pipes) so the LiDAR gets continuous returns. Denser LiDAR (180→360 rays).
  `run_sim.sh` gains `M20_SCENE=<file>`. Re-ran real slam_toolbox with smooth driving
  (armature + yaw feedback): map grew to 236×177, occupied 135→485 — walls, divider,
  boxes, pipes all appear. BUT it's **not a pristine closed-room map**: a central/looping
  circle gives poor along-the-wall views → noisy, partial walls. **The real fix is
  coverage planning** (autonomous frontier exploration or a wall-following lawnmower) —
  a distinct capability, not just open-loop driving. Next: build coverage exploration for
  a clean map; then #2 SLAM→EKF (drift-free) and #3 Nav2. Videos of current driving rendered.
- **2026-07-10 (rev 30)** — **SLAM working (slam_toolbox on live /scan).** Added an
  `odom→base_link` TF broadcaster to `mujoco_sim.py` (needed for the TF tree). Config at
  `tools/slam/mapper_params.yaml` (async mapping, base_link/odom/map frames, /scan). Ran
  sim(arena)+slam_toolbox+bridge+test_driver(circle): slam_toolbox registered the LiDAR,
  scan-matched, and built a **2D occupancy grid** (`/map`, 170×100 @0.05 m) — free space
  + obstacle-box corners visible. Sparse (small boxes, 2D slice, limited coverage) but the
  LiDAR→SLAM→map pipeline is live. Next: drive fuller coverage / denser scan for a complete
  map; feed slam's map→odom (drift correction) or pose into the EKF; then Nav2 on the map.
- **2026-07-10 (rev 29)** — Closed the missing loop: **IMU yaw-rate feedback** (answers
  "is the wheel velocity open-loop / what are we doing wrong"). The control stack is
  pure-pursuit(closed on position) → skid-steer inverse-kinematics(**was open-loop
  feedforward**) → wheel-speed P(closed) → leg PD. The open-loop IK understeers because
  skid-steer slips on μ=1.0 floor (realistic). Fix: command w, measure IMU yaw rate,
  **PI-trim the wheel differential** (Kp=2, Ki=0.8) — closes the body-velocity loop.
  Proven offline on the TIGHT A=3 figure-8: cross-track **2.06→0.08 m (~25×)**; the
  open-loop giant-loop failure becomes precise tracking. Tradeoff: on tight curves it
  reintroduces leg wiggle (0.53→2.6) — **physical scrubbing** (slew-limiting didn't help;
  holding a tight curve on grippy floor needs hard scrub). Sweet spot = feedback + gentle
  curves = tight AND smooth; truly tight+smooth turns need leg-articulation/crab (Phase 8).
  Applied to bridge `SimVendorSDK` (subscribes /IMU_DATA). Friction confirmed μ=1.0
  slide (realistic rubber-on-concrete). Tools: yaw_fb_test / yaw_fb2.
- **2026-07-10 (rev 28)** — Diagnosed & fixed the "robot doesn't drive smoothly / leg
  wiggle in turns" (user caught it in the figure-8 video). Instrumented the run: the
  **wheels chatter ±20 rad/s** around a smooth command — the velocity gain (kd=5) with
  **near-zero wheel inertia** rings and torque-saturates into a limit cycle; that shakes
  the legs (wiggle worst when turn rate is high). Speed sweep: leg wiggle knees up at
  **v≥0.5** (0.35→1.7, 0.55→3.9, 1.0→5.3). Gain sweep showed a hard tradeoff (low kd=smooth
  but can't turn; high kd=turns but chatters). **Fix: add wheel `dof_armature`=0.03**
  (realistic motor/gearbox rotor inertia) + drive slower — leg wiggle **3.88→0.53 rad/s
  (7× smoother)** while still tracing the 8. Applied to `mujoco_sim.py`. Tools:
  fig8_diag/fig8_kd/fig8_arm/fig8_combo. Lesson: measure the signals (wheel cmd vs actual)
  — the wiggle was a wheel-control instability, not a leg-controller problem. **LiDAR is now a live ROS node**:
  mujoco_sim publishes `sensor_msgs/LaserScan` on `/scan` @10 Hz via `mj_ray` (180 rays,
  12 m); verified live seeing arena obstacles (returns land on the boxes). **Live movement
  tests on the bus**: new `tools/test_driver.py` (MODE=circle|figure8|straight → /cmd_vel);
  ran circle (clean ~2.5 m loop) and figure8 (S-curve, both turn dirs) live via sim+bridge
  +test_driver, captured from /odom. Gotcha logged: LiDAR only sees obstacles when sim runs
  with `M20_ARENA=1` (bare M20.xml has none). Remaining for #3: SLAM (slam_toolbox/FAST-LIVO2
  consuming /scan → pose → EKF) for drift-free cm-level localization. Open: closed figure-8
  (open-loop drifts — needs trajectory-based), legged motion (Phase 8).
  #2: retuned EKF `Q` 0.6→0.2 (accurate + calmer; offline 0.28 m bounded). Honest limit:
  civilian GPS (±0.8 m @5 Hz) has a jitter floor — EKF makes it accurate, not perfectly
  smooth; true smooth+precise comes from LiDAR. #3 started: `tools/lidar_sim.py` casts a
  360-ray horizontal LiDAR slice via `mujoco.mj_ray` in the arena and VISUALIZES it
  (top-down scan traces obstacle outlines + polar range view) — the M20 has dual 3D
  LiDAR; this is the 2D proof-of-concept. Next for #3: wrap as a live ROS node publishing
  sensor_msgs/LaserScan, then SLAM (FAST-LIVO2, OQ-2) → feed pose into the EKF for
  drift-free, smooth, cm-level localization. (Legged motion still untouched — Phase 8.) ("not
  faking it" paid off). Running the REAL ROS chain (sim→estimator→bridge→trajectory
  follower on /odom_filtered) exposed a bug the offline script hid: EKF `Q` was tiny →
  low GPS gain → over-trusts dead-reckoning → **2.8 m drift live** (offline claimed 0.3 m).
  Fix: retuned `Q=diag([0.6,0.6,0.3])²`, `R=0.6²`, `P0=0.5` in tools/estimator.py.
  Re-verified LIVE: EKF error **mean 1.41→0.48 m, final 2.82→0.05 m**. Remaining: estimate
  is accurate but JITTERY (GPS noise passes through; needs middle-ground Q / more IMU
  weighting). Ops lessons: (1) full live chain OOMs the 5 GB WSL cap (exit 137) →
  raised to **memory=6GB** (swap 3GB) in .wslconfig; (2) `docker run -d` detached nodes
  die on this box — run nodes as foreground/harness-tracked tasks instead.
- **2026-07-09 (rev 24)** — Trajectory generation added (was missing — user caught it).
  The follower had been reactive **point-to-point** control (steer at the current WP,
  switch when close) — no planned path → occasional odd moves. Added a real motion layer:
  **Catmull-Rom spline** through the waypoints + **pure-pursuit tracker** (lookahead).
  Folded into the deployable `tools/course_follower.py`; benchmarked in
  `tools/traj_benchmark.py` on a **longer 8-WP loop**: smooth, **steady cross-track
  0.45 m** (Ld=0.6, v=0.5). Lesson: pure-pursuit turn *radius* is set by lookahead, not
  speed — slowing on curves shrinks the turn command and made it diverge; keep v constant,
  tune Ld. Estimation data recap for reference: dead-reckoning 2.4 m drift vs EKF 0.3 m.
- **2026-07-09 (rev 23)** — Live proof: autonomous loop on REAL estimation. Ran the full
  chain on the bus — sim → estimator(EKF) → /odom_filtered → follower → cmd_vel → bridge
  → sim — with the follower navigating on `/odom_filtered` (NOT ground truth): **COURSE
  COMPLETE, all 5 WPs**. Accuracy check: at WP5 the true pose was ~1.77 m from the
  waypoint — i.e., navigating on a GPS-fused estimate lands within ~GPS-level (~1–2 m,
  GPS noise ±0.8 m + live-EKF timing looseness vs offline 0.3 m). Takeaway: GPS bounds
  drift and the loop works, but GPS-only precision is meter-level → **#2 LiDAR SLAM is
  the sub-meter/indoor precision upgrade** (feeds the same EKF). Est. arc: EKF ✅ (drift
  bounded, loop proven live); precision pending LiDAR.
- **2026-07-09 (rev 22)** — #4 estimation SOLVED (EKF). Implemented a proper 2D EKF
  (state x,y,yaw; predict from wheel speed + IMU gyro; correct from GPS). Validated in
  `tools/ekf_benchmark.py` over 3 laps: dead-reckoning drift **2.4 m and climbing** →
  **EKF bounded ~0.3–0.5 m, smooth, no growth**. Deployable: `tools/estimator.py`
  (subscribes /JOINTS_DATA + /IMU_DATA + /gps → publishes /odom_filtered, map→base_link).
  Sim now publishes simulated **/gps** (noisy absolute position ~5 Hz). So drift-free
  localization works for the GPS/outdoor case today. Remaining for full coverage:
  **LiDAR SLAM (#2)** for cm precision + GPS-denied/indoor (FAST-LIVO2 pinned, OQ-2) —
  it becomes another input to this same EKF. Production swap-in: robot_localization.
- **2026-07-09 (rev 21)** — GPS benchmarked (M20 has GPS; sim didn't simulate it — now
  modeled in analysis). 3-lap run, wheel+IMU dead-reckoning vs GPS-fused (sim GPS =
  truth + 0.8 m noise @5 Hz, complementary correction): **dead-reckoning drift grows
  unbounded → 2.4 m and climbing; GPS-fused stays bounded ~0.5 m mean (flat) but jittery.**
  Conclusion: GPS is the absolute reference that BOUNDS drift (dead-reckoning can't);
  raw GPS is noisy, so the win is FUSION — GPS bounds + wheel/IMU smooths = an EKF
  (`robot_localization` + `navsat_transform`). So GPS makes #1 (EKF) actually drift-free,
  and complements #2 (LiDAR adds cm precision + indoor coverage). Estimation plan now:
  EKF fusing wheel+IMU+GPS → drift-free outdoors; add LiDAR SLAM for precision/indoors.
  Add "simulate GPS (NavSatFix/odom) in mujoco_sim" to the perception TODO (ties OQ-2).
- **2026-07-09 (rev 20)** — #4 estimation baseline. Measured wheel+IMU dead-reckoning
  vs ground truth over the waypoint course: **final drift 1.27 m** (~8% of a ~15 m path)
  from wheel slip + integration error. Insight (drives the plan): dead-reckoning always
  drifts (no absolute reference); an EKF (robot_localization) fusing wheel+IMU smooths
  but won't eliminate it — **drift-free localization needs the LiDAR → SLAM** (FAST-LIO2
  / vendor, OQ-2), which means simulating the dual 96-line LiDAR in MuJoCo (rangefinders)
  as part of perception. So estimation = (a) EKF to fuse what we have, then (b) LiDAR+SLAM
  for the real fix. Waypoint markers WP1–WP5 also baked into scene_test.xml (video shows them).
- **2026-07-09 (rev 19)** — #2 live ROS course + #3 waypoint markers. Sim now publishes
  ground-truth `/odom` (nav_msgs/Odometry from the MuJoCo base = "perfect estimator").
  New `tools/course_follower.py`: reads `/odom`, drives `/cmd_vel` (blended controller)
  through the bridge → **live autonomous loop verified on the ROS bus: WP1→…→WP5,
  COURSE COMPLETE** (follower → cmd_vel → bridge → JointsData → sim → odom → follower).
  #3: 5 waypoint markers baked into `tools/sim/scene_test.xml` (WP1–WP5, match the
  follower). Ran lean (headless sim + commander + bridge + follower) to stay under the
  5 GB WSL cap. Next: #4 Phase-2 estimation — replace perfect `/odom` with a real EKF
  (robot_localization) fusing sim odom + IMU, measured against this baseline.
- **2026-07-09 (rev 18)** — #1 movement polish done. The bang-bang "turn-then-drive"
  waypoint controller caused the small-turn wiggle (mode-switch chatter + laggy heading
  correction). Replaced with a **blended controller** (turn continuously; forward speed
  scaled by cos(heading error)) — no mode switch → smooth AND accurate: **5/5 waypoints,
  clean paths**, circle + figure-8 intact. `tools/benchmark.py` uses it. Recorded a
  video (stand-up → course, 958 frames, imageio). Next: #2 live ROS run with ground-
  truth `/odom` from the sim (perfect estimator) before real estimation (#4).
- **2026-07-09 (rev 17)** — Turning fixed + movement benchmark passes (resolves OQ-16).
  Turning was a control bug, not physics (see OQ-16). Three cmd_vel→wheel fixes in the
  bridge `SimVendorSDK` (mirrored in the benchmark): wheel velocity **kd 0.6→5.0**,
  **forward sign** (+vx was −x), **turn sign** (+wz now CCW/ROS). Promoted the course
  benchmark to `tools/benchmark.py` (+ `make benchmark`): stand up → 5 marked waypoints
  + full circle + figure-8 on ground-truth pose. Results: **waypoints 5/5**, circle
  closes (~1.9 m gap / ~5 m loop), figure-8 both directions. This is the movement
  regression test going forward. Stack image rebuilt to carry the bridge fixes.
- **2026-07-08 (rev 16)** — Benchmark harness + sensor inventory + spaced arena.
  Built a movement benchmark (`tools/` scratch → to be promoted): stand up, then run
  courses (go-to-target, circle, figure-8) on a clean field, logging **ground-truth
  pose** (MuJoCo = our "GPS/mocap" for benchmarking); outputs trajectory plots +
  metrics. **CRITICAL finding → OQ-16:** turning via wheel skid-steer is far too weak
  (go-to-target failed, drove 12.9 m away; circle/figure-8 barely curve) and is
  friction-independent — the M20 has no wheel-yaw DOF, so real turning needs the RL
  policy or a stepping gait. Sensors: real M20 has IMU/lidar×2/cameras/GPS; **sim
  today publishes IMU + joints only** (lidar/camera/GPS not simulated yet — added when
  perception/Phase-3 lands). Arena spaced out + target marker (scene_test.xml).
- **2026-07-08 (rev 15)** — First behavior + turning + arena. **`standup` behavior
  plugin** added (`m20_behaviors/behaviors/standup.py`): scripted smoothstep rise
  crouch(~0.27 m)→stand(~0.46 m), TIER_VENDOR (no lease), drdds lazy-imported (24/24
  tests still pass). Behavior engine already executes it (auto-discovered). Validated
  in a self-contained sim: **crouch → stand → drive → turn** in the obstacle/pipe
  arena (`tools/sim/scene_test.xml`, wired into setup_sim.sh + `M20_ARENA=1`).
  **Turning finding:** the wheeled quadruped skid-steers only weakly on gentle arcs
  (yaw ~2°); crisp turns need OPPOSITE-side wheels (in-place: ~25° in 4 s). The
  bridge's `angular.z`→differential mapping is correct for this; tight/arc turns may
  need tuning or leg-articulation later. Demo run self-contained (offline MuJoCo) to
  avoid the OOM seen running full stack+GUI sim on the 5 GB WSL cap.
- **2026-07-08 (rev 14)** — Vendor stand-up mapped (informs the behavior layer). The
  M20 SDK's `qw_state_machine` = `idle → standup → rl_control` (walk) + `liedown` /
  `joint_damping`. **`standup_state` is a SCRIPTED cubic-spline trajectory** (folded →
  IK stand pose over ~2 s, then raise to `stand_height`=0.48 m over ~2 s; kp≈200 kd≈4,
  wheels locked) — NOT the RL policy. Our empirical stance (0.458 m) ≈ their IK target.
  *Design consequence:* standup/liedown/self_right are TIER_VENDOR **behavior plugins**
  (m20_behaviors, ADR-009) — standup is the ideal first behavior (pure position interp,
  no joint-policy risk), ahead of camera_scan; `rl_control` (walking) stays Phase 8
  (TIER_POLICY). Vendor SafeController ↔ our Commander veto/failsafes. Plan: sim should
  start FOLDED and run a standup behavior to rise (faithful), vs today's cheat of
  spawning already-standing. Also planned next: skid-steer turning demo + obstacle/pipe
  test arena (scene_test.xml) for Phase-3 nav.
- **2026-07-08 (rev 13)** — Two fixes from watching it run: (1) **Standing pose** — the
  robot was dragging its belly because the vendor `JOINT_INIT` (knee≈2.76≈limit) is a
  FOLDED start pose, not a stand. Swept leg angles empirically → standing stance
  `hipy≈∓0.7, knee≈±1.4` (body ~0.46 m, level); now stands and drives on its wheels
  without dragging. Used in mujoco_sim / bridge / drive test. (2) **QoS bug** — the
  bridge subscribed `/m20/mode` VOLATILE while the Commander latches it
  (TRANSIENT_LOCAL) and only republishes on change, so a restarted bridge never saw
  "armed" and ignored `/cmd_vel`. Fixed: bridge now subscribes with matching
  TRANSIENT_LOCAL QoS → a (re)started bridge auto-arms from the latched mode.
- **2026-07-08 (rev 12)** — **Phase 1 chain complete in sim.** Full path proven:
  operator `/cmd_vel` → Commander (SetMode→TELEOP arms; veto/gating) → bridge → vendor
  (sim) → M20 drives. `m20_locomotion_bridge` gained a `sim` backend (`SimVendorSDK`,
  param `sdk_backend:=sim`) that skid-steers the wheels + holds stance via drdds
  `JointsDataCmd` (kept out of the ROS-free tests via lazy import; 24/24 still pass).
  Watchdog verified: killing `/cmd_vel` → `HALT: cmd_vel timeout (0.53s>0.5s)` →
  wheels to 0 (tighten `cmd_timeout_s` to 0.4 for the <500 ms exit criterion).
  Caveats: locomotion is PD stance-hold + wheel skid-steer on flat ground (NOT the
  vendor ONNX walking policy — legged gaits are Phase 8); IMU accel is a placeholder;
  bridge doesn't publish odom yet. Disk: recovered + capped WSL swap (.wslconfig);
  freed ~13 GB on C: (Boost + Downloads dupes/installers); external drive still TODO.
- **2026-07-08 (rev 11)** — Phase 1 sim milestone: the M20 **stands and drives in
  MuJoCo on the ROS 2 bus**. New `tools/mujoco_sim.py` (our sim node: pub JointsData/
  ImuData @200 Hz, sub JointsDataCmd, PD+ff control law), `docker/Dockerfile.sim`
  (`m20_sim` = stack image + mesa + mujoco), `tools/run_sim.sh` (GUI→WSLg via
  /dev/dxg + /usr/lib/wsl mounts), `tools/sim_drive_test.py` (spin wheels). Key fixes
  found by testing: (a) `qpos=0` is NOT the load-bearing stance → hip motors saturate;
  correct stance is the vendor JOINT_INIT (`hipx±0.438, hipy∓1.16, knee±2.76`), now
  used — robot holds stance firmly and all 4 wheels drive. Remaining Phase-1: wire
  `m20_locomotion_bridge` (/cmd_vel→JointsDataCmd) + the command-timeout watchdog test.
  **DISK WARNING:** dev box is a single 236 GB C: drive that ran to **0 free** during
  image builds (WSL ext4.vhdx grew to 15 GB). Recovered to ~9.5 GB via `wsl --shutdown`.
  This workflow is disk-starved here — see the operator notes; needs freeing C: space or
  a bigger disk before heavy further builds.
- **2026-07-08 (rev 10)** — Correction + Phase-1 unblock: `drdds` (vendor joint
  interface) is a pure ROS 2 msg package in `sdk_deploy/src/drdds`, builds standalone
  in our Humble image (validated) — supersedes the earlier "robot-only" assumption.
  Interface mapped (JointsDataCmd/JointsData/ImuData). Clears the path to run the vendor
  MuJoCo sim node off-robot and to have the bridge speak drdds (ADR-015). §6.7 updated.
- **2026-07-08 (rev 9)** — Sim preview stood up + made reproducible: MuJoCo + the
  pinned vendor M20 model render on the dev box (offscreen EGL image + live WSLg
  window), after resolving a bare-WSL "no OpenGL libs" issue (mesa install). Ad-hoc
  setup folded into repo tooling: `tools/setup_sim.sh` (mujoco + pinned model, no sudo),
  `tools/install_gui_deps.sh` (mesa, one-time sudo), `tools/view_sim.sh` (viewer),
  Makefile `sim-setup`/`sim-view`/`gui-deps`, README "Simulation preview" section.
  Added the test-arena plan to §6.7 (obstacles/pipes for Phase 3/5/8). Next: Phase 1
  step 2 — ROS-integrated MuJoCo sim (ONNX policy) so the M20 walks on the bus.
- **2026-07-08 (rev 8)** — Operability + docs: README rewritten with a "Running the
  system (Windows+WSL)" section (build contents, up/down/status/logs, full shutdown),
  a ros2-CLI introspection helper (`tools/ros2.sh` — runs ros2 against the running
  stack; fixes "can't check topic list"), and the Foxglove operator view. Added §6.7
  Simulation stack: MuJoCo is our sim of record (Phase 1+); Isaac is training-only
  (Phase 8) — resolves the "are we using Isaac?" confusion. Confirmed Humble stays
  (ADR-007): robot's onboard Foxy is behind the bridge/vendor boundary, so our whole
  stack is Humble. WSLg GUI display confirmed (`DISPLAY=:0`) for sim visualization.
- **2026-07-08 (rev 7)** — First full integration run: image built and stack brought
  up in docker compose on a dev box (WSL2 + Docker Engine). Core node logic proved
  sound; fixed three deployment-layer bugs found only at runtime: (1) container
  healthcheck ran under /bin/sh without sourcing ROS (rclpy ImportError) → now
  `bash -c 'source … && python3 /healthcheck.py'`; (2) station_bridge crash-looped on
  a malformed `robot_id=` launch arg AND was missing foxglove_bridge/rosbridge_suite
  in the image → arg now conditional `:=`, packages added to Dockerfile; (3) Commander
  never published its own HealthReport → added a 1 Hz heartbeat (HealthReport contract).
  Result: 5/5 services up, 4 healthy + station serving Foxglove(:8765)/rosbridge(:9090).
- **2026-07-07 (rev 6)** — Vendor SDK mapped from public GitHub (no vendor contact
  needed): ADR-007 DECIDED (Humble/Ubuntu 22.04; robot onboard Foxy), ADR-015 added
  (public M20 SDK is joint-level DDS `drdds` + ONNX + MuJoCo; velocity/gait tier is
  onboard-only). OQ-1/OQ-3 partially resolved, OQ-5 resolved. `tools/vendor.repos`
  populated with real DeepRoboticsLab repos pinned to commit hashes (sdk_deploy,
  rl_training, deep_robotics_model, fast-livo2, lightning-lm).
- **2026-07-07 (rev 5)** — Dev environment: tests/ with rclpy + message shims and
  24 passing unit tests (Commander, bridge, behavior engine), Makefile targets,
  .devcontainer, CI fast unit job gating the container job. Section 6.6 added.
- **2026-07-07 (rev 4)** — Station + fleet-readiness: ADR-013 (Foxglove-first
  operator station, rosbridge as the single external gateway), ADR-014 (fleet-ready
  rules: robot_id namespacing, gateway-only external access, missions as fleet
  integration point). New package m20_station. Phases 1.5 and 10 added. OQ-14/15
  opened.
- **2026-07-07 (rev 3)** — Deployment layer added: ADR-011 (containerized CI-gated
  deploys, Autoware modules rejected, lidar-tracking borrow tracked as OQ-12) and
  ADR-012 (compose + systemd + 3-layer watchdogs; k3s deferred to fleet scale).
  docker/, deploy/, CI workflow scaffolded. OQ-12/13 opened. Section 6.5 added.
- **2026-07-07 (rev 2)** — Behavior layer designed in: ADR-008/009/010, new package
  `m20_behaviors` (engine + camera_scan/self_right/jump_pipe/three_wheel plugin
  stubs), ControlAuthority lease contracts, Commander lease service implemented,
  Phases 5.5/8/9 added, OQ-8…11 opened.
- **2026-07-07** — Document created. Workspace scaffolded, contracts v0 drafted,
  ADR-001…007 recorded, roadmap phases 0–7 defined, OQ-1…7 opened. (Phase 0)
