# Onboarding — where we are, how to work here, how to get good at ROS 2

Read this first. It orients you in about twenty minutes, then points at the
right document for whatever you are actually doing.

---

## Part 1 — Where we are

**Authoritative source: `docs/ROADMAP.md`.** It has the current milestone and
the gate that closes it. If this section and the roadmap disagree, the roadmap
wins.

### The project in one paragraph
An autonomy stack for a wheel-legged quadruped (Deep Robotics Lynx M20 Pro)
doing oil & gas site inspection. ROS 2 Humble in Docker, MuJoCo as the
simulator of record, architected after PX4: single-responsibility nodes on a
message bus, contracts before code, a paranoid commander owning modes and
failsafes, Nav2 as the navigator, and an AI/mission layer that can only
*request* — never override — safety.

### What works today, with evidence
- **Navigation**: 10/10 single-goal gate, mean 0.33 m true error, on realistic
  sensors and a QA'd map (`checkpoint-05-anchored-map`).
- **Site map pipeline**: build → serialize → landmark quality gate. Shipped map
  scores 0.23 m worst-landmark (`maps/oil_gas_field.score.txt`).
- **Commander**: modes, arming, failsafes; tip-over drill passed (L1.6).
- **Behaviors**: engine plus `standup` (commander-gated rise from folded) and
  `camera_scan` (finds and centres a target — Phase 5.5 exit test passed at
  1.9° from true bearing).
- **Mission layer**: action server accepting waypoint lists; `tools/dev/m20` CLI.
- **Operator station**: Foxglove live view, click-to-drive.
- **Drift bounding**: anchor guardian with measured thresholds.

### What is in flight
**Milestone M1 (Site Tour)** — the five-location patrol gate. Best result so
far 4/5 legs at mean 0.27 m, including the first east-field success (0.11 m).
Every leg has individually passed. Two infrastructure blockers remain: the
commander container restarting silently mid-patrol, and the tip-over failsafe
firing on aggressive pivots that graze the 35° limit (needs a duration filter).

### The honest ladder ahead
`M1 Site Tour → M2 Robustness → M3 Deploy pipeline → M4 Hardware bench →
M5 Outdoor`. Outdoor testing is three gates away and the reasons are written
down, not guessed.

---

## Part 2 — The documents, in reading order

| Order | Document | Read it when |
|---|---|---|
| 1 | `docs/ONBOARDING.md` | you are here |
| 2 | `docs/ROADMAP.md` | every session start — current milestone + gate |
| 3 | `docs/RUN_GUIDE.md` | you want to run something |
| 4 | `docs/NAV2_ARCHITECTURE.md` | you need to understand the navigation stack |
| 5 | `docs/LESSONS_LEARNED.md` | before debugging anything — odds are it is here |
| 6 | `docs/ENGINEERING_RULES.md` | before writing any script or node |
| 7 | `docs/SECOND_BRAIN.md` | the full history: ADRs, phases, changelog to rev 44 |
| 8 | `docs/TEST_PLAN.md` | the verification ladder L1/L2/L3 + session ledgers |
| 9 | `docs/NAV2_SETUP_RUNBOOK.md` | standing a nav stack up from zero (any robot) |
| 10 | `docs/SIM_TO_REAL.md` | deployment, Jetson, platform flags |
| 11 | `docs/STATION.md` | operator station setup |

`SECOND_BRAIN.md` is the long-term memory: architecture decisions (ADRs), open
questions (OQs), the ten-phase plan, and a dated changelog. Its rule is *if it
isn't in this document, it didn't happen* — so when you land something
meaningful, add a changelog entry.

---

## Part 3 — How to work in this codebase

### The loop
1. **Read `ROADMAP.md`.** Work happens inside the current milestone. Anything
   else goes under "Later" — this rule exists because the project once
   wandered productively but unfocusedly for a week.
2. **Check `LESSONS_LEARNED.md`** for your symptom before investigating.
3. **Make the smallest change** that could work, behind a flag if it touches
   the known-good path.
4. **Smoke-test it** on tiny input before putting it in a long chain.
5. **Run the relevant gate** on a quiet box (`uptime` < 1.0).
6. **Commit with the evidence in the message** — numbers, not adjectives.
7. **Update the ledger** (`TEST_PLAN.md` session notes, `SECOND_BRAIN.md`
   changelog) when something meaningful lands.

### Non-negotiables
- **Never fake a result.** If a run failed, say so with the output. Every
  video in `docs/media/` replays logged data; none re-simulate.
- **Score against ground truth**, never the robot's own estimate.
- **One implementation per function.** Extend `bringup_plan_a.sh` with a flag;
  do not fork it. A forked copy once diverged and produced silent failures.
- **Flag-guard new behavior**, defaulting off, until its gate passes.
- **Rebuild the image after touching `src/`** — bringup will refuse stale
  safety code, and it is right to.
- **Merge on gate pass only.** A branch stays unmerged until its bar is met.

### Where changes go
| Changing… | Edit | Rebuild image? |
|---|---|---|
| Nav2 tuning | `src/m20_navigation/config/nav2_params.yaml` | no (staged at bringup) |
| Node logic | `src/m20_*/...` | **yes** |
| Scripts, tools, renderers | `tools/...` | no |
| SLAM config | `tools/slam/*.yaml` | no |
| Message contracts | `src/m20_msgs/` + all consumers, same commit | **yes** |

---

## Part 4 — Becoming a ROS 2 professional (Python and C++)

The fastest path is to read the concept, then read this repo's implementation
of it, then change it and watch a gate react.

### Tier 1 — Fundamentals
| Concept | What to know | See it here |
|---|---|---|
| Node | The unit of computation; one responsibility | `src/m20_commander/m20_commander/commander_node.py` |
| Topic (pub/sub) | Anonymous many-to-many streaming | `tools/mujoco_sim.py` (publishes sensors) |
| Message | A typed contract, versioned deliberately | `src/m20_msgs/msg/RobotMode.msg` |
| Parameters | Runtime config, never hardcoded | `nav2_params.yaml`; `sdk_backend` in the bridge |
| Launch | Composing a system, not a script | `src/m20_station/launch/station_bridge.launch.py` |
| Workspace/colcon | Build and overlay model | `Makefile`, `docker/Dockerfile` |

**Exercise:** add a field to a message, update every consumer in one commit,
rebuild, and watch the contract propagate.

### Tier 2 — What separates working from professional
| Concept | The thing people get wrong | See it here |
|---|---|---|
| **QoS** | Reliability/durability must match or you get *silence* | `nav_logger.py` (BEST_EFFORT for `/scan`, TRANSIENT_LOCAL for `/map`) |
| **TF2** | One publisher per edge; `map→odom→base_link` semantics | `anchor_guardian.py` (lookups), `mujoco_sim.py` (odom TF) |
| **Executors & callback groups** | Callbacks block each other by default | `m20_behaviors/engine.py` timer + subs |
| **Actions** | For long, cancellable, feedback-producing work | `m20_missions/mission_server.py` — a *server* to the CLI and a *client* to Nav2 |
| **Lifecycle nodes** | Deterministic startup ordering | Nav2's `lifecycle_manager` |
| **Time** | `use_sim_time`, and never mixing clocks | `docs/SIM_TO_REAL.md` §4 |

**Exercise:** deliberately break a QoS match and observe how *quiet* the
failure is. That single experiment will save you a day later.

### Tier 3 — System design
- **Contracts before code** (ADR-004) — design `m20_msgs` first; nodes build
  against it independently.
- **Hard boundaries** — exactly one node touches the vendor SDK. That is why
  hardware migration is a backend swap.
- **Supervisor with veto** — safety is a separate node that can refuse, not a
  flag inside the mover.
- **Plugins over forks** — Nav2 is customized through parameters, plugin
  selection, and a behavior tree; we have never forked it.
- **Test at three levels** — component (L1), subsystem (L2), integration (L3).

### The C++ path
Roughly 90% of the compute here is already C++ (Nav2, slam_toolbox, DDS,
pointcloud conversion). Our Python is the orchestration crust. Port order, and
why:

| Priority | Node | Why C++ | Effort |
|---|---|---|---|
| **P1** | `m20_locomotion_bridge` | 200 Hz joint streaming on hardware; GC pauses are unacceptable jitter | 1–2 weeks |
| **P2** | `m20_commander` | Safety code wants static typing, deterministic destruction, lifecycle semantics | ~1 week |
| **P3** | `m20_behaviors` | Only when TIER_POLICY lands (ONNX runtime + `pluginlib`) | 1–2 weeks |
| **P4** | `m20_missions` | Probably never — event orchestration at human timescales is Python's home | — |
| — | `tools/estimator.py` | **Delete, don't port**: replace with `robot_localization` (already C++) | — |

Each port is a drop-in behind identical topics: port one node, run the L3.2
gate, nothing else notices. That is what contracts-first bought.

**C++ specifics worth learning in order:** `rclcpp::Node` and the publisher/
subscription API → parameters and callbacks → `tf2_ros` buffer/listener →
`rclcpp_action` servers and clients → lifecycle nodes → `pluginlib` →
composition and intra-process comms (the real performance win: load bridge +
conversion + Nav2 into one process and eliminate serialization on the hot
path).

**Python specifics:** `rclpy` mirrors the same concepts, so learn the *ideas*
once. Python-specific competence is mostly about executors, callback groups,
and knowing when the GIL makes a node unsuitable for the job.

### Recommended external sources
- Nav2 documentation (`docs.nav2.org`) — configuration guide and plugin list.
- REP-105 (coordinate frames) and REP-103 (units/conventions). Short, and they
  settle arguments.
- The ROS 2 tutorials for the *mechanics*; this repo for the *judgment*.

### How to prove to yourself you have it
You can call yourself competent here when you can:
1. Explain why `odom → base_link` must not jump — and what breaks if it does.
2. Diagnose a silent topic in under a minute (QoS, then lifecycle, then TF).
3. Add a behavior plugin that passes its exit test against ground truth.
4. Change a Nav2 parameter and predict which gate metric moves, before running it.
5. Take a failing gate and produce a root cause with a measurement, not a guess.

---

## Part 5 — First day checklist

```bash
# 1. Orient
cat docs/ROADMAP.md

# 2. Prove the toolchain works (no ROS needed, seconds)
make unit

# 3. Bring the stack up and watch it drive
M20_STATIC_MAP=maps/oil_gas_field M20_NO_GOAL=1 bash tools/nav/bringup_plan_a.sh
tools/dev/m20 goto -6 -0.5

# 4. Watch it live — Foxglove WebSocket on ws://localhost:8765
#    layout: src/m20_station/config/m20_operator_layout.json

# 5. Tear down
bash tools/nav/kill_stack.sh
```

Then read `LESSONS_LEARNED.md` end to end. It is the fastest way to inherit
several weeks of debugging without paying for it.
