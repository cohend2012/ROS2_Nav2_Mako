# M20 Autonomy — Test Plan

Living document. Every system we build gets a test here **before** we call it done, with a
concrete metric, a pass threshold, and a repeatable command. Order of build-out:
**D (harden Plan A) → A (GPS/EKF) → B (3D perception) → C (missions).**

## Principles (non-negotiable)
1. **Verify on the real signal, never perfect info.** Score against ground truth
   (`/odom_true`), but the stack under test only ever consumes the honest signals
   (drifting `/odom`, live `/scan`, GPS). Ground truth is for the *scorecard*, not the robot.
2. **Every claim has an artifact.** A number (metric), a plot, or a video — reproducible
   from a committed script. No "it works" without evidence.
3. **Report failures.** Success rate is a number, not a vibe. If 3/10 runs fail, we say 3/10.
4. **One command to reproduce.** Each test runs from a committed script.

## Test levels

### L1 — Component checks (fast, deterministic)
| # | System | Test / script | Metric | Pass |
|---|--------|---------------|--------|------|
| L1.1 | Sim I/O | topics + rates up | `/scan ~10 Hz`, `/odom /IMU /JOINTS` present | all present |
| L1.2 | Drive signs | `tools/nav/drive_check.py` | fwd → +x, turn → +CCW | correct signs |
| L1.3 | Honest odom | `tools/nav/odom_drift_check.py` | fwd odom vs true err; yaw err/turn | <0.1 m fwd; <5°/turn |
| L1.4 | Bridge arming | set_mode + cmd_vel | moves only when armed; halts on veto | enforced |
| L1.5 | Watchdog | stop cmd_vel | zero-velocity within `cmd_timeout_s` | halts |

### L2 — Subsystem checks
| # | System | Test | Metric | Pass |
|---|--------|------|--------|------|
| L2.1 | SLAM builds map | run + `nav_logger` map | occupied cells match known obstacles | tanks/skids present |
| L2.2 | SLAM corrects drift | compare `/odom` vs `map→base_link` vs `/odom_true` | est-vs-true ≪ raw-odom-vs-true | est error < ½ raw |
| L2.3 | Planner | send goal | valid global path around obstacles | path clears footprint |
| L2.4 | Controller | follow path | no tip, no collision on a clear run | reaches within tol |

### L3 — Integration (full stack, the headline metric)
| # | Scenario | Script | Metric | Pass (target) |
|---|----------|--------|--------|---------------|
| L3.1 | Single go-to-goal | `bringup_plan_a.sh` + `nav_logger` | **true error to goal** | < 0.5 m |
| L3.2 | **Reliability batch** | `batch_nav_test.py` (D) — N fresh runs | **success rate**, mean/95p true error | ≥ 8/10, mean < 0.8 m |
| L3.3 | Obstacle avoidance | goal behind a skid | min clearance to obstacle | > 0.15 m |
| L3.4 | Replanning | obstacle appears mid-run (C) | recovers, still reaches | reaches |
| L3.5 | Multi-waypoint patrol | mission of 4 goals (C) | all reached in order | 4/4 |

## Per-phase acceptance

- **D — Harden Plan A.** Fix the stale-state aborts (always restart Nav2 with the sim;
  bake into bringup). Deliver `batch_nav_test.py` running N fresh runs → success rate +
  true-error distribution. **Accept:** L3.2 ≥ 8/10, mean true error < 0.8 m. Add the
  LiDAR-visualized video as a standing artifact.
- **A — GPS + IMU + wheel EKF (robot_localization).** **Accept:** with GPS fused, L3.2
  success ≥ 9/10 and mean true error < 0.4 m (GPS bounds SLAM drift); EKF pose vs
  `/odom_true` RMSE < 0.3 m over a run.
- **B — 3D perception / traversability.** **Accept:** the ground pipelines (tops 0.30 m,
  invisible to the 2D scan) appear as obstacles in the traversability layer; a goal across a
  pipe run is planned *around* it, not *through* it.
- **C — Missions / robustness.** **Accept:** L3.4 + L3.5 pass; recovery behaviors handle a
  blocked path without driving into obstacles.

## Known operational gotchas (regression guards)
- **Restart Nav2 whenever the sim restarts.** Reusing Nav2 across a sim restart leaves stale
  costmap/TF state (robot "teleports") → repeated collision/TF aborts. Full-stack fresh
  restart is reliable. (Root cause of the 3 failed runs on 2026-07-12.)
- **One bridge only.** Multiple `bridge_node` instances fight over `/JOINTS_CMD`. Kill stale
  ones; after `kill -9`, wait for DDS to reap phantom publishers before trusting counts.
- **`sdk_backend:=sim`** or the bridge is a no-op stub (no `/JOINTS_CMD`).

## Current status (2026-07-12)
- L1.1–L1.3 ✅ · L2.1–L2.2 ✅ · L3.1 ✅ (0.28 m best; 0.82 m typical).
- L3.2 ❌ not yet measured — **this is the first D task** (batch harness). Observed
  reliability is currently marginal in the open field (localization-driven failures),
  which is exactly what A (GPS/EKF) targets.
