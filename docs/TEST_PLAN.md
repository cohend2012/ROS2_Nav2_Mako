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
| L1.6 | Tip-over failsafe | drive into a ground pipe (teleop) | commander latches FS_TIPOVER >35° roll/pitch → ESTOP+disarm; auto-clears flag after 5 s upright (no auto re-arm) | BUILT 2026-07-15 (observed flip 2026-07-13 prompted it) — bench test pending |
| L1.7 | Real LiDAR interface | stack up, probe /LIDAR/POINTS | ~10 Hz PointCloud2 in lidar_link; no returns <0.5 m standing; derived /scan (pointcloud_to_laserscan) ~10 Hz; ground pipes visible ahead | ✅ 2026-07-15: 9.8 Hz, rmin 0.79, /scan 10.0 Hz from projection node, pipe_cross seen at 1.85 m |
| L1.8 | LiDAR realism | `tools/nav/lidar_realism_check.py` | XYZIRT contract (32-byte points, rslidar offsets); timestamps span ~100 ms, monotone; noise σ 0.5–3 cm; ground dropout 0.5–12%; skew: pipe line-fit RMS >2× parked and >3 cm at 1 rad/s | ✅ 2026-07-15 PASS 7/7: 98.6 ms / 20 steps, σ 1.49 cm, drop 3.3%, skew 3.6→22.7 cm (6.4×) |

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

## OPEN INVESTIGATION (2026-07-17): nav runs abort in rotate-to-heading
Fixed today (each verified at its level): commander-local lidar static TF
(sensing chain green), estimator radius 0.072, bridge mixer radius 0.072
(L1.2 best-ever: 99% forward speed, +42°/3s turn). Yet full nav runs STILL
never leave rotate-to-heading (true yaw frozen while Nav2 recoveries flap the
wheels; batch #9 was 0/10 this way). Ruled out: arming, sensing (/scan+/map
green), physics authority (L1.2), the ±π singularity (south goal fails too),
IMU delivery (199 Hz). NOT yet ruled out: what Nav2 itself thinks — because
**bringup discards Nav2's logs (`docker exec -d`)**. NEXT INSTRUMENT: launch
Nav2 with stdout captured to a file and read the planner/controller/BT errors
during a failing run. Do this BEFORE any further tuning or batches.

## OPEN INVESTIGATION (2026-07-17): Nav2 rotate-to-heading deadlock — nav broken on this branch
State: NO Nav2 goal has succeeded since the post-suspend era began. Master
(checkpoint-03) remains last known-good. Debugging ledger, all measured:
1. FIXED (verified): cross-container /tf_static latching died after host suspend →
   commander-local static_transform_publisher in all bringups (8372513, e65d64d).
2. FIXED (verified): bridge mixer R was nominal 0.10 vs calibrated 0.072 (40b69b3);
   teleop now tracks commands (0.3 cmd → 0.309 actual; pivot 42°/3 s at wz=0.5).
3. FIXED (verified, WIP): inner PI yaw loop (Ki=0.8) limit-cycled against Nav2's
   20 Hz heading loop (body −0.59 on −0.44 cmd, sign-flapping) → Kp/Ki = 3.0/0.1;
   oscillation gone, tracking clean.
4. FIXED (verified, WIP): RPP transform_tolerance 0.1 → "extrapolation into the
   future" zero-ticks chopped the angular ramp → 0.3; errors now zero.
5. IMPROVED (WIP): max_angular_accel 1.5→4.0 raised the rotation-command ceiling
   0.19→0.44 rad/s (RPP ramps from MEASURED rate; can never out-ramp a dead zone
   wider than accel*dt).
6. UNSOLVED: two runs froze at yaw = −0.44..−0.45 rad (−25°) and Nav2 aborts; even
   a straight-ahead goal aborts; a bisect to last-committed config ALSO fails at
   origin (but that config still contains the accel deadlock, so it discriminates
   nothing — bisect design error, see next steps).
SESSION 2 FINDINGS (2026-07-17, forensics on fresh VM — suspects CLEARED):
  * COMMANDER CRASH-LOOP EPOCH FOUND: docker logs showed ~30 "Commander up" boots
    over hours — the commander container's PID 1 was dying every 1-3 min, and every
    death killed ALL exec'd nodes (bridge/slam/Nav2/pc2ls) mid-run, and each rebirth
    published latched DISARMED (bridge halts). This invalidates EVERY WIP-fix test
    run during that epoch. Post-clean-reboot: commander stable 10+ min, zero deaths.
    (Docker RestartCount resets after 10 s healthy — deaths were invisible to it.)
  * Omniscient run (full observability) on batch-#6 config: commander armed
    throughout, ZERO failsafes (L1.6 innocent), planner publishes plans fine (118
    msgs — earlier "/plan missing" was a probe artifact). Failure = the known
    accel-1.5 rotate deadlock, as designed. Watchdog 1.0 s holds through BT gaps.
  * Fair test of the full 5-fix stack on healthy VM: STILL aborts (one
    progress-checker strike at 40 s), and /odom_true was GONE at run end —
    NEW PRIME SUSPECT: the SIM CONTAINER dying mid-run (~80-90 s), traceless
    because all scripts run it with --rm. A dead sim = no scan/odom = no progress
    = abort, matching everything.
NEXT SESSION (do in order, one variable at a time):
  0. Re-run omniscient_run.sh with the sim started WITHOUT --rm; if it dies:
     docker logs + docker inspect exit code + dmesg (OOM?) give the cause directly.
     Also record `docker events` during the run (catches death timestamps).
  0b. If the sim IS dying: prime suspects = MuJoCo instability from the robot
     rocking during compensated pivots (check sim log for physics warnings/NaN)
     or WSL OOM (sim ~1 GB RSS). Fix accordingly (solver params / memory).
  a. TRUE bisect: git checkout 665b6f9 -- bridge_node.py nav2_params.yaml AND
     revert bridge R to 0.10 (= exact batch-#6 files) + keep ONLY the static-TF
     script fix. One goal run. Batches #5/#6 scored 7/10 & 5/10 on that config.
  b. If (a) drives: walk forward one delta at a time (R=0.072 → gains → tolerance
     → accel → comp), one run each, find the poison pair.
  c. If (a) fails: the breakage is environmental/sim-side (post-suspend WSL state,
     realism sensor timing?) — A/B M20_LIDAR_REALISM=0, then reboot WSL cleanly.
  d. Capture /plan + rotate-mode state in the probe (why yaw froze at exactly −25°:
     suspicion = RPP rotating toward the CARROT heading, not the path end — check
     transformed-plan geometry near the robot).

## Known operational gotchas (regression guards)
- **Cross-container latched (transient_local) topics are NOT reliable on this box.**
  After a host sleep/resume cycle (2026-07-16), the sim's once-published `/tf_static`
  (base_link→lidar_link) stopped reaching other containers while continuously-published
  topics flowed fine — silently killing pc2ls → /scan → SLAM → all navigation (the
  "never-moved" batch-#8 runs). FIX: the lidar static TF is now published
  commander-local by `static_transform_publisher` in bringup/sim_up (kill pattern
  updated). Rule: never depend on cross-container latched delivery for anything
  critical; publish static TFs in the consumer's container.
- **Host sleep/resume also corrupts state, not just timing:** it can restart the
  commander, remove --rm containers (the sim), skew `uptime` vs wall clock (the
  diagnostic tell: `/sbin/init` start time ≠ boot time implied by `uptime`), and leave
  the ROS 2 CLI daemon with a stale topic cache (`ros2 daemon stop/start` fixes views).
- **WSL kills the whole VM (and every container) when no session is open.** Root cause
  of the 2026-07-15 "sim died between commands" mystery: WSL idles the VM out after the
  last session closes; `docker-commander-1` auto-restarts on next boot (restart policy)
  but the sim container silently vanishes. RULE: every scripted live test must start
  its own sim INSIDE its own single session (batch_nav_test.sh already does); never
  assume a container started by a previous command is still alive.
- **Host sleep poisons batch runs.** The dev box sleeping mid-batch produced 5,000 s
  "runs" and bogus TIMEOUTs (2026-07-15). batch_nav_test.sh now flags any run >400 s
  wall as HOST-SLEEP SUSPECTED and declares the batch unreliable. Disable Windows
  sleep (or run `powercfg /change standby-timeout-ac 0`) before starting a gate.
- **Restart Nav2 whenever the sim restarts.** Reusing Nav2 across a sim restart leaves stale
  costmap/TF state (robot "teleports") → repeated collision/TF aborts. Full-stack fresh
  restart is reliable. (Root cause of the 3 failed runs on 2026-07-12.)
- **One bridge only.** Multiple `bridge_node` instances fight over `/JOINTS_CMD`. Kill stale
  ones; after `kill -9`, wait for DDS to reap phantom publishers before trusting counts.
- **`sdk_backend:=sim`** or the bridge is a no-op stub (no `/JOINTS_CMD`).

## Current status (2026-07-15: REAL LiDAR INTERFACE GATED — phase D complete)
- **L3.2 re-gate on the real interface (`/LIDAR/POINTS` → pointcloud_to_laserscan →
  `/scan`): PASS — 8/10, mean 0.57 m, p95 0.63 m, no contamination.** Statistically
  equivalent to the old native-/scan baseline (8/10, 0.54 m): the interface switch
  cost nothing. Failures were 2 mid-course TIMEOUTs (1.55 m / 2.86 m from goal,
  genuine stalls) — same localization-driven class Phase A targets.
- First batch on the new interface (2026-07-15 early) was HOST-SLEEP contaminated
  (7/10 with 5,000 s runs) — detected, discarded, re-run. The gate now flags this.
- L1.6 tip-over failsafe implemented in commander (FS_TIPOVER → ESTOP); bench test
  still to be run against a scripted flip (carry into next phase).
- Phase D closes at tag `checkpoint-03-real-lidar-interface`. Next: Phase A (GPS EKF).

## Prior status (2026-07-12, first L3.2 batch — old native /scan interface)
- L1.1–L1.3 ✅ · L2.1–L2.2 ✅ · L3.1 ✅ (0.28 m best).
- **L3.2 MEASURED & PASSED: 8/10 success, mean 0.54 m, p95 0.59 m**
  (`tools/ci/batch_nav_test.sh`, results in `tools/ci/out/batch_results.csv`).
  Failure analysis (honest):
  - run 2 ABORTED and run 3 TIMEOUT — both in the FINAL APPROACH, truly 0.57 m /
    0.29 m from the goal (run 3 was within tolerance in ground truth when the
    150 s harness timeout hit). Nobody drives into obstacles anymore; the
    fresh-stack discipline killed that failure class.
  - Successes cluster tight: x ≈ −5.77±0.04, y ≈ 0.0±0.04 vs goal (−6, −0.5) —
    a SYSTEMATIC ~0.5 m offset (mostly in y), i.e. residual localization drift:
    Nav2 reaches the goal in the *estimated* frame; truth is offset by drift.
    That bias is precisely what Phase A (GPS EKF) is for — expect mean error to
    drop well under 0.4 m once GPS anchors the estimate.
  - Pass is at the floor (8/10 = exactly 80%): do not merge D on a single batch;
    re-run the gate after the interface switch.
