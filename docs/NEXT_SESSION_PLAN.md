# Next Session Plan (written 2026-07-19, post checkpoint-04)

State inherited: `master` @ `checkpoint-04-sensor-realism` (gate 9/10 @ 0.63 m, DWB,
realistic sensors). Open: observer split-brain / phantom-success DDS class blocks
trustworthy logging → blocks the two owed videos. Full ledger: TEST_PLAN.md session 4.

Standing rules (hard-learned): one experiment = one self-contained wsl session ·
quiet box before any batch (`uptime`) · evidence commit per result · no fake footage ·
one implementation per function (bringup/teardown are single-sourced — keep it that way).

## Block 0 — Preflight (10 min)
- [ ] `git status` clean on master; `uptime` load < 0.5; confirm sleep still disabled.
- [ ] Disk audit: `df -h /` in WSL + `docker system df` (Block 1 needs image-build room;
      `docker image prune` dangling layers if tight — do NOT prune tagged images).

## Block 1 — Durable DDS: migrate RMW to CycloneDDS (60–90 min) — THE blocker
Why: discovery-server fixed multicast flakiness but fails under participant churn
(ghost registrations → logger blind, 2× phantom SUCCEEDED). CycloneDDS is the
standard robust choice for single-host ROS 2.
1. Add `ros-humble-rmw-cyclonedds-cpp` to `docker/Dockerfile` AND the sim image
   recipe; rebuild both (interim cheap trial allowed: runtime install into the
   commander via container_deps + a derived sim image tag `m20_sim:cyclone`).
2. Export `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in compose + every script env
   (`SRC`/`S` strings, sim `docker run` lines, m20sh). REMOVE
   `ROS_DISCOVERY_SERVER`/`ROS_SUPER_CLIENT` and the compose `discovery` service
   (keep the service definition commented for rollback).
3. If lo multicast is still unreliable under WSL2 for cyclone: pin unicast via
   `CYCLONEDDS_URI` (peers list 127.0.0.1) — one XML, committed to docker/.
4. **Verification ladder (each step gates the next):**
   a. L1.1 topics + rates from a fresh bringup.
   b. OBSERVER-TRUST TEST ×2: one nav run each; PASS = nav_logger's true-trajectory
      x-range, a live end-of-run probe, and the action result all AGREE (travel
      really happened & was recorded). This is the test tonight's bus failed.
   c. Single goal green; then estimator A/B (M20_EKF=1) — retest the phantom link
      under the new RMW before blaming the estimator for anything.
Fallback if image rebuild is blocked (disk): stay on discovery server; restart
discovery+commander before every CAPTURE run; make nav_logger short-lived (start
at goal-send, not bringup). Bridge measure only — note it as debt.

## Block 2 — The two owed videos (45 min; unblocked by Block 1b)
1. Checkpoint-04 progress video: one instrumented green run (−6,−0.5) →
   `tools/ci/make_progress_video.sh` → `docs/media/checkpoint-04.mp4`.
2. Integrated video (the user's spec): same-format two-panel — MuJoCo replay +
   GPS estimate-race panel (`render_gps_video.py`). EKF trace from the live
   estimator if the A/B cleared it, else computed offline from logged /gps +
   /odom with the estimator's exact math, labeled as such.
3. Commit media; refresh the architecture artifact's status row (batch #14 result,
   checkpoint-04, videos linked).

## Block 3 — Phase A branch: GPS fusion with robot_localization (rest of day)
1. `git checkout -b phase-a-ekf` from master.
2. Design note first (30 min, in-branch doc): fusion topology for this stack —
   robot_localization EKF fusing wheel odom (/odom twist) + IMU + /gps pose,
   publishing the map-frame estimate; decide interplay with slam_toolbox TF
   (candidate: EKF owns map→odom, slam demoted to mapping-only; document the
   choice + rollback). Acceptance (TEST_PLAN): ≥9/10 & mean <0.4 m, EKF RMSE <0.3 m.
3. Implement ekf.yaml + bringup step; single green run; then 10-run gate
   (background, quiet box). Merge only on gate pass, tag checkpoint-05.

## Parked (do not start unless blocked on all above)
- Eastern pipe-corridor tuning (goal 7.5,4.0 declines the tight gap — DWB scoring
  vs corridor width; revisit with Phase B or after A).
- Camera simulation (#11) + progress-video automation (#12) — queued behind A.
- RPP root-cause writeup for upstream Nav2 (we have full probe data).
