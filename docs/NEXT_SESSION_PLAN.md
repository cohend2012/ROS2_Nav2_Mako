# Next Session Plan (written 2026-07-20, post CycloneDDS — TWO-TRACK edition)

State inherited: `master` @ tag `checkpoint-04-sensor-realism`. Gate 9/10 @ 0.63 m
(DWB, realistic sensors, CycloneDDS). Observers trusted (observer_trust.sh PASS 2/2).
Estimator exonerated. Videos delivered. Prior plan's Blocks 0–2 DONE (`7e41bf2`,
`b3aeb62`). Full ledger: TEST_PLAN.md session 5; SECOND_BRAIN rev 38–39.

DECISION (2026-07-20, user): run TWO parallel working branches and see where each
goes. Rationale: the remaining failure classes split cleanly in two —
- systematic ~0.5 m localization offset  → attacked by GPS anchoring (Track 1)
- corridor declines + blind-start behavior → attacked by a PRE-BUILT MAP (Track 2)
For real oil & gas inspection, "map the site once, then localize against it" is the
standard deployment pattern anyway — Track 2 is not a shortcut, it's the product shape.

Answers to the standing "why does it look wrong" questions (don't re-debug):
- Slow to get ready/moving = **DWB slow-align phase**. Known + logged, not a regression.
- Hesitating behind a box = **eastern pipe-corridor abort** (DWB scoring vs corridor
  width). Costmap sees the pipes; robot never touches them. Track 2's static map is
  the first credible attack on this.

Standing rules (hard-learned): one experiment = one self-contained wsl session ·
quiet box before any batch (`uptime`) · sleep disabled (`powercfg` check) · evidence
commit per result · no fake footage · one implementation per function (bringup/teardown
stay single-sourced through `bringup_plan_a.sh` / `kill_stack.sh`).

## Block 0 — Preflight (10 min, once, before either track)
- [ ] `git status` clean on master; `uptime` load < 0.5; sleep still disabled.
- [ ] Disk audit: `df -h /` + `docker system df` (prune dangling only, never tagged).
- [ ] One fresh bringup → L1.1 green → observer_trust.sh 1× (cheap regression check).

---

## Track 1 — branch `phase-a-ekf`: GPS fusion (robot_localization)
Target: the systematic ~0.5 m residual offset seen in every batch.
1. **Design note FIRST** (`docs/design/phase_a_ekf.md`): fusion topology — EKF fuses
   wheel odom twist + IMU + /gps pose → map-frame estimate. THE decision to write
   down: who owns `map→odom` (candidate: EKF owns it, slam_toolbox demoted to
   mapping-only). Document choice + rollback. 2D mode; GPS noise ±0.8 m @ 5 Hz.
2. Implement: `ekf.yaml` in `src/m20_perception/config/` + bringup step guarded by
   `M20_RL_EKF=1` (master behavior unchanged until gate pass).
3. Ladder: (a) TF tree clean — ONE publisher per edge (classic failure: TF fight
   with slam_toolbox); (b) single goal green + overlay EKF vs /odom_true vs raw;
   (c) EKF RMSE < 0.3 m; (d) 10-run gate.
4. **Accept: ≥9/10 AND mean <0.4 m AND RMSE <0.3 m** → merge, tag.
Fallback: if TF-ownership swap destabilizes SLAM, run EKF odom-frame-only
(smoothed source), document as debt. Never ship a two-parent TF tree.

## Track 2 — branch `phase-map-loc`: known map + localization mode
Premise (user decision): we MAY assume the area can be pre-mapped. Map once,
navigate against the saved map thereafter.
1. **Mapping run** (one-time artifact): bringup in mapping mode, drive coverage
   (teleop or scripted loop past tanks/skids/corridor), then save BOTH formats:
   - `ros2 run nav2_map_server map_saver_cli` → `maps/oil_gas_field.pgm/.yaml`
   - slam_toolbox serialize → `maps/oil_gas_field.posegraph` (+ .data)
   Quality gate = L2.1: tanks/skids/pipes present, no smearing. Commit the map
   as an artifact (it's small; it IS the deliverable of this step).
2. **Localization mode, smallest delta first**: slam_toolbox `mode: localization`
   loading the posegraph (same node we already run — no new stack member), still
   publishing `map→odom`. Fallback if relocalization is weak: classic
   map_server + AMCL (bigger change, keep as plan B inside the branch).
3. **Costmap switch**: global costmap rolling→static. `static_layer` from the map
   + obstacle layer on live /scan + inflation. The planner now sees the WHOLE
   field at t=0 — no more unknown=free gambles.
4. Bringup guarded by `M20_STATIC_MAP=maps/oil_gas_field` (empty = today's SLAM
   path; one bringup script, one flag — no forked bringup copies).
5. Ladder: (a) localization sanity — spawn robot, confirm map→base pose matches
   /odom_true within 0.3 m without driving; (b) single goal green; (c) 10-run gate;
   (d) STRETCH: the parked eastern corridor goal (7.5, 4.0) — first credible shot,
   since corridor geometry is now known a priori (may still need DWB critic tuning;
   if so, measure and file, don't rabbit-hole).
6. **Accept: ≥9/10 AND mean <0.5 m** (must beat live-SLAM 0.63 m to justify
   existence) → merge, tag.

## Convergence (after both gate — the real goal)
The production inspection stack is BOTH: known map + localization + GPS-anchored
EKF (+ GPS gives AMCL/slam-loc its initial pose for free). Order of merge = order
of gate pass; second branch rebases onto the first and RE-GATES before merging.
Converged config gates → `checkpoint-06` candidate. Neither branch touches master
until its gate passes (standing rule).

## Owed small items (fold into whichever session has slack)
- [ ] L1.6 tip-over bench test (scripted flip → FS_TIPOVER → ESTOP; owed since 07-15).
- [ ] SECOND_BRAIN changelog entry per result (it just went 5 days stale unnoticed).

## Parked (unchanged)
- DWB slow-align tuning (measure align time first; also the main batch wall-clock cost).
- Camera sim (#11), progress-video automation (#12), RPP upstream writeup.
