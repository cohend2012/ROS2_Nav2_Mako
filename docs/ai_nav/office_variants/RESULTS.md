# Classical Nav2 in the office: variant comparison (2026-11-05)

Real ROS 2 Humble + Nav2, office overlay (real 0.91 x 0.53 m footprint), kinematic office
world with box-exact LiDAR (`tools/ai_nav/ros_smoke.sh SITE=office VARIANT=<name>`).
Per run, two tests:
- **Nav leg:** reception → meeting room through its **0.85 m door**.
- **Agent leg:** "go to the kitchen and come back", two more door legs.

Raw logs are in `tools/ai_nav/ros_smoke/out/office/<variant>/run<N>/`; per-run JSON is in this folder.

| variant | runs | nav leg (0.85 m door) | agent kitchen round trip | failure mode |
|---|---|---|---|---|
| **`dwb`** (NavFn + DWB, office default) | 6 | **reached the goal in 5 of 5 runs that had time** (0.11-0.17 m, 137-176 s); 1 run hit the old 150 s limit en route | **2 PASS**, 1 stuck on the return (12x "failed to make progress"), 2 still running at the 300 s limit | slow, recovery-heavy at doors, but makes it |
| `shim_rpp` (NavFn + RotationShim→RPP) | 2 | arrived 0.11 m but Nav2 ABORTED (run 2); failed (run 1) | 1 PASS, 1 FAIL | RPP footprint collision check fires near door jambs/furniture (172-295x), then "patience exceeded" |
| `lattice_rpp` (Smac Lattice diff + Shim→RPP) | 2 | arrived 0.13 m but ABORTED (run 2); failed (run 1) | 1 PASS (0.19 m), 1 FAIL | same RPP collision check; the lattice plans themselves were fine |
| `mppi` (NavFn + MPPI DiffDrive) | 1 | did not reach it (stuck near reception) | FAIL | controller loop missed 20 Hz: 1000 rollouts on CPU, with a 2nd stack sharing the box |

## Read this before choosing
- **Use `dwb` (the office default).** It is the only variant that got through the door on
  every run that had enough time. It's slow (~150 s for ~15 m at the 0.3 m/s cap) and
  leans on recoveries, which hurts the agent round trip.
- **The RPP variants are close but blocked by one thing:** RPP's own footprint collision
  check (`use_collision_detection`) treats an 0.16 m-a-side door as a collision. Next on
  the real map: set `use_collision_detection: false` and put Nav2's `collision_monitor`
  behind the controller instead (polygon stop/slow zones). That is the documented Nav2
  pattern, and it keeps a hard safety stop.
- **MPPI needs a CPU budget test before any verdict:** run it alone, with `batch_size`
  400-600 and `time_steps` 40. On the robot it competes with SLAM and the agent.
- **Caveats:** 1-6 runs per variant; a kinematic world (no wheel slip); two stacks
  sometimes shared one CPU (rows 1, 4). These numbers rank the options and find their
  failure modes. They are NOT a reliability estimate. Re-run on the real office map with
  `VARIANT=` before switching.
