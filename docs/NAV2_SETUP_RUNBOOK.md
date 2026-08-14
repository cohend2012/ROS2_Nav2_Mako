# Nav2 bring-up runbook — zero to a navigating robot

Written so a person (or an AI in a fresh session) can stand this stack up from
nothing, in order, with a **checkpoint after every step**. The order is not
arbitrary: each stage consumes the previous stage's output, and skipping a
checkpoint means debugging three layers up from the actual fault. Everything
here was learned the expensive way — the "why" notes cite the incident.

Reference implementation: `tools/nav/bringup_plan_a.sh` (do not fork it —
one implementation per function; extend with a flag).

---

## Stage 0 — Prerequisites (before ROS is even involved)

0.1 **Robot description + frames.** You need `base_link` at the robot's control
    centre, a `<sensor>_link` per sensor, and the static transforms between
    them. Publish with `robot_state_publisher` from URDF (production) or
    `static_transform_publisher` (bring-up shortcut).
    - CHECK: `ros2 run tf2_tools view_frames` shows one tree, no orphans.
    - WHY: cross-container latched `/tf_static` silently died after a host
      suspend once; static TFs are now published in the CONSUMER's container.
      Never depend on latched delivery across process/host boundaries for
      anything safety- or perception-critical.

0.2 **Time.** Decide wall-clock vs `/clock` and set `use_sim_time` IDENTICALLY
    on every node. Mixed clocks produce "transform data too old" storms that
    look like sensor faults.
    - CHECK: `ros2 param get /<any_node> use_sim_time` agrees everywhere.

---

## Stage 1 — Odometry and the drive chain (no Nav2 yet)

1.1 Publish `nav_msgs/Odometry` on `/odom` AND broadcast TF `odom -> base_link`
    from the same source. Wheel encoders + gyro is fine; it may drift, it may
    NOT jump.

1.2 Accept `geometry_msgs/Twist` on `/cmd_vel` and drive the actuators.

- CHECK (L1.2, the single most valuable early test): command +0.3 m/s, measure
  actual body motion; command +0.5 rad/s, measure yaw rate.
  - forward command → +x motion → `/odom` x increases
  - positive angular → CCW yaw → `/odom` yaw increases
- CHECK (L1.3): drive 5 m straight; odom-vs-truth error < 0.1 m. Pivot 360°;
  heading error < 5°.
- WHY: our mixer used the nominal wheel radius (0.10) instead of the measured
  effective radius (0.072) — a 38% over-command that masqueraded as a Nav2
  tuning problem for days. **Calibrate the radius by driving a measured
  distance before touching a planner.**

---

## Stage 2 — Perception into the format Nav2 eats

2.1 Get the real sensor onto the bus in its native form (for us:
    `sensor_msgs/PointCloud2` on `/LIDAR/POINTS`, in `lidar_link`).

2.2 If the sensor is 3D and you're doing 2D nav, derive `/scan` with the
    PRODUCTION node (`pointcloud_to_laserscan`), not a bespoke script — the
    same node must run on hardware.
    - Height slice is in the TARGET frame; set `min_height`/`max_height` to
      include the obstacles that matter (we lowered ours to see 0.30 m ground
      pipes the old slice passed over).

- CHECK: `ros2 topic hz /scan` ≈ sensor rate; `ros2 topic echo --once /scan`
  has plausible `range_min`/`range_max` and non-inf returns.
- CHECK: visualize it (RViz LaserScan display or Foxglove 3D panel) and
  confirm hits land ON real obstacles.
- WHY: **verify sensors by what they SEE, not that they publish.** Our camera
  faced backwards for two weeks; `topic hz` was green the whole time.
- WHY (QoS): sensor topics are usually BEST_EFFORT (`SensorDataQoS`). A
  RELIABLE subscriber never connects and never warns. Match QoS explicitly;
  pass `--qos-reliability best_effort` to CLI probes.

---

## Stage 3 — The map (build it once, QA it, then reuse)

3.1 **Mapping run:** slam_toolbox in `mapping` mode, drive coverage of the
    site including a return-to-start LOOP so the graph gets a loop closure.
3.2 **Serialize** the posegraph (`/slam_toolbox/serialize_map`) AND save the
    `.pgm`/`.yaml` (`nav2_map_server map_saver_cli`) — the posegraph is what
    localization loads; the image is for human eyes and the quality gate.
3.3 **QA the map against ground truth** before anything depends on it: fit
    known landmarks (tank centres, surveyed markers) and require every one
    within tolerance (ours: 0.30 m).

- CHECK: landmark fit passes; obstacles crisp, no smearing/double walls.
- WHY: our first map froze the mapping run's odometry drift into the graph —
  up to 0.65 m of warp, region-dependent. Runtime localization was perfect and
  the robot still missed goals, because it was faithfully navigating a wrong
  map. **A map is an artifact with a QA gate, not a by-product.**
- WHY (drift during mapping): feed the mapper a GPS-anchored or otherwise
  bounded odometry prior — but SMOOTHLY (rate-limited). Raw GPS-corrected pose
  as the prior injects jitter into every scan registration and makes the map
  WORSE. Residual per-build is random: build N maps, ship the best-scoring.

---

## Stage 4 — Localization against the known map

4.1 Run slam_toolbox in `localization` mode on the serialized posegraph (or
    AMCL against the occupancy grid). It owns TF `map -> odom` ONLY.
4.2 **SEED the initial pose explicitly** by publishing `/initialpose` after the
    node has a subscriber.

- CHECK: with the robot stationary at a known spot, `map -> base_link` matches
  truth within a few cm, BEFORE driving anywhere.
- WHY: `map_start_at_dock: true` is silently unsupported in localization mode
  ("correctly not supported" in the log) — we ran unseeded for weeks;
  sometimes it self-converged, sometimes the robot simply never localized and
  never moved. Read your dependencies' startup logs.

4.3 **Bound the drift at runtime.** Long missions are where scan-matching
    mis-locks: it jumps, locks confidently wrong, and never recovers.
    Watch a residual (estimate vs. smoothed GPS or another absolute), and on a
    sustained excursion, re-seed `/initialpose`.
    - Derive the threshold from MEASURED residual distributions (healthy p95
      vs. failure plateau), never from a guess; add hysteresis (arm high,
      disarm low) or it flickers and never fires.
    - A re-anchor jumps the frame and will ABORT the in-flight goal — the
      mission layer must RETRY the leg. That is correct behaviour, not a bug.

---

## Stage 5 — Frames: the contract everything above depends on (REP-105)

```
map  --(localizer: may jump/correct)-->  odom  --(odometry: continuous, drifts)-->  base_link
```
- Exactly ONE publisher per edge, forever.
- Goals live in `map`. Odometry NEVER sees an absolute reference at runtime.
- CHECK: `ros2 run tf2_ros tf2_echo map base_link` streams without warnings;
  `view_frames` shows a single chain.
- WHY: we allow a GPS-pulled prior on the odom edge during OFFLINE map
  building only, and bring-up hard-refuses that flag in combination with
  runtime map mode. Enforce your exceptions in code, not in prose.

---

## Stage 6 — Nav2 itself

6.1 Launch `nav2_bringup navigation_launch.py` with your `nav2_params.yaml`.
    Start from the stock params and change one thing at a time.
6.2 **Capture Nav2's stdout to a file.** Non-negotiable.
    - WHY: two days lost to "why does it abort?" because bringup discarded the
      logs. The answer was in the controller's warnings the whole time.

Parameters that actually mattered for us, and why:
- `robot_radius` / footprint: real, plus margin. Safety, not tuning.
- `inflation_radius` + `cost_scaling_factor`: the soft cost around obstacles.
  Too large and legitimate corridors close ("No valid trajectories out of
  251!" repeated → patience exceeded → abort). We went 0.6 → 0.5 when an
  accurate map narrowed a real gap.
- `xy_goal_tolerance`: must match the localization accuracy you actually have.
  Ours sat at 0.45 (a live-SLAM-era value) long after localization improved —
  it was the largest term in our "error" metric.
- `transform_tolerance`: raise if `map->odom` lags on a loaded box; otherwise
  you get "extrapolation into the future" and chopped commands.
- Progress checker `movement_time_allowance`: rotation-in-place makes zero
  POSITION progress; the default aborted us mid-align.
- Acceleration limits: must exceed the actuator's stiction dead-zone, or the
  controller ramps from a measured zero forever and never breaks free.

- CHECK: costmaps in RViz/Foxglove show obstacles where the laser shows them.
- CHECK: `/plan` appears when a goal is sent; the path clears the footprint.

---

## Stage 7 — First goal, then reliability

7.1 Send one goal: `ros2 action send_goal /navigate_to_pose
    nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, ...}}"`.
7.2 Score it against GROUND TRUTH, not against the robot's own estimate — the
    estimate can be confidently wrong (that is the entire failure mode above).
7.3 **Then run a multi-goal patrol across the whole site**, not one goal
    repeatedly.
    - WHY: our stack passed 10/10 on a single goal for two weeks, then scored
      4/5 with 2 m errors the first time it toured five locations. Single-goal
      runs end before mission-scale drift matures. **Reliability is measured
      over missions.**
7.4 Batch it (N fresh full-stack runs, success rate + error distribution) on a
    QUIET machine, archiving per-run logs. Contention starves the control loop
    and produces failures that look algorithmic.

---

## Stage 8 — Safety and operations (do not defer)

- A supervisor that owns arming/modes and can veto: motion only when armed,
  hard stop on failsafe. Every autonomy feature ships with its failsafe.
- A command watchdog: zero velocity if `/cmd_vel` goes stale.
- Tip-over / attitude failsafe with a DURATION filter (an instantaneous
  threshold graze during an aggressive pivot should not e-stop a mission).
- An operator view (Foxglove/RViz) from day one — you cannot debug what you
  cannot see, and rendered-after-the-fact videos are not a substitute.
- Artifact discipline: the code you tested must be the code that runs. Verify
  installed-vs-source at bring-up.
  - WHY: our tip-over failsafe existed in git for two weeks while the running
    container had a pre-failsafe build. An inverted robot stayed armed.

---

## Order-of-operations summary (the short version)

1. Frames + time
2. Odometry + drive chain, CALIBRATED
3. Sensors, verified by what they see
4. Map: build → serialize → QA gate
5. Localization: seed it, bound it
6. Nav2: stock params, logs captured, one change at a time
7. One goal → patrol → batch, scored against truth
8. Safety + observability, never deferred

If a stage's check fails, fix it there. Every day this project lost was spent
debugging stage N+2 for a fault in stage N.
