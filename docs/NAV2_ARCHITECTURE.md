# The Nav2 Architecture — a standalone explainer

This document explains how the ROS 2 Navigation Stack ("Nav2") is put together:
what each component does, how they talk to each other, where the extension
points are, and how the pieces fail. It is written to be read without any other
context. Section 9 documents one concrete configuration — a wheel-legged
quadruped inspecting an oil & gas site — as a worked example.

---

## 1. What Nav2 is, and what it is not

Nav2 is the standard autonomous-navigation framework for ROS 2. Given a map, a
stream of sensor data, an estimate of where the robot is, and a goal pose, it
produces velocity commands that drive the robot to that goal while avoiding
obstacles.

**Nav2 is a collection of cooperating servers, not a single program.** Each
server is a separate ROS 2 node with one responsibility, and they coordinate
through ROS 2 *actions* — long-running, cancellable requests that stream
feedback — rather than through function calls.

Three things Nav2 is explicitly **not**:

- **Not a localizer.** Nav2 consumes a pose estimate; it does not produce one.
  Localization comes from AMCL, slam_toolbox, robot_localization, or a vendor
  system. Nav2 trusts whatever it is told.
- **Not a mapper.** It consumes an occupancy grid. Building that map is a
  separate concern (SLAM).
- **Not a low-level controller.** It emits velocity commands
  (`geometry_msgs/Twist`). Turning those into wheel torques, joint positions,
  or leg trajectories is the robot platform's job.

This boundary matters enormously in practice: a large fraction of "Nav2
problems" are actually localization, map, sensor, or transform problems, with
Nav2 faithfully executing against bad inputs.

---

## 2. The component model: lifecycle nodes and the manager

Every Nav2 server is a **managed (lifecycle) node**. A lifecycle node has
explicit states — unconfigured, inactive, active, finalized — and does not
process data until something transitions it to active.

The `lifecycle_manager` performs those transitions in a defined order at
startup and can restart a failed node. This exists so the stack comes up
deterministically: costmaps must be configured before the planner activates,
the planner before the behavior tree starts asking it for paths, and so on.

**Practical consequence:** if a Nav2 node is "running" but nothing happens,
check whether it is active. A configured-but-inactive server silently ignores
requests.

---

## 3. The servers

### 3.1 bt_navigator — the orchestrator
Exposes the primary entry points as ROS 2 actions:
- `NavigateToPose` — go to a single goal
- `NavigateThroughPoses` — go through an ordered list of poses

Internally it runs a **behavior tree**, an XML-defined policy that decides what
to do and when. The tree ticks at a high rate (default 100 Hz) and calls the
other servers as its leaf nodes. Everything about *sequencing* and *recovery*
lives here.

### 3.2 planner_server — the global path
Answers the `ComputePathToPose` action. Given start and goal, it searches the
**global costmap** and returns a geometric path (`nav_msgs/Path` on `/plan`).

Common planner plugins:
- **NavFn** — Dijkstra/A* on the costmap grid. Fast, reliable, ignores
  kinematic constraints. The usual default.
- **Smac Planner** — variants producing kinematically feasible paths
  (Hybrid-A* for car-like robots, State Lattice, 2D).
- **ThetaStar** — any-angle variant producing fewer, longer segments.

The global path has **no timing and no dynamics** — it is a sequence of poses
through free space, nothing more.

### 3.3 controller_server — the local trajectory
Answers the `FollowPath` action. Runs at `controller_frequency` (typically
20 Hz). Each cycle it takes the global path, the **local costmap**, and the
current pose, then emits one velocity command.

Common controller plugins:
- **DWB** (Dynamic Window Approach, the ROS 2 rewrite of `dwa_local_planner`):
  samples many candidate velocity pairs, forward-simulates each into a short
  trajectory, scores every trajectory with a set of **critics**, and publishes
  the winner. Critics include PathAlign (stay on the path), GoalAlign,
  PathDist, GoalDist, BaseObstacle (avoid cost), Oscillation (do not dither),
  and RotateToGoal (final heading). Each has a `scale` weight — tuning DWB is
  largely tuning those weights.
- **Regulated Pure Pursuit (RPP):** geometric path-following that picks a
  lookahead point and curves toward it, regulating speed by curvature and
  obstacle proximity. Smooth and predictable; fewer knobs than DWB.
- **MPPI** (Model Predictive Path Integral, newer): sampling-based model
  predictive control optimizing over a receding horizon. Generally smoother
  and better at tight maneuvers, at higher CPU cost.

The controller also hosts two pluggable checkers:
- **Progress checker** — has the robot moved `required_movement_radius` within
  `movement_time_allowance`? If not, the goal fails. Note it measures
  *position*, so pure rotation counts as zero progress.
- **Goal checker** — is the robot within `xy_goal_tolerance` and
  `yaw_goal_tolerance`? This defines "arrived."

### 3.4 behavior_server — recoveries
Hosts short, self-contained maneuvers the behavior tree invokes when
navigation stalls: Spin (rotate in place to clear the costmap and re-see the
world), BackUp, Wait, DriveOnHeading, AssistedTeleop. These publish velocity
commands directly — meaning `/cmd_vel` legitimately has more than one
publisher.

### 3.5 smoother_server — path polishing
Optional. Turns a jagged planner output into a smoother path (SimpleSmoother,
ConstrainedSmoother, SavitzkyGolaySmoother). Useful when grid-planner
staircase artifacts make the controller weave.

### 3.6 velocity_smoother — command conditioning
Sits between the controller and the robot, applying acceleration and jerk
limits so commanded velocity changes are physically reasonable. It is the last
node before the hardware layer.

### 3.7 waypoint_follower — simple multi-goal sequencing
Accepts a list of poses and calls `NavigateToPose` for each in turn, with
optional task executors at each stop (take a photo, wait). Adequate for simple
patrols; complex missions usually justify a purpose-built mission layer above
Nav2.

### 3.8 map_server / map_saver
Loads a saved occupancy grid (`.pgm` + `.yaml`) and publishes it on `/map`, or
saves the current map to disk. Not needed when a live SLAM node publishes
`/map` itself.

### 3.9 collision_monitor (optional safety layer)
Watches raw sensor data against configured zones and can slow or stop the
robot independently of the planning stack — a low-latency safety net beneath
the costmap logic.

---

## 4. Costmaps: how the world becomes numbers

A **costmap** is a 2D grid where each cell holds a cost from 0 (free) to 254
(lethal), plus a special "unknown" value. Nav2 runs two of them.

### 4.1 Global costmap
Covers the planning area, updated slowly (~1 Hz), used by the planner. Two
common configurations:
- **Static map mode:** `rolling_window: false` plus a static layer loading the
  saved map. The planner sees the entire site from the first instant.
- **Rolling window mode:** `rolling_window: true` with a fixed window that
  follows the robot. Used when no prior map exists; typically paired with
  `track_unknown_space: false` so unmapped area counts as free and distant
  goals remain plannable.

### 4.2 Local costmap
A small window (a few meters) around the robot, updated fast (~5 Hz), used by
the controller for immediate obstacle avoidance. Almost always rolling, and
almost always in the `odom` frame so it is immune to localization jumps.

### 4.3 Layers
Costmaps are built from stacked plugin layers, each contributing cost:
- **StaticLayer** — the prior map.
- **ObstacleLayer** — marks cells from live sensor data (LaserScan,
  PointCloud2) and clears cells by raytracing along the beam. Configured per
  observation source with marking/clearing flags, range limits, and height
  limits.
- **VoxelLayer** — 3D version tracking occupancy in vertical columns; better
  for overhangs and low obstacles.
- **InflationLayer** — the most misunderstood layer. It does not add
  obstacles; it spreads decaying cost *outward* from lethal cells so paths
  prefer clearance. Two parameters: `inflation_radius` (how far influence
  extends) and `cost_scaling_factor` (how steeply cost decays). Set too large
  and legitimate gaps become impassable; too small and the robot cuts corners.
  Actual collision safety comes from `robot_radius`/footprint, not inflation.
- **RangeLayer**, **DenoiseLayer**, and others for specific sensors.

---

## 5. The transform contract (REP-105)

Nav2 depends on a specific TF tree, and violations produce failures that look
like planning bugs:

```
map --(published by the localizer)--> odom --(published by odometry)--> base_link --> sensor frames
```

- **map to odom** is the *correction*, owned by the localizer (AMCL,
  slam_toolbox, or a map-frame EKF). It may jump discontinuously — that is
  precisely its job, correcting accumulated drift.
- **odom to base_link** is the *dead-reckoned pose*, owned by wheel/inertial
  odometry. It must be **continuous and smooth**; it may drift, but it must
  never jump. Anything absolute (GPS, a global localizer) must never be
  injected here.
- **base_link to sensor** are the static mounting transforms.

**Exactly one publisher per edge, always.** Two nodes publishing the same
transform produces intermittent, near-undebuggable behavior.

Why the split: the controller and local costmap work in `odom` because they
need smoothness; the planner and global costmap work in `map` because they
need global consistency. The two-frame design lets the robot receive an abrupt
localization correction without the local controller seeing a velocity spike.

---

## 6. End-to-end data flow

```
        goal (NavigateToPose action)
                    |
                    v
             [ bt_navigator ]        runs the behavior tree, ~100 Hz tick
              |            |
   ComputePathToPose    FollowPath        (both are ROS 2 actions)
              |            |
              v            v
      [ planner_server ]  [ controller_server ]
      (NavFn / Smac)      (DWB / RPP / MPPI)
            |                    |
     global costmap        local costmap        <-- sensors: /scan, pointclouds
     (map frame)           (odom frame)         <-- TF: map->odom->base_link
            |                    |
         /plan             /cmd_vel_nav
                                 |
                                 v
                      [ velocity_smoother ]
                                 |
                             /cmd_vel  -->  robot base / motor driver
```

A single navigation cycle, in words:

1. A client sends a `NavigateToPose` goal in the `map` frame.
2. `bt_navigator` ticks its tree, which asks `planner_server` for a path —
   by default about once per second, so the path stays current.
3. `planner_server` searches the global costmap and publishes `/plan`.
4. The tree hands that path to `controller_server` via `FollowPath`.
5. Every control cycle (~50 ms) the controller samples or optimizes candidate
   motions against the local costmap and emits a velocity command.
6. `velocity_smoother` conditions it; the base executes it.
7. The goal checker declares success when the robot is inside tolerance; the
   progress checker declares failure if the robot stops making progress.
8. On failure the tree runs recovery behaviors and retries.

---

## 7. The behavior tree: where policy lives

The default tree is `navigate_to_pose_w_replanning_and_recovery.xml`.
Simplified structure:

```
RecoveryNode (top level)
├── PipelineSequence "NavigateWithReplanning"
│   ├── RateController (1 Hz)
│   │   └── ComputePathToPose        <- ask the planner
│   └── FollowPath                   <- hand the path to the controller
└── SequenceStar "RecoveryActions"   <- only on failure
    ├── ClearEntireCostmap
    ├── Spin
    ├── Wait
    └── BackUp
```

Reading this tree explains behavior that otherwise looks mysterious:
- The robot re-plans continuously, so routing around a newly appeared obstacle
  requires no special handling.
- A stuck robot spins and backs up before giving up — that "confused
  shuffling" is the recovery subtree, not a malfunction.
- Recovery attempts are bounded; when exhausted, the action returns ABORTED.

Because the tree is XML, changing navigation *policy* (different recoveries,
domain-specific checks, mode switches) requires no C++ and no rebuild. This is
the intended customization path for behavior; plugins are the path for
algorithms.

---

## 8. The plugin architecture

Nearly every algorithmic component is a `pluginlib` plugin selected by name in
the parameter file:

| Extension point | Selected in | Examples |
|---|---|---|
| Global planner | `planner_server.planner_plugins` | NavFn, SmacHybrid, ThetaStar |
| Local controller | `controller_server.controller_plugins` | DWB, RPP, MPPI |
| Costmap layer | `<costmap>.plugins` | Static, Obstacle, Voxel, Inflation |
| Progress checker | `controller_server.progress_checker_plugin` | SimpleProgressChecker |
| Goal checker | `controller_server.goal_checker_plugins` | SimpleGoalChecker, StoppedGoalChecker |
| Recovery behavior | `behavior_server.behavior_plugins` | Spin, BackUp, Wait |
| Smoother | `smoother_server.smoother_plugins` | SimpleSmoother, ConstrainedSmoother |
| BT node | BT XML + plugin list | custom condition/action nodes |

**Consequence for maintenance:** a well-run Nav2 deployment almost never forks
Nav2. Customization is (a) parameters, (b) plugin selection, (c) a custom
behavior tree, and (d) nodes *around* Nav2 that shape its inputs or supervise
its outputs. Forking the source forfeits upstream fixes and should be a last
resort.

---

## 9. A worked example: an inspection quadruped

The following is one real configuration — a wheel-legged quadruped performing
autonomous inspection of an outdoor industrial site — chosen to show how the
generic architecture gets instantiated and where the non-obvious decisions are.

### 9.1 Stack composition
- **Localization:** slam_toolbox in *localization mode*, matching live scans
  against a pre-built, quality-checked pose graph. It owns map to odom. The
  site map is built once (mapping mode with a loop closure), serialized, and
  validated against surveyed landmark positions before it is trusted.
- **Odometry:** wheel encoders plus gyro, publishing `/odom` and odom to
  base_link. Drifts, never jumps.
- **Sensing:** a 3D LiDAR publishing PointCloud2, converted to a 2D LaserScan
  by the standard `pointcloud_to_laserscan` node, with a height slice chosen
  to include low ground obstacles.
- **Nav2:** NavFn planner, DWB controller, standard recoveries, velocity
  smoother.
- **Base interface:** a bridge node consuming `/cmd_vel` and producing the
  vendor's joint-level commands. The only node that talks to the vendor SDK.
- **Supervisor:** a commander node owning arming and failsafes, with veto
  authority over motion. Nav2 can request motion; the supervisor can refuse.

### 9.2 Parameter decisions that mattered, and why

- **Controller: DWB rather than RPP.** RPP was tried first but repeatedly
  rotated toward a heading far from the goal and dithered indefinitely on this
  platform. DWB, with no other changes, produced immediate success. The lesson
  is not "DWB is better" — it is that controller choice is empirical and
  swapping plugins is a one-line experiment.
- **Goal tolerance matched to actual localization accuracy.** The value was
  inherited from an era of noisier localization and never revisited; once
  tightened, measured goal error dropped correspondingly. A tolerance looser
  than your localization is a silent accuracy ceiling. Note it must be set in
  two places — the goal checker and the controller each read their own copy.
- **Inflation radius reduced when the map got *better*.** A more accurate map
  rendered a real corridor at its true (narrow) width; the previous inflation
  closed it, and the controller reported "no valid trajectories" until
  patience expired. Collision safety was unaffected, being governed by
  `robot_radius`.
- **Progress-checker allowance raised.** The checker measures translation; the
  robot's slow rotate-to-align phase registered as zero progress and aborted
  goals mid-turn.
- **Transform tolerance raised.** On a loaded compute platform the map-to-odom
  correction lagged ~0.1 s, producing "extrapolation into the future" errors
  that truncated commands.
- **Angular acceleration limits raised — in four places.** Drivetrain stiction
  creates a dead zone; a controller ramping from the *measured* rate can never
  escape it if acceleration times timestep is smaller than that dead zone. The
  limit must agree across the controller, the behavior server, and the
  velocity smoother, or the most restrictive silently wins.

### 9.3 Supervision built *around* Nav2

Two failures could not be fixed with Nav2 parameters at all, because Nav2 was
not the faulty component.

**Map quality.** The first site map baked the mapping run's odometry drift
into the pose graph — up to 0.65 m of position error, varying by region.
Localization was excellent and navigation still missed goals, because the
robot was accurately navigating a subtly wrong map. The fix was a map-build
pipeline that bounds drift during mapping, plus a landmark-based quality gate
that refuses to publish a map whose known features are misplaced.

**Mid-mission localization mis-lock.** Over long multi-goal missions the
scan-matcher occasionally lost its anchor in feature-sparse areas, jumped
roughly 2 m, and locked confidently onto the wrong solution. The robot then
planned against phantom geometry and spent minutes fighting obstacles that
were not where it believed them to be. Single-goal tests never revealed this,
because they ended before the failure had time to develop.

The fix is a small supervisory node that:
1. computes the residual between the localizer's pose and a smoothed absolute
   reference (GPS),
2. arms when that residual exceeds a threshold and stays armed with hysteresis
   (a single threshold flickers across the line and never fires),
3. re-publishes `/initialpose` to force the localizer to re-converge, and
4. respects a cooldown afterward.

Thresholds were derived from *measured* residual distributions — healthy
driving peaked near 1.2 m, the failure plateaued near 2.0 m — rather than
guessed. A guessed threshold sitting inside the healthy band caused eleven
false corrections in a single mission.

One important second-order effect: re-anchoring jumps the map frame, which
correctly aborts the goal in flight. The mission layer must retry the
interrupted leg. That retry is part of the design, not a workaround.

### 9.4 Testing structure
Because most navigation failures are statistical, the configuration is
validated at three levels:
- **Component checks:** drive-direction signs, odometry error over a measured
  distance, sensor rate *and content*, watchdog and failsafe behavior.
- **Subsystem checks:** does the map contain the real obstacles, does
  localization correct drift, does the planner produce a valid path.
- **Integration:** a single goal, then a multi-goal patrol across the whole
  site, then a batch of many fresh runs reporting success rate and error
  distribution. Scoring is always against ground truth, never the robot's own
  estimate — a mis-locked robot reports success while standing meters away.

---

## 10. Common failure modes and what they actually mean

| Symptom | Usual cause |
|---|---|
| "No valid trajectories out of N!" then abort | Inflation or footprint closes the gap the planner chose; local costmap sees an obstacle the global one does not |
| Goal succeeds but the robot is meters away | Localization is confidently wrong; the robot did reach the goal *in its own frame* |
| Robot rotates forever without translating | Controller aligning to a bad heading, or angular acceleration limit below the drivetrain dead zone |
| "Extrapolation into the future" / "transform data too old" | Clock mismatch (`use_sim_time`) or a lagging localizer; raise transform tolerance only after ruling out clocks |
| Robot spins and backs up repeatedly | Recovery subtree running: the controller could not make progress |
| Nothing happens when a goal is sent | Lifecycle nodes not active, or the localizer never initialized so there is no map-to-odom transform |
| Works, then intermittently does not | Two publishers on one TF edge, or a QoS mismatch (a RELIABLE subscriber never connects to a BEST_EFFORT sensor publisher — silently) |
| Topic publishes at the right rate but nothing works | Verify sensor *content*, not just frequency; a sensor can publish perfect data while pointing the wrong direction |

---

## 11. Glossary

- **Action** — a ROS 2 request/response pattern for long-running tasks, with
  progress feedback and cancellation. Nav2's servers are wired together with
  these.
- **Behavior tree** — an XML-defined policy of sequences, fallbacks, and
  decorators governing what the navigator does and when.
- **Costmap** — a grid of traversal costs derived from a map and live sensors.
- **Critic** — a scoring function inside DWB rating a candidate trajectory on
  one dimension (path alignment, obstacle proximity, and so on).
- **Footprint / robot_radius** — the robot's collision geometry; the actual
  safety boundary.
- **Inflation** — cost spread outward from obstacles to encourage clearance. A
  preference, not a collision boundary.
- **Lifecycle node** — a managed node with explicit configure/activate states.
- **Odometry** — dead-reckoned pose from proprioceptive sensors; smooth and
  drifting.
- **Plugin (pluginlib)** — a runtime-loadable algorithm implementation selected
  by name in the parameter file.
- **REP-105** — the ROS convention defining the map / odom / base_link frame
  hierarchy.
- **SLAM** — Simultaneous Localization and Mapping; builds a map while tracking
  pose within it.
- **TF (tf2)** — the ROS transform system tracking coordinate frames over time.
