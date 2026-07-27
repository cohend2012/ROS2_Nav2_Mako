# Phase A design note — GPS fusion (robot_localization)

Written 2026-07-20, on `phase-map-loc` (the finding below was made here; the
`phase-a-ekf` branch inherits this note).

## UPDATE (same night, later): the map itself is warped — Phase A is back on

The "centimeter-accurate localization" measurement below was an artifact: the
averaging window was dominated by idle time at the origin. A thirds-of-run
analysis on a fresh drive shows est-vs-true drifting 0.00 → −0.05 → **−0.30 m
(y)** as the robot travels west. And landmark-fitting the committed map against
the model's true geometry measures the map itself:

    tank1    true (-5.00,+4.00)  map (-4.73,+4.10)  offset (+0.27,+0.10)
    tank2    true (-1.00,+4.50)  map (-1.19,+4.30)  offset (-0.19,-0.20)
    wellhead true (+6.00,-1.00)  map (+5.39,-1.25)  offset (-0.61,-0.25)

**The mapping run's odometry drift froze into the posegraph, region-dependent,
up to ~0.65 m.** Localization against it is locally precise and globally warped.
Nav2 reaches map-frame goals within tolerance (verified: est final 0.19 m from
goal); the true-world error IS the local map warp. Everything now agrees:
tolerance fix worked, runtime localization works, the MAP is the debt.

Revised Phase A (sim): **GPS-anchored mapping.** During map-building runs, feed
slam_toolbox a GPS-EKF-corrected odometry (estimator.py already produces
/odom_filtered live at ~0.48 m bounded error) instead of raw dead-reckoning, so
frozen drift stays bounded instead of accumulating. Then rebuild the map,
re-measure the landmarks (the fit above is now a committed quality gate for
maps), and re-gate. Operational note: map-frame missions are repeatable TODAY —
the warp only matters when goals are specified in true-world/GPS coordinates.

## The (superseded) measurement that first changed the plan

The Phase A premise was: "successes cluster with a systematic ~0.5 m offset —
residual localization drift — GPS anchoring removes it" (TEST_PLAN, 2026-07-12).

Measured tonight on the known-map config (patrol run, 3554 matched samples):

    slam-est minus TRUE:  bias x -0.003 m (std 0.035)
                          bias y -0.006 m (std 0.065)   |bias| = 0.007 m
    GPS minus TRUE:       bias x +0.041, y +0.004       (unbiased, as modeled)

**Localization against the pre-built map is centimeter-accurate.** The 0.55–0.6 m
"error" in gate batches is NOT estimation — it is `xy_goal_tolerance: 0.45`
(a live-SLAM-noise-era setting) plus stopping dynamics. Nav2 stops the robot
0.45 m out because it is allowed to.

Action taken (this branch): tolerance 0.45 → 0.20 (goal checker + DWB), with
history preserved in nav2_params.yaml. Localization σ=0.065 leaves 3σ margin.

## What robot_localization is actually for (revised scope)

NOT for sim accuracy on the known map — that problem no longer exists. The EKF
earns its place for:

1. **Georeferencing on real hardware (the real Phase A).** Real GPS is lat/lon,
   not world-frame meters: `navsat_transform_node` + EKF anchors the site map to
   Earth coordinates (datum), so missions can be specified in GPS coordinates.
   This is required for the real M20; the sim's `/gps` shortcut hides it.
2. **Robustness**: smooth pose through scan-match dropouts (dust, featureless
   stretches — LIDAR_RESEARCH weather modes), sensor-dropout tolerance,
   covariance-honest state for the Commander's estimator-health failsafe.
3. **Odom-frame smoothing** (`odom→base_link` from wheel+IMU at high rate) —
   the standard two-EKF topology, useful once the vendor's real odom replaces
   the sim's dead-reckoning.

## Topology decision (for when it lands)

- slam_toolbox (localization mode) KEEPS `map→odom`; it is measured-excellent
  and battle-proven (10/10 gate). Do NOT demote it in sim.
- robot_localization runs as the **odom-frame** EKF (wheel odom twist + IMU →
  smoothed `odom→base_link`), replacing the sim's direct TF only when the
  REAL robot's sensor suite is in play (its odom will be noisier than the
  scan-matcher can absorb raw).
- `navsat_transform_node` + a map-frame EKF instance joins ONLY on hardware,
  publishing nothing in sim (the sim GPS is already world-frame).
- Never two publishers for one TF edge (hard rule).

## Acceptance (revised)

- Sim (this branch): gate ≥9/10 AND mean <0.5 m via the tolerance fix — no EKF.
- Hardware Phase A (future): missions accepted in lat/lon; EKF pose RMSE <0.3 m
  vs RTK/total-station reference on the bench course; scan-dropout injection
  (M20_WEATHER=dust) does not abort a mission.

## Rollback

The tolerance change is one knob, documented in nav2_params.yaml. If live-SLAM
mode (no M20_STATIC_MAP) shows arrive-never-declare timeouts again, the knob is
the first suspect — the 0.45 rationale still applies to THAT config.
