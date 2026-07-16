# M20 LiDAR & Real-Robot Autonomy Stack — Research Report (2026-07-12)

Deep-research pass (21 sources fetched, 100 claims extracted, 25 adversarially
verified: **13 confirmed 3-0, 3 refuted, 9 unverified** — rate-limit cut the last
verification round; unverified items are marked and one was verified locally).

**Round 2 (finished 2026-07-12): 8 of the 9 unverified claims resolved.**
- CONFIRMED: `fast-livo2-deep-robotics` supports **Lite3 Venture ONLY** ("ONLY THIS
  VERSION!"), targets **Livox Mid-360/360s** via `livox_ros_driver2` CustomMsg — it is
  NOT the M20 SLAM path. Lightning-LM is the M20 offering ("Lidar Localization and
  Mapping"); `sdk_deploy` covers M20+Lite3; `deep_robotics_model` hosts the URDFs.
- CONFIRMED LOCALLY: vendor M20 URDF and MJCF contain **no sensor links** (IMU site only).
- **KEY NEW FINDING — the mount-location question is ANSWERED:** all three official M20
  Lightning-LM configs (`config/default_deep_robotics*.yaml`) ship
  `extrinsic_T: [0,0,0]`, `extrinsic_R: identity`, `extrinsic_est_en: false`. The robot's
  driver **merges both physical LiDARs and publishes `/LIDAR/POINTS` already expressed in
  a single body frame co-located with the IMU** (`lidar_link`). Consumers never need the
  physical mount positions; `base_link→lidar_link` TF comes from the robot. Also from the
  configs: **`blind: 0.5`** (returns <0.5 m dropped — near-body blind zone),
  `scan_line: 32` (faster-lio line binning of the merged cloud), and the vendor's own
  grid-mapper (`g2p5`) uses `min_th_floor: 0.50` — i.e. **Deep Robotics' own 2.5D map
  ignores obstacles lower than 0.5 m above floor, so ground pipes require our Plan B
  3D layer even on the vendor stack.**
- Still unverified (source 403): the MDPI quadruped slam_toolbox+Nav2/MPPI paper.
Purpose: understand the real sensors so the sim models them faithfully and the
Nav2/SLAM architecture matches what the real robot needs.

## 1. The hardware (confirmed)

| Fact | Detail | Source |
|---|---|---|
| LiDARs | **2 × 96-line units**, combined **360° × 90°** FOV, **~860,000 pts/s**. Standard on both M20 and M20 Pro. | [deeprobotics.us](https://www.deeprobotics.us/products/lynx-m20/), [deeprobotics.cn](https://www.deeprobotics.cn/en/index/lynx.html) |
| LiDAR vendor | **RoboSense** — stated in Deep Robotics' own M20 SLAM guide (`fasterlio.lidar_type: 4 (RoboSense)`). Likely the **RS-LiDAR-Airy** (96-beam, exact hemispherical 360°×90° match), though DR never names the model. | [lightning-lm repo](https://github.com/DeepRoboticsLab/lightning-lm-deep-robotics), [RoboSense Airy](https://www.robosense.ai/en/IncrementalComponents/Airy) |
| Mounting | **Not published anywhere.** No vendor page, manual, URDF or MJCF states positions/tilt. Verified locally: our vendor MJCF has *no* sensor links at all (only `imu_site` at (0.0632, −0.0268, 0.0435)). Real extrinsics come from the robot's live TF (`base_link → lidar_link`). | vendor model on disk |
| Compute | **RK3588 (Rockchip ARM), NOT NVIDIA Jetson.** M20 = 2× octa-core 16 GB+128 GB units; Pro = 3×. Memory-constrained: DR warns a 4-core compile OOM-hangs it. | [lightning-lm repo](https://github.com/DeepRoboticsLab/lightning-lm-deep-robotics), [shop.deeprobotics.us](https://shop.deeprobotics.us/products/lynx-m20) |
| Onboard ROS | **ROS 2 Foxy** (not Humble). Our Humble Nav2 stack cannot run natively on the robot host — needs a Humble container on the robot or an offboard/backpack PC on the DDS bus. | [lightning-lm repo](https://github.com/DeepRoboticsLab/lightning-lm-deep-robotics) |

## 2. The software Deep Robotics actually ships (confirmed)

- **Topics on the real robot:** LiDAR is published as **`/LIDAR/POINTS` (PointCloud2)** —
  ONE merged cloud topic, not two raw per-unit topics — plus **`/IMU`**. TF:
  **`base_link → lidar_link`**. This matches our drdds-style bus experience.
- **Official SLAM = Lightning-LM** (faster-lio based LIO). It consumes
  `/LIDAR/POINTS` + `/IMU`, publishes **`/lightning/odom`**, `/lightning/path`, and
  `map → lidar_link` TF. BSD-3, C++, tuned to build on the RK3588 (low-memory build).
- **RoboSense's `rslidar_sdk`** driver supports **ROS 2 Humble** natively and publishes
  **XYZIRT** point format (per-point ring + timestamp) — exactly what FAST-LIO2/LIO-SAM
  style deskewing needs on a walking robot.
- *(Unverified, rate-limited: DR's `fast-livo2-deep-robotics` repo appears to target the
  Lite3 + Livox Mid-360, NOT the M20 — don't plan around it until checked.)*

## 3. What this means for OUR stack

**The real architecture (target):**
```
/LIDAR/POINTS (PointCloud2, merged, 860k pts/s)   /IMU
        │                                           │
        ├── 3D LIO SLAM (Lightning-LM onboard, or FAST-LIO2-class in our container)
        │        └── map→odom (or /lightning/odom)  ← replaces "slam_toolbox scan-matching"
        ├── pointcloud_to_laserscan → /scan          ← feeds Nav2 2D costmaps (cheap, robust)
        └── 3D costmap layer (STVL / elevation)      ← Plan B: sees ground pipes & low obstacles
                 └── Nav2 (Humble, containerized on robot or backpack PC) → /cmd_vel → vendor motion host
```

**Key consequences:**
1. **Our sim's `/scan` is the wrong interface.** The real robot gives a 3D PointCloud2
   on `/LIDAR/POINTS`; any 2D `/scan` is something WE derive (pointcloud_to_laserscan).
   → Sim change: publish an honest reduced-beam hemispherical PointCloud2 on
   `/LIDAR/POINTS` (mj_ray fan: 360° az × ~90° elevation, subsampled beams), then run the
   REAL `pointcloud_to_laserscan` node to get `/scan`. Same code path as the real robot.
2. **The 90° vertical FOV sees the ground.** The real sensor DOES see the ground
   pipelines our 2D ring misses — the gap is not the sensor, it's the 2D projection.
   Plan B = consume the 3D cloud in the costmap (STVL/elevation), not new hardware.
3. **2D slam_toolbox is a sim crutch.** Real localization will be LIO (Lightning-LM or
   FAST-LIO2 via XYZIRT). Our GPS+IMU+wheel EKF (phase A) remains right: fuse LIO odom
   + GPS in robot_localization exactly as planned — LIO replaces "SLAM pose", GPS bounds it.
4. **Compute budget is real.** RK3588, no GPU: rules out nvblox (CUDA); favors
   STVL/elevation mapping (CPU) and modest cloud rates. Humble runs in a container.
5. **Mount positions: RESOLVED (round 2).** The vendor publishes the merged cloud in one
   body frame with identity extrinsics — physical mount positions are handled inside the
   driver and never exposed. What we DO adopt from the configs: the 0.5 m near-body blind
   radius, and the `lidar_link`≈IMU-frame convention. In sim: one virtual sensor at the
   top-of-body (matching the merged-frame convention), blind radius 0.5 m; exact vertical
   tilt band remains ASSUMED (not published) — verify on hardware day 1 via
   `base_link→lidar_link` TF + a wall test.

See `docs/media/lidar_2d_vs_real_hemisphere.png` (generated by
`tools/nav/lidar_perspective.py`): same pose in the field — our current 2D ring returns
78 points and misses every ground pipe; the real hemispherical pattern (subsampled 24×120
of the 96-line pair) returns 2,153 points, paints the ground, pipes, skids and tanks.

## 3b. Sim realism status (updated 2026-07-15, sensor-realism update)

The hardware-vs-sim gaps identified below are now largely CLOSED in the sim
(`tools/mujoco_sim.py`, verified by TEST_PLAN L1.8, PASS 7/7):
- **XYZIRT** — modeled: rslidar field layout (intensity from per-material
  reflectivity, ring index, per-point float64 timestamps).
- **Motion distortion** — modeled *for real*: progressive sector casting (96 rays per
  200 Hz tick, 20 sectors = one 100 ms sweep from the moving robot's live poses).
  Measured: pipe line-fit RMS 3.6 cm parked → 22.7 cm at 1 rad/s. Assumption: single
  mechanical 10 Hz rotation (the real dual-unit pattern is unpublished).
- **Noise/weather** — modeled: σ=1.5 cm range noise, distance-scaled dropout, optional
  `M20_WEATHER=dust` phantom-wall mode. Point density remains reduced-beam
  (~19k pts/s vs real ~860k) — the real robot sends MORE data, not different data.
- **Still NOT modeled** (deliberate): multipath/ghost returns off reflective tanks —
  needs EM-level simulation; declared gap rather than a bad fake.

## 4. Field pitfalls to design for (from practitioner/deployment sources)
- Dust/rain/fog create phantom "walls" of noise points and shorten range — plan
  intensity/outlier filtering before costmaps (MDPI all-weather LiDAR evaluation).
- Highly reflective tanks can produce multipath/ghost returns; scan-match constraints
  weaken in sparse open pads (we already measured this: 0.82 m honest-drift error).
- Ground clutter below any single 2D plane is invisible — 3D layer is mandatory for
  oil & gas (pipes at 0.15–0.30 m).

## 5. Actions (feeds the D→A→B→C plan)
- **D (now):** keep 2D loop for reliability hardening, but rename sim topics to real
  ones (`/LIDAR/POINTS` + derived `/scan`) so all later work is on the real interface.
- **A:** robot_localization EKF = wheel + IMU + GPS (+ LIO odom later). Unchanged, confirmed right.
- **B:** hemispherical PointCloud2 in sim → STVL/elevation costmap layer → pipes become
  obstacles. (CPU-only, RK3588-compatible choice confirmed.)
- **Real robot bring-up:** Humble container/backpack PC; Lightning-LM onboard for LIO;
  rslidar_sdk if we ever need raw per-unit clouds; measure lidar TF on hardware day 1.

## Sources (primary)
- https://www.deeprobotics.us/products/lynx-m20/ · https://www.deeprobotics.cn/en/index/lynx.html · https://shop.deeprobotics.us/products/lynx-m20
- https://github.com/DeepRoboticsLab/lightning-lm-deep-robotics · https://github.com/DeepRoboticsLab/URDF_model · https://github.com/orgs/DeepRoboticsLab/repositories
- https://www.robosense.ai/en/IncrementalComponents/Airy · https://github.com/RoboSense-LiDAR/rslidar_sdk
- https://github.com/ros-perception/pointcloud_to_laserscan · https://github.com/SteveMacenski/spatio_temporal_voxel_layer · https://docs.nav2.org/setup_guides/sensors/mapping_localization.html
- https://www.mdpi.com/2218-6581/15/4/70 (quadruped slam_toolbox+Nav2 pattern, unverified) · https://www.mdpi.com/1424-8220/25/24/7436 (all-weather LiDAR)
