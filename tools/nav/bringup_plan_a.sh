#!/usr/bin/env bash
# ============================================================================
# Plan A bringup: autonomous 2D navigation in the oil & gas field sim, using the
# PRODUCTION stack -- Nav2 (Regulated Pure Pursuit + NavFn/A*) + slam_toolbox.
#
#   sim (MuJoCo, m20_sim)  ->  /scan /odom /IMU + TF odom->base_link
#   bridge (m20_autonomy)  ->  /cmd_vel -> wheel/leg joint commands
#   slam_toolbox           ->  /map + TF map->odom  (scan-matching)
#   Nav2                   ->  global plan -> costmap-aware control -> /cmd_vel
#
# Honest scope (Plan A):
#   * Perception (/scan), map-building, global planning, obstacle avoidance and
#     path-following are ALL real (live 10 m LiDAR, costmaps, RPP replanning).
#   * Localization is currently fed by the sim's ground-truth odom TF, so
#     slam_toolbox's map->odom correction is ~identity. Honest dead-reckoned odom
#     (real drift for SLAM to correct) is the immediate Plan-A follow-on.
#   * The 2D scan (horizontal ring at z=0.35 m) does NOT see the ground pipelines
#     (tops at 0.30 m). Low-obstacle traversability is Plan B (3D perception).
#
# Prereqs: docker-commander-1 running (m20_autonomy image, ROS_DOMAIN_ID=42),
#          m20_sim:latest image, oil_gas_field.xml installed next to M20.xml.
# Usage:   bash tools/nav/bringup_plan_a.sh              # full run + verify
#          GOAL_X=-6 GOAL_Y=-0.5 bash tools/nav/bringup_plan_a.sh
# Flags (phase-map-loc):
#   M20_STATIC_MAP=maps/oil_gas_field  navigate against the PRE-BUILT map:
#       slam_toolbox localization mode (posegraph) + static-layer global costmap
#       (planner sees the whole field at t=0). Empty = live-SLAM path (master).
#   M20_STANDUP=1  spawn the sim FOLDED and rise via the commander-gated behavior
#       engine (standup, TIER_VENDOR) BEFORE the bridge starts. Real standup on
#       camera; joints logged for the replay render.
#   M20_NO_GOAL=1  bring the stack up but send no goal (mapping / coverage runs).
# Flags (3dgs-sim-env):
#   M20_SCENE=indoor_splat.xml  run a different world (any tools/sim/*.xml installed by
#       setup_sim.sh). Default oil_gas_field.xml. Non-default scenes have no pre-built
#       map (live SLAM only) and need explicit GOAL_X/GOAL_Y unless M20_NO_GOAL=1.
#   M20_SPLAT_CAMERA=1  photoreal /camera/image_raw rendered from the scene's Gaussian
#       splat (tools/splat/splat_camera_node.py, m20_splat image, GPU). Needs a
#       <scene>.scene.yaml next to the scene and the PLY in ~/m20_sim/splats/.
# ============================================================================
set -e
# FRAME CONTRACT GUARD (ship rule): M20_EKF_TF is a MAP-PRODUCTION tool only —
# the smooth GPS-pulled prior may never own odom->base_link while navigating
# against the shipped map. Runtime tree stays REP-105: sim dead-reckon owns
# odom->base_link, slam localization owns map->odom. See SECOND_BRAIN §3.
if [ "${M20_EKF_TF:-0}" = "1" ] && [ -n "${M20_STATIC_MAP:-}" ]; then
  echo "ERROR: M20_EKF_TF=1 with M20_STATIC_MAP is forbidden (mapping-pipeline tool leaking into runtime frames)"; exit 1
fi
C=docker-commander-1
DOM=42
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
MODEL="$HOME/m20_sim/sdk_deploy/src/M20_sdk_deploy/M20_description"
SCENE="${M20_SCENE:-oil_gas_field.xml}"
[ -f "$MODEL/m20_mjcf/mjcf/$SCENE" ] || { echo "ERROR: scene $SCENE not installed next to M20.xml (run tools/setup_sim.sh)"; exit 1; }
if [ "$SCENE" != "oil_gas_field.xml" ]; then
  # the only pre-built map and the default goal both belong to the oil & gas field
  [ -z "${M20_STATIC_MAP:-}" ] || { echo "ERROR: M20_STATIC_MAP is an oil_gas_field map; $SCENE runs live SLAM only"; exit 1; }
  [ "${M20_NO_GOAL:-0}" = "1" ] || [ -n "${GOAL_X:-}" ] || { echo "ERROR: $SCENE needs GOAL_X/GOAL_Y (the default goal is an oil_gas_field point) or M20_NO_GOAL=1"; exit 1; }
fi
GOAL_X="${GOAL_X:--6.0}"; GOAL_Y="${GOAL_Y:--0.5}"
SRC="source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; export ROS_DOMAIN_ID=$DOM RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"

# STALENESS ASSERTION (2026-07-28 lesson, L1.6): the image-baked commander
# predated the tip-over failsafe — an inverted robot stayed armed. The
# commander is PID 1 (not re-copyable like bridge/engine), so we ASSERT the
# installed copy matches the repo and refuse to run stale safety code.
# Override for deliberate old-image tests: M20_ALLOW_STALE=1.
if [ "${M20_ALLOW_STALE:-0}" != "1" ]; then
  REPO_MD5=$(md5sum "$REPO/src/m20_commander/m20_commander/commander_node.py" | cut -d" " -f1)
  INST_PATH=$(docker exec $C bash -c "find /ws -name commander_node.py 2>/dev/null | head -1")
  if [ -z "$INST_PATH" ]; then
    echo "ERROR: no commander_node.py found anywhere in /ws — image layout changed or container broken."
    exit 1
  fi
  INST_MD5=$(docker exec $C md5sum "$INST_PATH" | cut -d" " -f1)
  if [ "$REPO_MD5" != "$INST_MD5" ]; then
    echo "ERROR: installed commander != repo ($INST_PATH=$INST_MD5 repo=$REPO_MD5)."
    echo "       Rebuild the image (docker build + cyclone layer + recreate) or set M20_ALLOW_STALE=1."
    exit 1
  fi
fi

echo "[1/7] staging config + tools into $C:/cfg"
bash "$REPO/tools/dev/container_deps.sh" || exit 1
docker exec $C bash -lc "mkdir -p /cfg /cfg/out"
docker cp "$REPO/src/m20_locomotion_bridge/m20_locomotion_bridge/bridge_node.py" $C:/cfg/bridge_node.py
docker cp "$REPO/src/m20_navigation/config/nav2_params.yaml"                      $C:/cfg/nav2_params.yaml
docker cp "$REPO/tools/slam/mapper_params.yaml"                                   $C:/cfg/mapper_params.yaml
docker cp "$REPO/src/m20_navigation/config/pointcloud_to_laserscan.yaml"          $C:/cfg/pointcloud_to_laserscan.yaml
docker cp "$REPO/tools/nav/nav_logger.py"                                         $C:/cfg/nav_logger.py
docker cp "$REPO/tools/estimator.py"                                              $C:/cfg/estimator.py
docker cp "$REPO/src/m20_missions/m20_missions/mission_server.py"                 $C:/cfg/mission_server.py

if [ -n "${M20_STATIC_MAP:-}" ]; then
  echo "[1.5/7] static-map mode: staging pre-built map + localization config"
  [ -f "$REPO/$M20_STATIC_MAP.posegraph" ] || { echo "ERROR: $M20_STATIC_MAP.posegraph not found (run tools/map/build_map.sh first)"; exit 1; }
  docker exec $C bash -lc "mkdir -p /cfg/maps"
  docker cp "$REPO/$M20_STATIC_MAP.posegraph" $C:/cfg/maps/oil_gas_field.posegraph
  docker cp "$REPO/$M20_STATIC_MAP.data"      $C:/cfg/maps/oil_gas_field.data
  docker cp "$REPO/tools/slam/localization_params.yaml" $C:/cfg/localization_params.yaml
  # Global costmap: rolling window -> full-field static layer from the known map.
  # Patch the STAGED copy only — nav2_params.yaml stays single-source for master.
  docker exec $C python3 - <<'PYEOF'
import yaml
p = "/cfg/nav2_params.yaml"
cfg = yaml.safe_load(open(p))
g = cfg["global_costmap"]["global_costmap"]["ros__parameters"]
g["rolling_window"] = False
g.pop("width", None); g.pop("height", None)
g["track_unknown_space"] = True
g["plugins"] = ["static_layer", "obstacle_layer", "inflation_layer"]
g["static_layer"] = {"plugin": "nav2_costmap_2d::StaticLayer",
                     "map_subscribe_transient_local": True,
                     "map_topic": "/map"}
yaml.safe_dump(cfg, open(p, "w"), sort_keys=False)
print("patched global costmap -> static layer (full known field)")
PYEOF
fi

if [ "$SCENE" != "oil_gas_field.xml" ]; then
  # Splat/indoor scenes: faster cruise for exploration tours. STAGED copy only — the
  # oil & gas gates keep nav2_params.yaml as-is. Measured 2026-10-05 (splat demo): 1.0-1.6
  # m/s with 2.0 m/s^2 accel TIPPED the M20 — flooring it out of a tight spot while still
  # turning makes the skid-steer fishtail (uncommanded -1.2 rad/s yaw, body lifts, rolls).
  # Keep the ORIGINAL accel (1.0) and a modest cruise; DWB still turns in place.
  # Goal tolerance back to the live-SLAM value 0.45 (nav2_params.yaml history note:
  # 0.20 assumes known-map localization; live SLAM jitters -> arrive-never-declare).
  # Footprint: the real M20 rectangle (0.86 x 0.56 m incl. wheels) instead of the 0.45 m
  # circle — its corners reach 0.51 m, so the circle let them scrape boxes at speed.
  VMAX="${M20_MAX_VEL:-0.7}"; AMAX="${M20_MAX_ACC:-1.0}"
  echo "[1.6/7] ${SCENE%.xml}: cruise ${VMAX} m/s, accel ${AMAX} / decel 1.5, M20 footprint, keep-away inflation 1.0 m, goal tol 0.45 m (staged params only)"
  # -i is REQUIRED: without it docker exec drops stdin and `python3 -` runs an empty script
  docker exec -i -e VMAX="$VMAX" -e AMAX="$AMAX" $C python3 - <<'PYEOF'
import os, yaml
p = "/cfg/nav2_params.yaml"
v = float(os.environ["VMAX"])
cfg = yaml.safe_load(open(p))
c = cfg["controller_server"]["ros__parameters"]
f = c["FollowPath"]
f["max_vel_x"] = f["max_speed_xy"] = v
acc = float(os.environ["AMAX"])
f["acc_lim_x"], f["decel_lim_x"] = acc, -1.5
f["xy_goal_tolerance"] = c["goal_checker"]["xy_goal_tolerance"] = 0.45
s = cfg["velocity_smoother"]["ros__parameters"]
s["max_velocity"][0] = v
s["max_accel"][0], s["max_decel"][0] = acc, -1.5
fp = "[[0.43, 0.28], [0.43, -0.28], [-0.43, -0.28], [-0.43, 0.28]]"
# Keep-away (2026-10-05): the stock 0.55 m inflation decayed fast (scaling 3.0) and DWB
# barely weighed obstacle cost (BaseObstacle 0.02) -> laps spent 10-18 % within 0.3 m of
# boxes (closest 8 cm). Wider, slower-decaying inflation + a real obstacle weight make
# the planner use aisle centres and the controller steer away from cost.
for k in ("global_costmap", "local_costmap"):
    cm = cfg[k][k]["ros__parameters"]
    cm.pop("robot_radius", None)
    cm["footprint"], cm["footprint_padding"] = fp, 0.10
    cm["inflation_layer"]["inflation_radius"] = 1.0
    cm["inflation_layer"]["cost_scaling_factor"] = 1.5
f["BaseObstacle.scale"] = 0.15
# Anti-dither (2026-10-05): with keep-away costs DWB's best option in tight spots was
# "vx 0, wz +-0.04" — too small for the skid-steer to turn (bridge pivot assist starts at
# 0.15 rad/s), so the robot sat 30-160 s until the progress checker fired. DWB rejects any
# command below BOTH min_speed_xy and min_speed_theta: it must drive or really turn.
f["min_speed_xy"], f["min_speed_theta"] = 0.10, 0.25
# ...and if it still stalls, recover sooner: 40 s dated from slow 15-20 s pivots; a 180 deg
# turn now takes ~5 s (0.7 rad/s), so 15 s still never aborts a legitimate in-place turn.
c["progress_checker"]["movement_time_allowance"] = 15.0
yaml.safe_dump(cfg, open(p, "w"), sort_keys=False)
print(f"  patched: max_vel_x={v}, accel {acc}/-1.5, footprint 0.86x0.56 +0.10, inflation 1.0 (scale 1.5), BaseObstacle 0.15, goal tol 0.45")
PYEOF
fi

# the splat camera replaces the MuJoCo camera (one /camera/image_raw publisher)
SIM_CAMERA="${M20_CAMERA:-0}"; [ "${M20_SPLAT_CAMERA:-0}" = "1" ] && SIM_CAMERA=0
echo "[2/7] starting MuJoCo sim (${SCENE%.xml}, headless, start=$([ "${M20_STANDUP:-0}" = 1 ] && echo folded || echo stand))"
docker rm -f m20_sim_run 2>/dev/null || true
docker run -d --name m20_sim_run --network host --ipc host \
  -e ROS_DOMAIN_ID=$DOM -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e M20_SIM_GUI=0 -e M20_LIDAR_REALISM=${M20_LIDAR_REALISM:-1} \
  -e M20_START_POSE=${M20_STANDUP:+folded} \
  -e M20_CAMERA=$SIM_CAMERA -e MUJOCO_GL=egl \
  -e M20_NO_ODOM_TF=${M20_EKF_TF:-0} -e M20_UPRIGHT_ASSIST=${M20_UPRIGHT_ASSIST:-0} -e M20_GT_ODOM=${M20_GT_LOC:-0} \
  -e M20_MJCF=/model/m20_mjcf/mjcf/$SCENE \
  -v "$MODEL":/model:ro -v "$REPO/tools/mujoco_sim.py":/mujoco_sim.py:ro \
  m20_sim:latest python3 /mujoco_sim.py
sleep 8

if [ "${M20_SPLAT_CAMERA:-0}" = "1" ]; then
  SCENE_YAML="$REPO/tools/sim/${SCENE%.xml}.scene.yaml"
  [ -f "$SCENE_YAML" ] || { echo "ERROR: M20_SPLAT_CAMERA=1 but $SCENE has no scene yaml (not a splat world)"; exit 1; }
  SPLAT_FILE=$(awk '/^  file:/{print $2; exit}' "$SCENE_YAML")
  [ -f "$HOME/m20_sim/splats/$SPLAT_FILE" ] || { echo "ERROR: splat ~/m20_sim/splats/$SPLAT_FILE missing (see RUN_GUIDE 'Splat worlds')"; exit 1; }
  echo "[2.2/7] starting splat camera (gsplat on GPU, $SPLAT_FILE -> /camera/image_raw)"
  docker rm -f m20_splat_cam 2>/dev/null || true
  docker run -d --name m20_splat_cam --gpus all --network host --ipc host \
    -e ROS_DOMAIN_ID=$DOM -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e SPLAT_PLY=/splat/$SPLAT_FILE -e SPLAT_SCENE=/scene/$(basename "$SCENE_YAML") \
    -v "$HOME/m20_sim/splats":/splat:ro -v "$REPO/tools/sim":/scene:ro \
    -v "$REPO/tools/splat/splat_camera_node.py":/splat_camera_node.py:ro \
    m20_splat:latest python3 /splat_camera_node.py
fi

echo "[2.4/7] static TF base_link->lidar_link (commander-local; cross-container"
echo "        transient_local latching proved unreliable after WSL reboots)"
# 0.10 m z-offset = LIDAR_OFFSET in tools/mujoco_sim.py (keep in sync)
docker exec -d $C bash -lc "$SRC; ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.10 --frame-id base_link --child-frame-id lidar_link"
sleep 1

echo "[2.5/7] starting pointcloud_to_laserscan (/LIDAR/POINTS -> /scan)"
docker exec -d $C bash -lc "$SRC; ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r cloud_in:=/LIDAR/POINTS -r scan:=/scan --params-file /cfg/pointcloud_to_laserscan.yaml"
sleep 2

if [ "${M20_EKF:-0}" = "1" ]; then
  echo "[2.6/7] starting GPS-fused EKF estimator (tf=${M20_EKF_TF:-0} — 1 = GPS-anchored mapping)"
  docker exec -d $C bash -lc "$SRC; export M20_EKF_TF=${M20_EKF_TF:-0}; python3 /cfg/estimator.py >/cfg/out/estimator.log 2>&1"
fi

if [ "${M20_STATION:-1}" = "1" ]; then
  echo "[2.7/7] starting operator station (Foxglove ws://localhost:8765, rosbridge :9090)"
  docker exec -d $C bash -lc "$SRC; ros2 launch m20_station station_bridge.launch.py >/cfg/out/station.log 2>&1"
fi

if [ "${M20_STANDUP:-0}" = "1" ]; then
  echo "[3/7] commander-gated STANDUP (folded -> standing via behavior engine), then bridge"
  # arm in ASSISTED (2); if the commander is stuck in ESTOP, go through IDLE first
  OK=$(docker exec $C bash -lc "$SRC; ros2 service call /m20/set_mode m20_msgs/srv/SetMode '{mode: 2, requester: bringup_standup}'" | grep -c "accepted=True" || true)
  if [ "$OK" = "0" ]; then
    docker exec $C bash -lc "$SRC; ros2 service call /m20/set_mode m20_msgs/srv/SetMode '{mode: 0, requester: bringup_standup}'" | tail -1
    docker exec $C bash -lc "$SRC; ros2 service call /m20/set_mode m20_msgs/srv/SetMode '{mode: 2, requester: bringup_standup}'" | tail -1
  fi
  # behavior engine from the REPO source (same freshness rule as bridge_node.py —
  # the image-baked copy burned us once already); m20_msgs/drdds come from /ws
  docker exec $C bash -lc "rm -rf /cfg/m20_behaviors"
  docker cp "$REPO/src/m20_behaviors/m20_behaviors" $C:/cfg/m20_behaviors
  docker exec -d $C bash -lc "$SRC; PYTHONPATH=/cfg:\$PYTHONPATH python3 -c 'from m20_behaviors.engine import main; main()' >/cfg/out/engine.log 2>&1"
  sleep 3
  # logger starts BEFORE the standup so joints.csv captures the rise
  docker exec -d $C bash -lc "$SRC; RUN_SECS=${RUN_SECS:-300} python3 /cfg/nav_logger.py >/cfg/out/logger.log 2>&1"
  sleep 2
  # request + wait in ONE node (no pub/echo race): subscribe status, wait for the
  # engine to match, publish the request, then wait for a terminal state.
  # STATE_SUCCEEDED=3, ABORTED=4, FAILED=5; 25 s hard stop.
  ST=$(docker exec $C bash -lc "$SRC; timeout 25 python3 - <<'PYEOF'
import rclpy
from m20_msgs.msg import BehaviorRequest, BehaviorStatus
rclpy.init(); n = rclpy.create_node('standup_request')
done = []
n.create_subscription(BehaviorStatus, '/m20/behavior/status',
    lambda m: done.append(m.state) if m.state in (3, 4, 5) else None, 10)
pub = n.create_publisher(BehaviorRequest, '/m20/behavior/request', 10)
while rclpy.ok() and pub.get_subscription_count() == 0:
    rclpy.spin_once(n, timeout_sec=0.2)
req = BehaviorRequest(); req.behavior_name = 'standup'; req.requester = 'bringup'
pub.publish(req)
while rclpy.ok() and not done:
    rclpy.spin_once(n, timeout_sec=0.2)
print(done[0] if done else 'timeout')
PYEOF" | tail -1)
  [ "$ST" = "3" ] || { echo "ERROR: standup did not succeed (state=$ST)"; exit 1; }
  echo "  standup SUCCEEDED — starting bridge (holds stance) + MODE_AUTONOMOUS"
  docker exec -d $C bash -lc "$SRC; python3 /cfg/bridge_node.py --ros-args -p sdk_backend:=sim"
  sleep 4
  docker exec $C bash -lc "$SRC; ros2 service call /m20/set_mode m20_msgs/srv/SetMode '{mode: 3, requester: bringup_standup}'" | tail -2
else
  echo "[3/7] starting bridge (sim backend) + arming"
  docker exec -d $C bash -lc "$SRC; python3 /cfg/bridge_node.py --ros-args -p sdk_backend:=sim"
  sleep 4
  docker exec $C bash -lc "$SRC; ros2 service call /m20/set_mode m20_msgs/srv/SetMode '{mode: 1, requester: nav_test}'" | tail -2
fi

if [ -n "${M20_STATIC_MAP:-}" ]; then
  echo "[4/7] starting slam_toolbox in LOCALIZATION mode (pre-built posegraph)"
  docker exec -d $C bash -lc "$SRC; ros2 run slam_toolbox localization_slam_toolbox_node --ros-args --params-file /cfg/localization_params.yaml >/cfg/out/slam.log 2>&1"
  # SEED the localizer (2026-08-04 leg-1 root cause): map_start_at_dock is
  # SILENTLY UNSUPPORTED in localization mode ("correctly not supported" in
  # slam.log) — slam started UNSEEDED every mission; sometimes it converged
  # from identity, sometimes leg 1 never localized and the robot never moved.
  # Robot spawns at the map origin in sim; hardware will seed from GPS.
  docker exec $C bash -lc "$SRC; timeout 20 python3 - <<'PYEOF'
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
rclpy.init(); n = rclpy.create_node('loc_seed')
pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
while rclpy.ok() and pub.get_subscription_count() == 0:
    rclpy.spin_once(n, timeout_sec=0.2)
p = PoseWithCovarianceStamped()
p.header.frame_id = 'map'
p.header.stamp = n.get_clock().now().to_msg()
p.pose.pose.orientation.w = 1.0
p.pose.covariance[0] = p.pose.covariance[7] = 0.25
p.pose.covariance[35] = 0.05
pub.publish(p)
import time; time.sleep(0.5)
print('localization seeded at spawn')
PYEOF" | tail -1
  if [ "${M20_GUARDIAN:-1}" = "1" ]; then
    echo "[4.5/7] starting anchor_guardian (mission drift bounding; M20_GUARDIAN=0 disables)"
    docker cp "$REPO/tools/nav/anchor_guardian.py" $C:/cfg/anchor_guardian.py
    docker exec -d $C bash -lc "$SRC; python3 /cfg/anchor_guardian.py >/cfg/out/guardian.log 2>&1"
  fi
elif [ "${M20_GT_LOC:-0}" = "1" ]; then
  # DEMO-ONLY ground-truth localization: the sim publishes odom = truth (M20_GT_ODOM),
  # map->odom is a fixed identity, and no SLAM runs. Nav2 can't mislocalize; obstacles
  # still come from the live LiDAR (rolling costmaps). NOT for gates / evidence runs.
  echo "[4/7] GROUND-TRUTH localization (demo only): static map->odom, no slam_toolbox"
  docker exec -d $C bash -lc "$SRC; ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --frame-id map --child-frame-id odom"
else
  echo "[4/7] starting slam_toolbox (mapping)"
  docker exec -d $C bash -lc "$SRC; ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file /cfg/mapper_params.yaml >/cfg/out/slam.log 2>&1"
fi
sleep 6

echo "[5/7] starting Nav2 (logs -> /cfg/out/nav2.log; the 2026-07-17 lesson)"
docker exec -d $C bash -lc "$SRC; ros2 launch nav2_bringup navigation_launch.py params_file:=/cfg/nav2_params.yaml use_sim_time:=false >/cfg/out/nav2.log 2>&1"
sleep 14

echo "[5.5/7] starting mission server (/m20/mission/run — tools/dev/m20 goto|mission)"
docker exec -d $C bash -lc "$SRC; python3 /cfg/mission_server.py >/cfg/out/mission.log 2>&1"

if [ "${M20_STANDUP:-0}" != "1" ]; then
  echo "[6a/7] starting logger (standup path started it earlier)"
  docker exec -d $C bash -lc "$SRC; RUN_SECS=${RUN_SECS:-300} python3 /cfg/nav_logger.py >/cfg/out/logger.log 2>&1"
  sleep 2
fi
if [ "${M20_NO_GOAL:-0}" = "1" ]; then
  echo "[6/7] M20_NO_GOAL=1 — stack up, no goal sent (mapping/coverage run)"
else
  echo "[6/7] sending goal ($GOAL_X, $GOAL_Y)"
  docker exec $C bash -lc "$SRC; ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    '{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y}, orientation: {z: 1.0, w: 0.0}}}}'" | tail -3
fi

echo "[7/7] run complete. Copy results + render with:"
echo "  docker cp $C:/cfg/out tools/nav/out && OUTDIR=tools/nav/out python3 tools/nav/render_nav.py"
