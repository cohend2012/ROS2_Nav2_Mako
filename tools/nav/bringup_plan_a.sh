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
#   M20_AI_CONTROLLER=follow_path|standalone|direct  (M20-Brain L1, docs/AI_NAV_BRIEF.md)
#       follow_path: Nav2 WITHOUT controller_server; the learned PPO policy serves
#                    /follow_path (hybrid under Nav2's planner/BT/recoveries)
#       standalone : NO Nav2; our A* on /map + learned policy serve /navigate_to_pose
#       direct     : NO Nav2, no map; learned policy chases the goal, avoids reactively
#       Empty = classical Nav2 DWB (master behavior unchanged).
#   M20_AGENT=claude|cosmos|offline  start the language mission agent (/m20/agent/command).
#       claude needs ANTHROPIC_API_KEY in the container env; cosmos needs
#       M20_COSMOS_BASE_URL + M20_COSMOS_MODEL. Gauge reads need M20_CAMERA=1 on the sim.
#   M20_SITE=office  indoor profile: Nav2 gets the real 0.91 x 0.53 m footprint (doors
#       plannable), 0.35 m inflation and a 0.3 m/s cap (nav2_office_overlay.yaml, merged
#       into the STAGED params only); the AI controller plans on 0.32 m clearance; the
#       agent loads office_landmarks.yaml + a taught-places file. Default: outdoor.
#   M20_NAV_VARIANT=dwb|shim_rpp|lattice_rpp|mppi  (with M20_SITE=office) classical
#       Nav2 variant layered on the office overlay (config/office_variants/README.md).
#   M20_STANDUP=1  spawn the sim FOLDED and rise via the commander-gated behavior
#       engine (standup, TIER_VENDOR) BEFORE the bridge starts. Real standup on
#       camera; joints logged for the replay render.
#   M20_NO_GOAL=1  bring the stack up but send no goal (mapping / coverage runs).
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
SITE=${M20_SITE:-outdoor}
case "$SITE" in outdoor|office) ;; *) echo "ERROR: M20_SITE must be outdoor|office (got '$SITE')"; exit 1;; esac
if [ "$SITE" = "office" ] && [ "${M20_AI_CONTROLLER:-}" = "direct" ]; then
  echo "ERROR: M20_AI_CONTROLLER=direct is refused indoors (no map -> cannot route through doors)"; exit 1
fi
LANDMARKS_FILE=oil_gas_field_landmarks.yaml
if [ "$SITE" = "office" ]; then
  echo "[1.1/7] office profile: merging nav2_office_overlay.yaml into the staged nav2_params"
  docker cp "$REPO/src/m20_navigation/config/nav2_office_overlay.yaml" $C:/cfg/nav2_office_overlay.yaml
  docker cp "$REPO/tools/nav/apply_overlay.py" $C:/cfg/apply_overlay.py
  VARIANT_CFG=""
  if [ -n "${M20_NAV_VARIANT:-}" ] && [ "$M20_NAV_VARIANT" != "dwb" ]; then
    VF="$REPO/src/m20_navigation/config/office_variants/$M20_NAV_VARIANT.yaml"
    [ -f "$VF" ] || { echo "ERROR: M20_NAV_VARIANT=$M20_NAV_VARIANT: no $VF (see office_variants/README.md)"; exit 1; }
    docker cp "$VF" $C:/cfg/nav2_office_variant.yaml; VARIANT_CFG=/cfg/nav2_office_variant.yaml
    echo "[1.1/7] classical variant: $M20_NAV_VARIANT"
  fi
  docker exec $C bash -lc "$SRC; python3 /cfg/apply_overlay.py /cfg/nav2_params.yaml /cfg/nav2_office_overlay.yaml $VARIANT_CFG /cfg/nav2_params.yaml" \
    || { echo "ERROR: office overlay failed to merge"; exit 1; }
  LANDMARKS_FILE=${M20_LANDMARKS:-office_landmarks.yaml}
fi
if [ -n "${M20_AI_CONTROLLER:-}${M20_AGENT:-}" ]; then
  echo "[1.2/7] M20-Brain: staging m20_ai_nav + m20_agent into $C:/cfg"
  case "${M20_AI_CONTROLLER:-}" in ""|follow_path|standalone|direct) ;; *)
    echo "ERROR: M20_AI_CONTROLLER must be follow_path|standalone|direct (got '$M20_AI_CONTROLLER')"; exit 1;; esac
  [ -z "${M20_AI_CONTROLLER:-}" ] || [ -f "$REPO/src/m20_ai_nav/models/m20_nav_policy.onnx" ] || {
    echo "ERROR: src/m20_ai_nav/models/m20_nav_policy.onnx missing (train + export: tools/ai_nav/)"; exit 1; }
  docker exec $C bash -lc "rm -rf /cfg/m20_ai_nav /cfg/m20_agent"
  docker cp "$REPO/src/m20_ai_nav" $C:/cfg/m20_ai_nav
  docker cp "$REPO/src/m20_agent"  $C:/cfg/m20_agent
  docker exec $C bash -lc "python3 -c 'import onnxruntime, anthropic, openai, yaml, PIL' 2>/dev/null || pip3 install -q onnxruntime==1.23.2 'anthropic>=2.4,<3' 'openai>=1.60,<4' pyyaml pillow" \
    || { echo "ERROR: could not install M20-Brain python deps in $C"; exit 1; }
fi

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

echo "[2/7] starting MuJoCo sim (oil_gas_field, headless, start=$([ "${M20_STANDUP:-0}" = 1 ] && echo folded || echo stand))"
docker rm -f m20_sim_run 2>/dev/null || true
docker run -d --name m20_sim_run --network host --ipc host \
  -e ROS_DOMAIN_ID=$DOM -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e M20_SIM_GUI=0 -e M20_LIDAR_REALISM=${M20_LIDAR_REALISM:-1} \
  -e M20_START_POSE=${M20_STANDUP:+folded} \
  -e M20_CAMERA=${M20_CAMERA:-0} -e MUJOCO_GL=egl \
  -e M20_NO_ODOM_TF=${M20_EKF_TF:-0} \
  -e M20_MJCF=/model/m20_mjcf/mjcf/oil_gas_field.xml \
  -v "$MODEL":/model:ro -v "$REPO/tools/mujoco_sim.py":/mujoco_sim.py:ro \
  m20_sim:latest python3 /mujoco_sim.py
sleep 8

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
else
  echo "[4/7] starting slam_toolbox (mapping)"
  docker exec -d $C bash -lc "$SRC; ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file /cfg/mapper_params.yaml >/cfg/out/slam.log 2>&1"
fi
sleep 6

if [ -n "${M20_AI_CONTROLLER:-}" ]; then
  echo "[5/7] starting M20-Brain navigator mode=$M20_AI_CONTROLLER (logs -> /cfg/out/ai_nav.log)"
  docker exec -d $C bash -lc "$SRC; ros2 launch /cfg/m20_ai_nav/launch/ai_nav.launch.py mode:=$M20_AI_CONTROLLER site:=$SITE params_file:=/cfg/nav2_params.yaml model_path:=/cfg/m20_ai_nav/models/m20_nav_policy.onnx >/cfg/out/ai_nav.log 2>&1"
else
  echo "[5/7] starting Nav2 (logs -> /cfg/out/nav2.log; the 2026-07-17 lesson)"
  docker exec -d $C bash -lc "$SRC; ros2 launch nav2_bringup navigation_launch.py params_file:=/cfg/nav2_params.yaml use_sim_time:=false >/cfg/out/nav2.log 2>&1"
fi
sleep 14

echo "[5.5/7] starting mission server (/m20/mission/run — tools/dev/m20 goto|mission)"
docker exec -d $C bash -lc "$SRC; python3 /cfg/mission_server.py >/cfg/out/mission.log 2>&1"
if [ -n "${M20_AGENT:-}" ]; then
  echo "[5.6/7] starting M20-Brain agent backend=$M20_AGENT (send: tools/agent/say.sh '<request>')"
  docker exec -d $C bash -lc "$SRC; cd /cfg/m20_agent && ANTHROPIC_API_KEY='${ANTHROPIC_API_KEY:-}' M20_COSMOS_BASE_URL='${M20_COSMOS_BASE_URL:-}' M20_COSMOS_MODEL='${M20_COSMOS_MODEL:-}' python3 -m m20_agent.agent_node --ros-args -p backend:=$M20_AGENT -p landmarks:=/cfg/m20_agent/config/$LANDMARKS_FILE -p taught_places:=/cfg/out/agent/taught_places.yaml -p report_dir:=/cfg/out/agent >/cfg/out/agent.log 2>&1"
fi

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
