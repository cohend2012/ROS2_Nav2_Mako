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
# ============================================================================
set -e
C=docker-commander-1
DOM=42
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
MODEL="$HOME/m20_sim/sdk_deploy/src/M20_sdk_deploy/M20_description"
GOAL_X="${GOAL_X:--6.0}"; GOAL_Y="${GOAL_Y:--0.5}"
SRC="source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; export ROS_DOMAIN_ID=$DOM"

echo "[1/7] staging config + tools into $C:/cfg"
bash "$REPO/tools/dev/container_deps.sh" || exit 1
docker exec $C bash -lc "mkdir -p /cfg /cfg/out"
docker cp "$REPO/src/m20_locomotion_bridge/m20_locomotion_bridge/bridge_node.py" $C:/cfg/bridge_node.py
docker cp "$REPO/src/m20_navigation/config/nav2_params.yaml"                      $C:/cfg/nav2_params.yaml
docker cp "$REPO/tools/slam/mapper_params.yaml"                                   $C:/cfg/mapper_params.yaml
docker cp "$REPO/src/m20_navigation/config/pointcloud_to_laserscan.yaml"          $C:/cfg/pointcloud_to_laserscan.yaml
docker cp "$REPO/tools/nav/nav_logger.py"                                         $C:/cfg/nav_logger.py
docker cp "$REPO/tools/estimator.py"                                              $C:/cfg/estimator.py

echo "[2/7] starting MuJoCo sim (oil_gas_field, headless)"
docker rm -f m20_sim_run 2>/dev/null || true
docker run -d --name m20_sim_run --network host --ipc host \
  -e ROS_DOMAIN_ID=$DOM -e M20_SIM_GUI=0 \
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
  echo "[2.6/7] starting GPS-fused EKF estimator (wheel+IMU+GPS -> /odom_filtered)"
  docker exec -d $C bash -lc "$SRC; python3 /cfg/estimator.py"
fi

echo "[3/7] starting bridge (sim backend) + arming"
docker exec -d $C bash -lc "$SRC; python3 /cfg/bridge_node.py --ros-args -p sdk_backend:=sim"
sleep 4
docker exec $C bash -lc "$SRC; ros2 service call /m20/set_mode m20_msgs/srv/SetMode '{mode: 1, requester: nav_test}'" | tail -2

echo "[4/7] starting slam_toolbox"
docker exec -d $C bash -lc "$SRC; ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file /cfg/mapper_params.yaml"
sleep 6

echo "[5/7] starting Nav2"
docker exec -d $C bash -lc "$SRC; ros2 launch nav2_bringup navigation_launch.py params_file:=/cfg/nav2_params.yaml use_sim_time:=false"
sleep 14

echo "[6/7] logging + sending goal ($GOAL_X, $GOAL_Y)"
docker exec -d $C bash -lc "$SRC; RUN_SECS=120 python3 /cfg/nav_logger.py"
sleep 2
docker exec $C bash -lc "$SRC; ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y}, orientation: {z: 1.0, w: 0.0}}}}'" | tail -3

echo "[7/7] run complete. Copy results + render with:"
echo "  docker cp $C:/cfg/out tools/nav/out && OUTDIR=tools/nav/out python3 tools/nav/render_nav.py"
