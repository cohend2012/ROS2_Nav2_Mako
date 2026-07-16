#!/usr/bin/env bash
# TERMINAL 1: bring up the full sim stack and STAY IN THE FOREGROUND.
# Keeping this terminal open is what keeps the stack alive (backgrounded
# processes don't survive a closed session on this box). Ctrl-C tears it all down.
#
#   Terminal 1:  wsl -d Ubuntu -e bash <repo>/tools/dev/sim_up.sh
#   Terminal 2:  wsl -d Ubuntu -e bash <repo>/tools/dev/m20sh     # then drive
set -e
C=docker-commander-1
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
MODEL="$HOME/m20_sim/sdk_deploy/src/M20_sdk_deploy/M20_description"
SRC="source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; export ROS_DOMAIN_ID=42"

# GUI: live MuJoCo viewer window via WSLg (default ON). M20_GUI=0 for headless.
GUI="${M20_GUI:-1}"

cleanup() {
  trap - INT TERM EXIT
  echo; echo "[sim_up] tearing down stack..."
  bash "$REPO/tools/nav/kill_stack.sh" >/dev/null 2>&1 || true
  echo "[sim_up] down. bye."
  exit 0
}
trap cleanup INT TERM EXIT

echo "[sim_up] cleaning any prior stack..."
bash "$REPO/tools/nav/kill_stack.sh" >/dev/null 2>&1 || true
sleep 1

echo "[sim_up] checking container deps..."
bash "$REPO/tools/dev/container_deps.sh" || exit 1

echo "[sim_up] staging configs..."
docker exec $C bash -lc "mkdir -p /cfg /cfg/out"
docker cp "$REPO/src/m20_locomotion_bridge/m20_locomotion_bridge/bridge_node.py" $C:/cfg/bridge_node.py >/dev/null
docker cp "$REPO/src/m20_navigation/config/nav2_params.yaml" $C:/cfg/nav2_params.yaml >/dev/null
docker cp "$REPO/tools/slam/mapper_params.yaml" $C:/cfg/mapper_params.yaml >/dev/null
docker cp "$REPO/src/m20_navigation/config/pointcloud_to_laserscan.yaml" $C:/cfg/pointcloud_to_laserscan.yaml >/dev/null

if [ "$GUI" = "1" ]; then
  echo "[sim_up] starting MuJoCo sim (oil_gas_field) WITH LIVE VIEWER — a window will open..."
  docker run -d --rm --name m20_sim_run --network host --ipc host \
    -e ROS_DOMAIN_ID=42 -e M20_SIM_GUI=1 \
    -e DISPLAY="${DISPLAY:-:0}" -e WAYLAND_DISPLAY="$WAYLAND_DISPLAY" \
    -e XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
    -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg \
    -e M20_MJCF=/model/m20_mjcf/mjcf/oil_gas_field.xml \
    -v "$MODEL":/model:ro -v "$REPO/tools/mujoco_sim.py":/mujoco_sim.py:ro \
    m20_sim:latest python3 /mujoco_sim.py >/dev/null
else
  echo "[sim_up] starting MuJoCo sim (oil_gas_field, headless — M20_GUI=0)..."
  docker run -d --rm --name m20_sim_run --network host --ipc host \
    -e ROS_DOMAIN_ID=42 -e M20_SIM_GUI=0 \
    -e M20_MJCF=/model/m20_mjcf/mjcf/oil_gas_field.xml \
    -v "$MODEL":/model:ro -v "$REPO/tools/mujoco_sim.py":/mujoco_sim.py:ro \
    m20_sim:latest python3 /mujoco_sim.py >/dev/null
fi
sleep 8
if ! docker ps --format '{{.Names}}' | grep -q '^m20_sim_run$'; then
  echo "[sim_up] SIM FAILED TO START — log:"; docker logs m20_sim_run 2>&1 | tail -5
  exit 1
fi

# static lidar TF published commander-local (cross-container transient_local latching
# proved unreliable after WSL reboots). 0.10 m = LIDAR_OFFSET in tools/mujoco_sim.py.
docker exec -d $C bash -lc "$SRC; ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.10 --frame-id base_link --child-frame-id lidar_link"
sleep 1
echo "[sim_up] starting pointcloud_to_laserscan (/LIDAR/POINTS -> /scan, real interface)..."
docker exec -d $C bash -lc "$SRC; ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r cloud_in:=/LIDAR/POINTS -r scan:=/scan --params-file /cfg/pointcloud_to_laserscan.yaml"
sleep 2

echo "[sim_up] starting bridge + arming..."
docker exec -d $C bash -lc "$SRC; python3 /cfg/bridge_node.py --ros-args -p sdk_backend:=sim"
sleep 4
docker exec $C bash -lc "$SRC; ros2 service call /m20/set_mode m20_msgs/srv/SetMode '{mode: 1, requester: sim_up}'" >/dev/null 2>&1 || true

echo "[sim_up] starting slam_toolbox..."
docker exec -d $C bash -lc "$SRC; ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file /cfg/mapper_params.yaml"
sleep 6

echo "[sim_up] starting Nav2..."
docker exec -d $C bash -lc "$SRC; ros2 launch nav2_bringup navigation_launch.py params_file:=/cfg/nav2_params.yaml use_sim_time:=false"
sleep 14

STATE=$(docker exec $C bash -lc "$SRC; ros2 lifecycle get /bt_navigator 2>&1" | tail -1)
echo
echo "=================================================================="
echo " STACK UP.  Nav2: $STATE"
if [ "$GUI" = "1" ]; then
  echo " A MuJoCo window with the robot should be on your screen."
  echo " CLOSING THAT WINDOW also shuts the whole stack down."
fi
echo " >>> LEAVE THIS TERMINAL OPEN <<<   (Ctrl-C here = shut it all down)"
echo " Drive from a 2nd terminal:  bash tools/dev/m20sh"
echo "=================================================================="
echo " (following sim log; the robot is idle until you command it)"
docker logs -f m20_sim_run || true
echo "[sim_up] sim exited (window closed?) — cleaning up the rest..."
