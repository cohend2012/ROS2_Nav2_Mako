#!/usr/bin/env bash
# GPS-video capture run, one session: sim + bridge + estimator (GPS EKF) + logger,
# then a scripted 90 s tour of the field via /cmd_vel (drive segments + gentle arcs
# — no Nav2 involved). Logs: truth, dead-reckon, GPS fixes, EKF estimate.
set -u
R=/mnt/c/GIT/StatefullXOne/m20_autonomy_ws/m20_autonomy_ws
MODEL=$HOME/m20_sim/sdk_deploy/src/M20_sdk_deploy/M20_description
S="source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; export ROS_DOMAIN_ID=42"
C=docker-commander-1

bash $R/tools/nav/kill_stack.sh >/dev/null 2>&1; sleep 2
docker run -d --name m20_sim_run --network host --ipc host \
  -e ROS_DOMAIN_ID=42 -e M20_SIM_GUI=0 \
  -e M20_MJCF=/model/m20_mjcf/mjcf/oil_gas_field.xml \
  -v "$MODEL":/model:ro -v "$R/tools/mujoco_sim.py":/mujoco_sim.py:ro \
  m20_sim:latest python3 /mujoco_sim.py >/dev/null 2>&1
sleep 8
docker exec $C bash -lc "mkdir -p /cfg/out && rm -f /cfg/out/*.csv /cfg/out/*.npz"
docker cp "$R/src/m20_locomotion_bridge/m20_locomotion_bridge/bridge_node.py" $C:/cfg/bridge_node.py >/dev/null
docker cp "$R/tools/estimator.py" $C:/cfg/estimator.py >/dev/null
docker cp "$R/tools/nav/nav_logger.py" $C:/cfg/nav_logger.py >/dev/null
docker exec -d $C bash -lc "$S; python3 /cfg/bridge_node.py --ros-args -p sdk_backend:=sim > /tmp/bridge_gps.log 2>&1"
sleep 4
docker exec $C bash -lc "$S; ros2 service call /m20/set_mode m20_msgs/srv/SetMode '{mode: 1, requester: gpsvid}'" | grep -o "accepted=[a-z]*"
docker exec -d $C bash -lc "$S; python3 /cfg/estimator.py > /tmp/est.log 2>&1"
sleep 2
docker exec -d $C bash -lc "$S; RUN_SECS=95 python3 /cfg/nav_logger.py > /tmp/logger.log 2>&1"
sleep 2

drive() {  # drive vx wz secs
  docker exec $C bash -lc "$S; timeout $3 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: $1}, angular: {z: $2}}' >/dev/null 2>&1" || true
}
echo "[tour] west leg..."
drive 0.35 0.06 22          # long gentle-left drive toward the tanks
echo "[tour] arc south..."
drive 0.30 -0.28 14         # arc left-hand down
echo "[tour] east leg..."
drive 0.35 -0.06 22         # return drive
echo "[tour] arc north..."
drive 0.30 0.28 14          # arc back up
echo "[tour] final leg..."
drive 0.30 0.0 12
drive 0.0 0.0 3
sleep 8                      # let logger finish RUN_SECS
mkdir -p $R/tools/nav/out_gps
docker cp $C:/cfg/out/. $R/tools/nav/out_gps/ >/dev/null
echo "== logger summary =="; docker exec $C bash -c "tail -2 /tmp/logger.log"
ls $R/tools/nav/out_gps/
bash $R/tools/nav/kill_stack.sh >/dev/null 2>&1
