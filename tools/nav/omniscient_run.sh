#!/usr/bin/env bash
# ONE nav run with every signal recorded: mode, failsafes, cmd_vel, bridge log,
# nav2 log, plan, positions. The run that ends the guessing.
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
docker cp "$R/src/m20_locomotion_bridge/m20_locomotion_bridge/bridge_node.py" $C:/cfg/bridge_node.py >/dev/null
docker cp "$R/src/m20_navigation/config/nav2_params.yaml" $C:/cfg/nav2_params.yaml >/dev/null
docker cp "$R/tools/slam/mapper_params.yaml" $C:/cfg/mapper_params.yaml >/dev/null
docker cp "$R/src/m20_navigation/config/pointcloud_to_laserscan.yaml" $C:/cfg/pointcloud_to_laserscan.yaml >/dev/null
docker exec -d $C bash -lc "$S; ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.10 --frame-id base_link --child-frame-id lidar_link"
sleep 1
docker exec -d $C bash -lc "$S; ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r cloud_in:=/LIDAR/POINTS -r scan:=/scan --params-file /cfg/pointcloud_to_laserscan.yaml > /tmp/o_pc2ls.log 2>&1"
sleep 2
docker exec -d $C bash -lc "$S; python3 /cfg/bridge_node.py --ros-args -p sdk_backend:=sim > /tmp/o_bridge.log 2>&1"
sleep 4
docker exec $C bash -lc "$S; ros2 service call /m20/set_mode m20_msgs/srv/SetMode '{mode: 1, requester: omni}'" | grep -o "accepted=[a-z]*"
docker exec -d $C bash -lc "$S; ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file /cfg/mapper_params.yaml > /tmp/o_slam.log 2>&1"
sleep 6
docker exec -d $C bash -lc "$S; ros2 launch nav2_bringup navigation_launch.py params_file:=/cfg/nav2_params.yaml use_sim_time:=false > /tmp/o_nav2.log 2>&1"
sleep 14
# recorders
docker exec -d $C bash -lc "$S; timeout 150 ros2 topic echo /m20/mode > /tmp/o_mode.txt 2>&1"
docker exec -d $C bash -lc "$S; timeout 150 ros2 topic echo /m20/failsafe > /tmp/o_fs.txt 2>&1"
docker exec -d $C bash -lc "$S; timeout 150 ros2 topic echo /plan --field poses > /tmp/o_plan.txt 2>&1"
docker exec -d $C bash -lc "$S; timeout 150 ros2 topic echo /odom_true --field pose.pose.position > /tmp/o_pos.txt 2>&1"
sleep 2
docker exec $C bash -lc "$S; timeout 140 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: map}, pose: {position: {x: -6.0, y: -0.5}, orientation: {z: 1.0, w: 0.0}}}}' 2>&1 | tail -2"

echo "=========== VERDICTS ==========="
echo "-- mode transitions (armed flips):"
docker exec $C bash -c "grep -E 'mode:|armed:' /tmp/o_mode.txt | uniq -c | head -12"
echo "-- failsafes raised:"
docker exec $C bash -c "grep -E 'active|severity|detail' /tmp/o_fs.txt | sort | uniq -c | sort -rn | head -6"
echo "-- plan messages seen:"
docker exec $C bash -c "grep -c 'position' /tmp/o_plan.txt || echo 0"
echo "-- bridge log:"
docker exec $C bash -c "tail -5 /tmp/o_bridge.log"
echo "-- nav2 aborts/errors:"
docker exec $C bash -c "grep -iE 'error|abort|fail' /tmp/o_nav2.log | grep -viE 'declare|bond' | tail -8"
echo "-- final position:"
docker exec $C bash -c "tail -4 /tmp/o_pos.txt"
