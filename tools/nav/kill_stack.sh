#!/usr/bin/env bash
# Tear down the full sim stack: Nav2 + slam_toolbox + bridge + logger (in the
# commander container) and the MuJoCo sim container. Leaves docker-commander-1 up.
C=docker-commander-1
PAT="navigation_launch|nav2_smoother|nav2_planner|nav2_behaviors|nav2_bt_navigator|nav2_waypoint|nav2_velocity|nav2_lifecycle|nav2_controller|component_container|slam_toolbox|bridge_node|nav_logger|pointcloud_to_laserscan|static_transform_publisher"
docker exec $C bash -c "
  PIDS=\$(ps -eo pid,args | grep -E '$PAT' | grep -v grep | awk '{print \$1}')
  [ -n \"\$PIDS\" ] && kill -9 \$PIDS 2>/dev/null
  sleep 1
  echo \"remaining stack procs: \$(ps -eo args | grep -E '$PAT' | grep -v grep | wc -l)\""
docker rm -f m20_sim_run 2>/dev/null
echo "sim container removed; stack down."
