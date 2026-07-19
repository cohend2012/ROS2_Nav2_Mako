#!/usr/bin/env bash
# Tear down the full sim stack: Nav2 + slam_toolbox + bridge + logger + estimator
# (in the commander container) and the MuJoCo sim container. Leaves the commander
# container up. WAITS until everything is actually dead — a fire-and-forget kill
# let dying Nav2 nodes collide with the next bringup (duplicate action servers →
# zombie bt_navigator answered a goal with SUCCEEDED while the robot sat parked).
# SIGINT first so DDS participants unregister from the discovery server cleanly.
C=docker-commander-1
PAT="navigation_launch|nav2_smoother|nav2_planner|nav2_behaviors|nav2_bt_navigator|nav2_waypoint|nav2_velocity|nav2_lifecycle|nav2_controller|component_container|slam_toolbox|bridge_node|nav_logger|pointcloud_to_laserscan|static_transform_publisher|estimator.py"
docker exec $C bash -c "
  PIDS=\$(ps -eo pid,args | grep -E '$PAT' | grep -v grep | awk '{print \$1}')
  [ -n \"\$PIDS\" ] && kill -2 \$PIDS 2>/dev/null
  sleep 2
  for i in 1 2 3 4 5 6 7 8; do
    PIDS=\$(ps -eo pid,args | grep -E '$PAT' | grep -v grep | awk '{print \$1}')
    [ -z \"\$PIDS\" ] && break
    kill -9 \$PIDS 2>/dev/null
    sleep 2
  done
  echo \"remaining stack procs: \$(ps -eo args | grep -E '$PAT' | grep -v grep | wc -l)\""
docker rm -f m20_sim_run 2>/dev/null
echo "sim container removed; stack down."
