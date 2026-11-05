#!/usr/bin/env bash
# Entry for the `perception` service: /LIDAR/POINTS -> /scan, plus slam_toolbox.
#   M20_MAP=<name>  localization on /maps/<name>.posegraph (mounted ./maps); unset = MAPPING
#                   (drive by teleop, then: bash tools/map/save_map.sh <name>)
set -euo pipefail
# ROS setup files are not nounset-safe (AMENT_TRACE_SETUP_FILES etc.): under set -u
# they abort the script SILENTLY. Relax -u only while sourcing (found in the
# container verification build, 2026-11-05).
set +u
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
set -u
CFG="$(ros2 pkg prefix m20_navigation)/share/m20_navigation/config"
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r cloud_in:=/LIDAR/POINTS -r scan:=/scan --params-file "$CFG/pointcloud_to_laserscan.yaml" &
if [ -n "${M20_MAP:-}" ]; then
  if [ ! -f "/maps/$M20_MAP.posegraph" ]; then
    echo "[perception] /maps/$M20_MAP.posegraph missing"; exit 1
  fi
  echo "[perception] LOCALIZATION on /maps/$M20_MAP"
  exec ros2 run slam_toolbox localization_slam_toolbox_node --ros-args \
    --params-file /ws/config/slam/localization_params.yaml -p map_file_name:="/maps/$M20_MAP"
fi
echo "[perception] MAPPING (no M20_MAP set)"
exec ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file /ws/config/slam/mapper_params.yaml
