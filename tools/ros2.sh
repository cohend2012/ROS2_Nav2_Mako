#!/usr/bin/env bash
# Run any ros2 CLI command against the RUNNING m20 stack.
# The stack lives inside Docker (ROS is not on your host/WSL shell), so this spins
# a throwaway container on the same DDS domain (ROS_DOMAIN_ID=42) and host network.
#
# Usage (from WSL, in the repo root):
#   bash tools/ros2.sh topic list
#   bash tools/ros2.sh topic echo /m20/mode
#   bash tools/ros2.sh node list
#   bash tools/ros2.sh topic hz /m20/health
#
# From Windows PowerShell:
#   wsl -d Ubuntu -- bash /mnt/c/GIT/StatefullXOne/m20_autonomy_ws/m20_autonomy_ws/tools/ros2.sh topic list
IMAGE="${M20_IMAGE:-m20_autonomy:latest}"
TTY=""; [ -t 0 ] && TTY="-it"
exec docker run --rm $TTY --network host --ipc host -e ROS_DOMAIN_ID=42 "$IMAGE" ros2 "$@"
