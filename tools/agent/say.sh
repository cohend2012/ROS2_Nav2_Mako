#!/usr/bin/env bash
# Send a language mission to the running M20-Brain agent and print its report.
#   bash tools/agent/say.sh "read all the gauges and return to where you started"
set -euo pipefail
[ $# -ge 1 ] || { echo "usage: $0 '<request>'"; exit 1; }
C=docker-commander-1
SRC="source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
REQ=${1//\'/}
docker exec $C bash -lc "$SRC; ros2 topic pub --once /m20/agent/command std_msgs/msg/String \"{data: '$REQ'}\" >/dev/null"
echo "[say] sent: $REQ — waiting for /m20/agent/report (Ctrl-C to stop waiting)"
docker exec $C bash -lc "$SRC; ros2 topic echo --once --full-length /m20/agent/report std_msgs/msg/String" \
  | sed -n 's/^data: //p' | python3 -c "import sys,json; s=sys.stdin.read().strip().strip(\"'\"); print(json.dumps(json.loads(s), indent=2))"
