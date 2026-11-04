#!/usr/bin/env bash
# Save the map slam_toolbox is building RIGHT NOW (any site, any way you drove it),
# as the artifacts M20_STATIC_MAP=maps/<name> loads.
#
#   M20_NO_GOAL=1 bash tools/nav/bringup_plan_a.sh       # mapping mode, no goal sent
#   ...drive the whole office by teleop (Foxglove), slowly, closing loops...
#   bash tools/map/save_map.sh office                    # -> maps/office.{posegraph,data,pgm,yaml}
#
# Indoors there is no GPS: do NOT set M20_EKF/M20_EKF_TF (that is the oil-field
# anchored-mapping pipeline, tools/map/build_map.sh). Walls give slam_toolbox
# plenty to match; close at least one loop (come back past where you started).
set -euo pipefail
[ $# -eq 1 ] || { echo "usage: $0 <map-name>"; exit 1; }
NAME=$1
C=docker-commander-1
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
SRC="source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
docker exec $C bash -lc "mkdir -p /cfg/maps"
echo "[save_map] serializing slam_toolbox posegraph -> /cfg/maps/$NAME"
docker exec $C bash -lc "$SRC; ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph '{filename: /cfg/maps/$NAME}'" > "$REPO/maps/.save_$NAME.log" 2>&1 \
  || { echo "ERROR: serialize_map failed (is slam_toolbox running in mapping mode?) — see maps/.save_$NAME.log"; exit 1; }
echo "[save_map] saving occupancy image (for eyeballs + the keepout edits below)"
docker exec $C bash -lc "$SRC; timeout 30 ros2 run nav2_map_server map_saver_cli -f /cfg/maps/$NAME" >> "$REPO/maps/.save_$NAME.log" 2>&1 \
  || echo "[save_map] WARN: map_saver_cli failed (pgm is for eyeballs only; posegraph is the artifact)"
for f in "$NAME.posegraph" "$NAME.data" "$NAME.pgm" "$NAME.yaml"; do
  docker cp "$C:/cfg/maps/$f" "$REPO/maps/$f" >> "$REPO/maps/.save_$NAME.log" 2>&1 || true
done
[ -f "$REPO/maps/$NAME.posegraph" ] || { echo "ERROR: no posegraph produced — see maps/.save_$NAME.log"; exit 1; }
echo "[save_map] done: maps/$NAME.* — navigate with M20_STATIC_MAP=maps/$NAME"
echo "[save_map] GLASS: the LiDAR sees through it, so glass walls/doors are NOT in this map."
echo "           Editing the .pgm does NOT fix that (navigation uses the posegraph's /map)."
echo "           Until a Nav2 keepout mask is set up (docs/OFFICE_DEPLOYMENT.md, open item),"
echo "           keep missions away from glass or tape the glass at 0.1-1.0 m height."
