#!/usr/bin/env bash
# ============================================================================
# Build the SITE MAP for phase-map-loc: one mapping-mode stack, a coverage loop
# of nav goals around the oil & gas field, then serialize the slam_toolbox
# posegraph (localization-mode input) + save the .pgm/.yaml (human-checkable).
#
# Output (committed as the map artifact):
#   maps/oil_gas_field.posegraph + .data     <- what localization mode loads
#   maps/oil_gas_field.pgm + .yaml           <- for eyeballs + docs
#
# Coverage loop (picked off tools/sim/oil_gas_field.xml geometry, >=1 m clear
# of every obstacle; the tight eastern corridor (7.5,4) is deliberately NOT
# entered — mapping sees it from outside, we don't need to drive it):
#   (-6,-0.5) SW past skid1 -> (-7,2) west of tank1 -> (2,4) north lane between
#   tank2 and the rack -> (4,-3) south of the wellhead -> (0.5,-1) home-ish.
# A goal that aborts is logged and skipped — coverage still accumulates.
#
# Usage: bash tools/map/build_map.sh    (~10 min: 5 goals + serialize)
# ============================================================================
set -e
C=docker-commander-1
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
SRC="source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"

echo "=== build_map: fresh mapping stack ==="
bash "$REPO/tools/nav/kill_stack.sh" >/dev/null 2>&1 || true
M20_NO_GOAL=1 RUN_SECS=900 bash "$REPO/tools/nav/bringup_plan_a.sh"

GOALS="-6.0,-0.5 -7.0,2.0 2.0,4.0 4.0,-3.0 0.5,-1.0"
i=0
for g in $GOALS; do
  i=$((i+1)); X="${g%,*}"; Y="${g#*,}"
  echo "=== build_map: coverage goal $i/5 ($X, $Y) ==="
  docker exec $C bash -lc "$SRC; timeout 150 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    '{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y}, orientation: {z: 1.0, w: 0.0}}}}'" | tail -2 \
    || echo "    goal $i did not complete (skipping — coverage still counts)"
done

echo "=== build_map: serializing posegraph + saving pgm ==="
docker exec $C bash -lc "mkdir -p /cfg/maps"
docker exec $C bash -lc "$SRC; ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph '{filename: /cfg/maps/oil_gas_field}'" | tail -1
docker exec $C bash -lc "$SRC; timeout 30 ros2 run nav2_map_server map_saver_cli -f /cfg/maps/oil_gas_field" | tail -2 \
  || echo "    map_saver_cli failed (pgm is for eyeballs only; posegraph is the artifact)"

mkdir -p "$REPO/maps"
for f in oil_gas_field.posegraph oil_gas_field.data oil_gas_field.pgm oil_gas_field.yaml; do
  docker cp "$C:/cfg/maps/$f" "$REPO/maps/$f" 2>/dev/null || true
done
ls -la "$REPO/maps/"
[ -f "$REPO/maps/oil_gas_field.posegraph" ] || { echo "ERROR: serialize produced no posegraph"; exit 1; }

echo "=== build_map: teardown ==="
bash "$REPO/tools/nav/kill_stack.sh" >/dev/null 2>&1 || true
echo "=== build_map: DONE — inspect maps/oil_gas_field.pgm, then commit maps/ ==="
