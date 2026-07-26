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

echo "=== build_map: fresh mapping stack (GPS-ANCHORED: EKF owns odom->base_link) ==="
bash "$REPO/tools/nav/kill_stack.sh" >/dev/null 2>&1 || true
M20_EKF=1 M20_EKF_TF=1 M20_NO_GOAL=1 RUN_SECS=900 bash "$REPO/tools/nav/bringup_plan_a.sh"

# (4,-3) and (0.5,-1) ABORTED in both prior builds (SE approach blocked) -> the
# wellhead region got only ~54 occupied cells. Replaced with a reachable east
# vantage (3.5,-1.5) + an explicit RETURN LEG to the origin so the graph gets a
# loop closure (pulls accumulated drift out of the whole map).
# East field is only reachable AROUND the north end of the ground pipe (the
# (2,4) goal succeeds every build; direct SE approaches abort). So: north lane
# first, then two east-field goals reached via that route, then home for the
# loop closure.
# (6.3,0.9) = wellhead orbit leg: multi-side scan constraints on the map's
# far-east extremity (it4: wellhead was the only failing landmark, 0.68 m,
# scanned mostly from one side).
GOALS="-6.0,-0.5 -7.0,2.0 2.0,4.0 4.5,1.5 5.5,0.3 6.3,0.9 0.0,0.5"
N_GOALS=7
i=0
for g in $GOALS; do
  i=$((i+1)); X="${g%,*}"; Y="${g#*,}"
  echo "=== build_map: coverage goal $i/$N_GOALS ($X, $Y) ==="
  docker exec $C bash -lc "$SRC; timeout 150 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    '{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y}, orientation: {z: 1.0, w: 0.0}}}}'" | tail -2 \
    || echo "    goal $i did not complete (skipping — coverage still counts)"
done

echo "=== build_map: serializing posegraph + saving pgm ==="
docker exec $C bash -lc "mkdir -p /cfg/maps"
docker exec $C bash -lc "$SRC; ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph '{filename: /cfg/maps/oil_gas_field}'" | tail -1
docker exec $C bash -lc "$SRC; timeout 30 ros2 run nav2_map_server map_saver_cli -f /cfg/maps/oil_gas_field" | tail -2 \
  || echo "    map_saver_cli failed (pgm is for eyeballs only; posegraph is the artifact)"

# stage to a CANDIDATE dir first — the committed map is only replaced if the
# landmark gate passes (tools/map/check_map_landmarks.py)
CAND="$REPO/maps/candidate"
mkdir -p "$CAND"
for f in oil_gas_field.posegraph oil_gas_field.data oil_gas_field.pgm oil_gas_field.yaml; do
  docker cp "$C:/cfg/maps/$f" "$CAND/$f" 2>/dev/null || true
done
[ -f "$CAND/oil_gas_field.posegraph" ] || { echo "ERROR: serialize produced no posegraph"; exit 1; }

echo "=== build_map: teardown ==="
bash "$REPO/tools/nav/kill_stack.sh" >/dev/null 2>&1 || true

echo "=== build_map: LANDMARK QUALITY GATE ==="
if python3 "$REPO/tools/map/check_map_landmarks.py" "$CAND/oil_gas_field.pgm" "$CAND/oil_gas_field.yaml"; then
  mv -f "$CAND"/oil_gas_field.* "$REPO/maps/"
  rmdir "$CAND" 2>/dev/null || true
  echo "=== build_map: DONE — gate PASS, maps/ replaced; commit maps/ ==="
else
  echo "=== build_map: gate FAIL — candidate kept in maps/candidate/, committed map UNTOUCHED ==="
  exit 1
fi
