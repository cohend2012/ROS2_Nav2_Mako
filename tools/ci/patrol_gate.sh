#!/usr/bin/env bash
# ============================================================================
# M1a PATROL GATE (ROADMAP.md): one mission of 5 diverse goals spanning the
# site — W field, NW corner, N lane, E field, home. Scores each leg's TRUE
# error at arrival (ground truth read at leg completion, scorecard only).
# Pass: 5/5 legs complete AND mean true error < 0.5 m.
#
# This exists because every prior gate drove ONE goal: (-6,-0.5). A controller
# tuned on one route can overfit; the patrol is the anti-overfit measurement.
# Usage: bash tools/ci/patrol_gate.sh   (~8 min, quiet box)
# ============================================================================
set -u
C=docker-commander-1
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
S="source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
OUT="$REPO/tools/ci/out"
mkdir -p "$OUT"
RES="$OUT/patrol_results.csv"
echo "leg,gx,gy,result,true_x,true_y,err_m,secs" > "$RES"

GOALS="-6.0,-0.5 -7.0,2.0 2.0,4.0 5.5,0.3 0.0,0.5"

LOAD=$(awk '{print $1}' /proc/loadavg)
awk "BEGIN{exit !($LOAD > 2.0)}" && echo "!! load $LOAD > 2 — results will be contaminated"

echo "=== patrol_gate: fresh stack ==="
bash "$REPO/tools/nav/kill_stack.sh" >/dev/null 2>&1 || true
M20_STATIC_MAP=maps/oil_gas_field M20_NO_GOAL=1 RUN_SECS=1200 \
  bash "$REPO/tools/nav/bringup_plan_a.sh" >/dev/null 2>&1 || { echo "bringup FAILED"; exit 1; }

true_pose() {
  docker exec $C bash -lc "$S; timeout 5 ros2 topic echo --once /odom_true 2>/dev/null" \
    | python3 -c "
import sys
x=y=None
for l in sys.stdin:
    s=l.strip()
    if s.startswith('x:') and x is None: x=float(s[2:])
    elif s.startswith('y:') and y is None: y=float(s[2:])
    if x is not None and y is not None: break
print(f'{x} {y}' if x is not None else 'nan nan')"
}

i=0
for g in $GOALS; do
  i=$((i+1)); GX="${g%,*}"; GY="${g#*,}"
  T0=$(date +%s)
  R=$(docker exec $C bash -lc "$S; timeout 200 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    '{pose: {header: {frame_id: map}, pose: {position: {x: $GX, y: $GY}, orientation: {z: 1.0, w: 0.0}}}}'" 2>&1 \
    | grep -oE "SUCCEEDED|ABORTED|CANCELED" | tail -1)
  [ -z "$R" ] && R=TIMEOUT
  read -r TX TY <<< "$(true_pose)"
  ERR=$(python3 -c "import math;print(f'{math.hypot($TX-($GX), $TY-($GY)):.2f}')" 2>/dev/null || echo nan)
  SECS=$(( $(date +%s) - T0 ))
  echo "leg $i ($GX,$GY): $R err=${ERR}m ${SECS}s"
  echo "$i,$GX,$GY,$R,$TX,$TY,$ERR,$SECS" >> "$RES"
done
bash "$REPO/tools/nav/kill_stack.sh" >/dev/null 2>&1 || true

python3 - "$RES" <<'PY'
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1])))
ok = [r for r in rows if r["result"] == "SUCCEEDED"]
errs = [float(r["err_m"]) for r in ok if r["err_m"] != "nan"]
mean = sum(errs)/len(errs) if errs else float("nan")
print(f"\n=== M1a PATROL GATE: {len(ok)}/{len(rows)} legs, mean {mean:.2f} m ===")
print("PASS" if len(ok) == len(rows) and mean < 0.5 else "FAIL (need 5/5 and mean < 0.5)")
PY
