#!/usr/bin/env bash
# OBSERVER-TRUST TEST (TEST_PLAN session-4 fallout): one nav run; PASS iff three
# independent observers AGREE the robot really traveled:
#   1. the action result (SUCCEEDED)
#   2. the in-run nav_logger trajectory (x-range must span the journey)
#   3. a live end-of-run probe of /odom_true
# This is the exact three-way agreement the FastDDS eras failed (blind loggers,
# phantom SUCCEEDED). Run it twice after any DDS/transport change.
set -u
R="$(cd "$(dirname "$0")/../.." && pwd)"
S="source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
C=docker-commander-1

bash "$R/tools/nav/kill_stack.sh" >/dev/null 2>&1; sleep 3
RUNOUT=$(GOAL_X=-6.0 GOAL_Y=-0.5 timeout 300 bash "$R/tools/nav/bringup_plan_a.sh" 2>&1)
RESULT=$(echo "$RUNOUT" | grep -oE "SUCCEEDED|ABORTED|CANCELED" | tail -1); [ -z "$RESULT" ] && RESULT=TIMEOUT
LIVE=$(docker exec $C bash -lc "$S; timeout 6 ros2 topic echo --once /odom_true 2>/dev/null" | grep -m1 -E "^ +x:" | awk '{print $2}')
sleep 14   # let nav_logger finish its window and write CSVs
TMP=$(mktemp -d); docker cp $C:/cfg/out/traj_true.csv "$TMP/" >/dev/null 2>&1
python3 - "$TMP/traj_true.csv" "$RESULT" "${LIVE:-nan}" <<'PY'
import csv, sys, math
path, result, live = sys.argv[1], sys.argv[2], sys.argv[3]
try:
    rows = list(csv.reader(open(path)))[1:]
    xs = [float(r[1]) for r in rows]
    span = max(xs) - min(xs)
    logger_final = xs[-1]
except Exception:
    span, logger_final = float("nan"), float("nan")
try: live_x = float(live)
except ValueError: live_x = float("nan")
traveled_logger = span > 3.0                      # journey is ~6 m in x
traveled_live = not math.isnan(live_x) and live_x < -4.0
agree = (result == "SUCCEEDED") == traveled_logger == traveled_live
print(f"action={result}  logger: span={span:.2f} final_x={logger_final:.2f}  live_x={live_x:.2f}")
print("OBSERVER-TRUST: " + ("PASS — all three observers agree" if agree and result == "SUCCEEDED"
      else "FAIL — observers disagree or run failed (see TEST_PLAN phantom-success class)"))
sys.exit(0 if agree and result == "SUCCEEDED" else 1)
PY
