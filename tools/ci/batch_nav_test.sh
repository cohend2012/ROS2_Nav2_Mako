#!/usr/bin/env bash
# ============================================================================
# L3.2 reliability gate (TEST_PLAN.md): N fresh full-stack nav runs -> success
# rate + true-error stats. Merge gate for phase branches -> master.
#
# Each run = ONE call to tools/nav/bringup_plan_a.sh (the SINGLE bringup code
# path — the batch used to carry its own inline copy, which diverged and
# produced never-moved runs while manual bringups were green; never fork it).
#
# Usage:  bash tools/ci/batch_nav_test.sh [N_RUNS]        (default 10)
# Output: tools/ci/out/batch_results.csv + printed summary.
# Pass:   success >= 8/10 AND mean true error < 0.8 m.
# ============================================================================
set -u
N=${1:-10}
C=docker-commander-1
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="$REPO/tools/ci/out"
GX=-6.0; GY=-0.5
SRC="source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; export ROS_DOMAIN_ID=42 ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_SUPER_CLIENT=TRUE"
mkdir -p "$OUT"
RES="$OUT/batch_results.csv"
echo "run,result,true_x,true_y,err_m,secs" > "$RES"

teardown() {
  # SIGINT first so DDS participants UNREGISTER from the discovery server
  # (kill -9 leaves stale registrations that poison later matching); then
  # force-kill leftovers. Do NOT bounce the discovery server (orphans the
  # long-lived commander client).
  docker exec $C bash -lc 'PAT="navigation_launch|nav2_|component_container|slam_toolbox|bridge_node|nav_logger|pointcloud_to_laserscan|static_transform_publisher";
    PIDS=$(ps -eo pid,args | grep -E "$PAT" | grep -v grep | awk "{print \$1}");
    [ -n "$PIDS" ] && kill -2 $PIDS 2>/dev/null; sleep 2;
    PIDS=$(ps -eo pid,args | grep -E "$PAT" | grep -v grep | awk "{print \$1}");
    [ -n "$PIDS" ] && kill -9 $PIDS 2>/dev/null; true' >/dev/null 2>&1
  docker rm -f m20_sim_run >/dev/null 2>&1
}

true_pose() {  # echo "x y" of ground truth
  docker exec $C bash -lc "$SRC; timeout 5 ros2 topic echo --once /odom_true 2>/dev/null" \
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

# contention guard: a loaded box starves the 20 Hz control loop (observed:
# load 4.7 -> robot never moved). The gate is only meaningful on a quiet box.
LOAD=$(awk '{print $1}' /proc/loadavg)
if awk "BEGIN{exit !($LOAD > 2.0)}"; then
  echo "!! WARNING: load average $LOAD > 2.0 — ANOTHER WORKLOAD IS RUNNING."
  echo "!! Gate results will be CONTENTION-CONTAMINATED. Run on a quiet box."
fi
echo "=== batch_nav_test: $N runs, goal ($GX,$GY), start load=$LOAD ==="
for i in $(seq 1 "$N"); do
  T0=$(date +%s)
  teardown; sleep 2
  RUNOUT=$(GOAL_X=$GX GOAL_Y=$GY timeout 300 bash "$REPO/tools/nav/bringup_plan_a.sh" 2>&1)
  G=$(echo "$RUNOUT" | grep -oE "SUCCEEDED|ABORTED|CANCELED" | tail -1)
  [ -z "$G" ] && G=TIMEOUT
  read -r TX TY <<< "$(true_pose)"
  ERR=$(python3 -c "import math;print(f'{math.hypot($TX-($GX), $TY-($GY)):.2f}')" 2>/dev/null || echo nan)
  SECS=$(( $(date +%s) - T0 ))
  echo "run $i: $G  true=($TX,$TY)  err=${ERR}m  ${SECS}s"
  echo "$i,$G,$TX,$TY,$ERR,$SECS" >> "$RES"
done
teardown

python3 - "$RES" <<'PY'
import csv, sys, math
rows = list(csv.DictReader(open(sys.argv[1])))
n = len(rows)
succ = [r for r in rows if r["result"] == "SUCCEEDED"]
errs = sorted(float(r["err_m"]) for r in succ if r["err_m"] != "nan")
slept = [r["run"] for r in rows if r["secs"] not in ("", "nan") and float(r["secs"]) > 500]
print("\n=== L3.2 SUMMARY ===")
print(f"runs={n}  success={len(succ)}/{n}")
if slept:
    print(f"!! HOST-SLEEP SUSPECTED in run(s) {','.join(slept)} (>500 s wall) — "
          f"BATCH UNRELIABLE, disable PC sleep and RE-RUN before trusting the gate")
if errs:
    mean = sum(errs)/len(errs)
    p95 = errs[min(len(errs)-1, int(round(0.95*len(errs)))-1)]
    print(f"true error: mean={mean:.2f} m  p95={p95:.2f} m  max={errs[-1]:.2f} m")
    gate = len(succ) >= 0.8*n and mean < 0.8
    print(f"GATE: {'PASS' if gate else 'FAIL'} (need >=80% success and mean <0.8 m)")
else:
    print("GATE: FAIL (no successful runs)")
PY
