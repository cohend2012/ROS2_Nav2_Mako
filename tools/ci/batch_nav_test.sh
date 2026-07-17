#!/usr/bin/env bash
# ============================================================================
# L3.2 reliability gate (TEST_PLAN.md): N fresh full-stack nav runs -> success
# rate + true-error stats. This is the merge gate for phase branches -> master,
# and the future CI job.
#
# Each run uses a FULLY FRESH stack (sim + bridge + slam + nav2) -- reusing
# Nav2 across a sim restart leaves stale costmap/TF state and causes bogus
# aborts (root-caused 2026-07-12; see TEST_PLAN "regression guards").
#
# Usage:  bash tools/ci/batch_nav_test.sh [N_RUNS]        (default 10)
# Output: tools/ci/out/batch_results.csv + printed summary.
# Pass:   success >= 8/10 AND mean true error < 0.8 m.
# ============================================================================
set -u
N=${1:-10}
C=docker-commander-1
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
MODEL="$HOME/m20_sim/sdk_deploy/src/M20_sdk_deploy/M20_description"
OUT="$REPO/tools/ci/out"
GX=-6.0; GY=-0.5
SRC="source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; export ROS_DOMAIN_ID=42 ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_SUPER_CLIENT=TRUE"
mkdir -p "$OUT"
bash "$REPO/tools/dev/container_deps.sh" || exit 1
RES="$OUT/batch_results.csv"
echo "run,result,true_x,true_y,err_m,secs" > "$RES"

stage() {  # copy current configs/tools into the container once
  docker exec $C bash -lc "mkdir -p /cfg"
  docker cp "$REPO/src/m20_locomotion_bridge/m20_locomotion_bridge/bridge_node.py" $C:/cfg/bridge_node.py
  docker cp "$REPO/src/m20_navigation/config/nav2_params.yaml" $C:/cfg/nav2_params.yaml
  docker cp "$REPO/tools/slam/mapper_params.yaml" $C:/cfg/mapper_params.yaml
  docker cp "$REPO/src/m20_navigation/config/pointcloud_to_laserscan.yaml" $C:/cfg/pointcloud_to_laserscan.yaml
}

teardown() {
  docker exec $C bash -lc 'PAT="navigation_launch|nav2_|component_container|slam_toolbox|bridge_node|nav_logger|pointcloud_to_laserscan|static_transform_publisher";
    PIDS=$(ps -eo pid,args | grep -E "$PAT" | grep -v grep | awk "{print \$1}"); kill -9 $PIDS 2>/dev/null; true' >/dev/null 2>&1
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

# contention guard: a loaded box starves Nav2's 20 Hz control loop below the bridge
# watchdog (observed 2026-07-15: concurrent workload -> load 4.7 -> robot never moved,
# 3 bogus TIMEOUTs). Warn loudly; the gate is only meaningful on a quiet box.
LOAD=$(awk '{print $1}' /proc/loadavg)
if awk "BEGIN{exit !($LOAD > 2.0)}"; then
  echo "!! WARNING: load average $LOAD > 2.0 — ANOTHER WORKLOAD IS RUNNING."
  echo "!! Gate results will be CONTENTION-CONTAMINATED. Run on a quiet box."
fi
echo "=== batch_nav_test: $N runs, goal ($GX,$GY), start load=$LOAD ==="
for i in $(seq 1 "$N"); do
  T0=$(date +%s)
  teardown; sleep 2
  # sim (fresh field, robot at origin)
  docker run -d --name m20_sim_run --network host --ipc host \
    -e ROS_DOMAIN_ID=42 -e ROS_DISCOVERY_SERVER=127.0.0.1:11811 -e M20_SIM_GUI=0 \
    -e M20_MJCF=/model/m20_mjcf/mjcf/oil_gas_field.xml \
    -v "$MODEL":/model:ro -v "$REPO/tools/mujoco_sim.py":/mujoco_sim.py:ro \
    m20_sim:latest python3 /mujoco_sim.py >/dev/null 2>&1
  sleep 8
  stage >/dev/null 2>&1
  # lidar static TF, commander-local (cross-container transient_local latching is
  # unreliable on this box — see TEST_PLAN gotchas). 0.10 m = LIDAR_OFFSET in the sim.
  docker exec -d $C bash -lc "$SRC; ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.10 --frame-id base_link --child-frame-id lidar_link"
  sleep 1
  # projection: /LIDAR/POINTS (real interface) -> /scan for slam/Nav2
  docker exec -d $C bash -lc "$SRC; ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r cloud_in:=/LIDAR/POINTS -r scan:=/scan --params-file /cfg/pointcloud_to_laserscan.yaml"
  sleep 2
  # bridge + arm (idempotent: 'illegal transition 1 -> 1' means already armed)
  docker exec -d $C bash -lc "$SRC; python3 /cfg/bridge_node.py --ros-args -p sdk_backend:=sim"
  sleep 4
  docker exec $C bash -lc "$SRC; ros2 service call /m20/set_mode m20_msgs/srv/SetMode '{mode: 1, requester: ci}'" >/dev/null 2>&1
  # slam + nav2
  docker exec -d $C bash -lc "$SRC; ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file /cfg/mapper_params.yaml"
  sleep 6
  docker exec -d $C bash -lc "$SRC; ros2 launch nav2_bringup navigation_launch.py params_file:=/cfg/nav2_params.yaml use_sim_time:=false"
  # wait for bt_navigator active (max 40 s)
  ACTIVE=no
  for _ in $(seq 1 20); do
    ST=$(docker exec $C bash -lc "$SRC; ros2 lifecycle get /bt_navigator 2>/dev/null" | tail -1)
    case "$ST" in active*) ACTIVE=yes; break;; esac
    sleep 2
  done
  if [ "$ACTIVE" != yes ]; then
    echo "run $i: INFRA_FAIL (nav2 not active)"; echo "$i,INFRA_FAIL,nan,nan,nan,$(( $(date +%s) - T0 ))" >> "$RES"; continue
  fi
  # send goal (bounded)
  G=$(docker exec $C bash -lc "$SRC; timeout 150 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    '{pose: {header: {frame_id: map}, pose: {position: {x: $GX, y: $GY}, orientation: {z: 1.0, w: 0.0}}}}' 2>&1" | grep -oE "SUCCEEDED|ABORTED|CANCELED" | tail -1)
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
# a normal run is ~110-190 s wall; anything wildly longer means the HOST SLEPT
# mid-run (observed 2026-07-15: 5032 s runs) and the whole batch is suspect.
slept = [r["run"] for r in rows if r["secs"] not in ("", "nan") and float(r["secs"]) > 400]
print("\n=== L3.2 SUMMARY ===")
print(f"runs={n}  success={len(succ)}/{n}")
if slept:
    print(f"!! HOST-SLEEP SUSPECTED in run(s) {','.join(slept)} (>400 s wall) — "
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
