#!/usr/bin/env bash
# Run the M20 MuJoCo sim on the ROS 2 bus (drdds interface), GUI routed to WSLg.
# Publishes /JOINTS_DATA + /IMU_DATA, subscribes /JOINTS_CMD, on ROS_DOMAIN_ID=42
# (same domain as the stack). Set M20_SIM_GUI=0 for headless.
#
# Usage (WSL):        bash tools/run_sim.sh
#          PowerShell: wsl -d Ubuntu -- bash /mnt/c/git/StatefullXOne/m20_autonomy_ws/m20_autonomy_ws/tools/run_sim.sh
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODEL_DIR="${M20_MODEL_DIR:-$HOME/m20_sim/sdk_deploy/src/M20_sdk_deploy/M20_description}"
GUI="${M20_SIM_GUI:-1}"
# M20_ARENA=1 loads the obstacle/pipe test arena instead of the bare model.
MJCF_REL="m20_mjcf/mjcf/M20.xml"
[ "${M20_ARENA:-0}" = "1" ] && MJCF_REL="m20_mjcf/mjcf/scene_test.xml"
[ -n "${M20_SCENE:-}" ] && MJCF_REL="m20_mjcf/mjcf/${M20_SCENE}"   # e.g. mapping_arena.xml
if [ ! -f "$MODEL_DIR/$MJCF_REL" ]; then
  echo "Model $MJCF_REL not found under $MODEL_DIR — run tools/setup_sim.sh first."; exit 1
fi
exec docker run --rm --network host --ipc host \
  -e ROS_DOMAIN_ID=42 -e M20_SIM_GUI="$GUI" \
  -e M20_MJCF="/model/$MJCF_REL" \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /mnt/wslg:/mnt/wslg \
  -v /usr/lib/wsl:/usr/lib/wsl \
  --device=/dev/dxg \
  -v "$MODEL_DIR":/model:ro \
  -v "$SCRIPT_DIR/mujoco_sim.py":/mujoco_sim.py:ro \
  m20_sim:latest python3 /mujoco_sim.py
