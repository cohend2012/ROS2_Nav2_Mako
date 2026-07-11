#!/usr/bin/env bash
# Dev helper: open the MuJoCo viewer on the vendor M20 model so you can SEE the robot.
# Standalone preview (physics runs, no controller yet -> the robot settles under
# gravity). Superseded by the ROS-integrated sim in Phase 1, step 2.
#
# Requires the one-time setup (mujoco + model in ~/m20_sim). Usage:
#   from WSL:        bash tools/view_sim.sh
#   from PowerShell: wsl -d Ubuntu -- bash /mnt/c/git/StatefullXOne/m20_autonomy_ws/m20_autonomy_ws/tools/view_sim.sh
# Close the window (or Ctrl+C) to exit.
set -e
MODEL="${M20_MJCF:-$HOME/m20_sim/sdk_deploy/src/M20_sdk_deploy/M20_description/m20_mjcf/mjcf/M20.xml}"
if [ ! -f "$MODEL" ]; then
  echo "M20 model not found at: $MODEL"
  echo "Run the sim setup first (mujoco install + model fetch)."
  exit 1
fi
export DISPLAY="${DISPLAY:-:0}"
echo "Opening MuJoCo viewer on the M20 (drag to orbit, scroll to zoom; close window to exit)..."
exec python3 -m mujoco.viewer --mjcf="$MODEL"
