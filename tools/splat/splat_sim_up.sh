#!/usr/bin/env bash
# Splat-world sim with a live PHOTOREAL view in the browser. STAYS IN THE FOREGROUND:
# this terminal keeps WSL (and the stack) alive; Ctrl-C tears everything down.
#
#   wsl -d Ubuntu-24.04 -e bash <repo>/tools/splat/splat_sim_up.sh
#   then open http://localhost:8080   (arm + send goals from the viewer panel)
#
# Bringup stays single-sourced in tools/nav/bringup_plan_a.sh; this only adds the viewer.
# M20_SCENE (default indoor_splat.xml), VIEW_PORT (default 8080),
# M20_EXPLORE=1 (start the exploration tour automatically once the sim is up).
set -e
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
SCENE="${M20_SCENE:-indoor_splat.xml}"
PORT="${VIEW_PORT:-8080}"
SCENE_YAML="$REPO/tools/sim/${SCENE%.xml}.scene.yaml"
SPLAT_FILE=$(awk '/^  file:/{print $2; exit}' "$SCENE_YAML")

cleanup() {
  trap - INT TERM EXIT
  echo; echo "[splat_sim_up] tearing down..."
  bash "$REPO/tools/nav/kill_stack.sh" >/dev/null 2>&1 || true
  echo "[splat_sim_up] down."
  exit 0
}
trap cleanup INT TERM EXIT

bash "$REPO/tools/nav/kill_stack.sh" >/dev/null 2>&1 || true
# demo guarantees, both ON by default (set =0 for real physics / real SLAM):
#   M20_UPRIGHT_ASSIST  virtual anti-tip stabilizer — the robot cannot fall
#   M20_GT_LOC          ground-truth localization — Nav2 cannot mislocalize mid-lap
M20_UPRIGHT_ASSIST="${M20_UPRIGHT_ASSIST:-1}" M20_GT_LOC="${M20_GT_LOC:-1}" \
M20_SCENE="$SCENE" M20_SPLAT_CAMERA=1 M20_NO_GOAL=1 \
  bash "$REPO/tools/nav/bringup_plan_a.sh"

echo "[splat_sim_up] starting photoreal viewer on :$PORT"
# The sim's own MJCF: the viewer renders it with MuJoCo (EGL) and depth-composites it into
# the splat. On WSL2, GPU OpenGL is Mesa's D3D12 driver over the /usr/lib/wsl libs (without
# them EGL falls back to llvmpipe, ~5x slower); native Linux uses NVIDIA EGL (graphics cap).
M20_DESC="$HOME/m20_sim/sdk_deploy/src/M20_sdk_deploy/M20_description"
GL_ARGS=(-e MUJOCO_GL=egl -e NVIDIA_DRIVER_CAPABILITIES=all)
if [ -d /usr/lib/wsl/lib ]; then
  GL_ARGS+=(-v /usr/lib/wsl:/usr/lib/wsl:ro -e LD_LIBRARY_PATH=/usr/lib/wsl/lib
            -e GALLIUM_DRIVER=d3d12 -e MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA)
fi
mkdir -p "$HOME/m20_sim/laps"            # recorded laps for Viser replay (persist across restarts)
docker rm -f m20_splat_view >/dev/null 2>&1 || true
docker run -d --name m20_splat_view --gpus all --network host --ipc host \
  -e ROS_DOMAIN_ID=42 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e VIEW_PORT="$PORT" \
  -e VIEW_AUTO_EXPLORE="${M20_EXPLORE:-0}" "${GL_ARGS[@]}" \
  -e SPLAT_PLY=/splat/$SPLAT_FILE -e SPLAT_SCENE=/scene/$(basename "$SCENE_YAML") \
  -v "$HOME/m20_sim/splats":/splat:ro -v "$REPO/tools/sim":/scene:ro \
  -v "$REPO/tools/splat":/splat_tools:ro -v "$HOME/m20_sim/m20_urdf":/urdf:ro \
  -v "$M20_DESC":/model:ro -v "$HOME/m20_sim/laps":/laps \
  m20_splat:latest python3 /splat_tools/splat_viewer.py >/dev/null
[ -f "$HOME/m20_sim/m20_urdf/M20.urdf" ] || echo "[splat_sim_up] NOTE: ~/m20_sim/m20_urdf/M20.urdf missing — robot drawn as a box"

# Object detector (GPU): OWLv2 office vocabulary on the photoreal camera -> /detections/image
# (annotated) + /detections/objects (3D object map). Viser shows both. M20_DETECT=0 = off.
if [ "${M20_DETECT:-1}" = "1" ]; then
  echo "[splat_sim_up] starting object detector"
  docker rm -f m20_detector >/dev/null 2>&1 || true
  docker run -d --name m20_detector --gpus all --network host --ipc host \
    -e ROS_DOMAIN_ID=42 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -v "$REPO/tools/splat":/splat_tools:ro \
    m20_splat:latest python3 -u /splat_tools/detector_node.py >/dev/null
fi

# MuJoCo mirror relay: streams the sim's robot state on :8770 so a NATIVE MuJoCo viewer on
# the host (tools/splat/mujoco_mirror.py) shows the same run — WSLg can't paint MuJoCo here.
echo "[splat_sim_up] starting MuJoCo mirror relay on :${MIRROR_PORT:-8770}"
docker rm -f m20_mujoco_mirror >/dev/null 2>&1 || true
docker run -d --name m20_mujoco_mirror --network host --ipc host \
  -e ROS_DOMAIN_ID=42 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e M20_SCENE="$SCENE" -e MIRROR_PORT="${MIRROR_PORT:-8770}" \
  -v "$REPO/tools/splat":/splat_tools:ro \
  m20_splat:latest python3 -u /splat_tools/mujoco_mirror_relay.py >/dev/null
[ -f "$M20_DESC/m20_mjcf/mjcf/M20.xml" ] || echo "[splat_sim_up] NOTE: $M20_DESC missing — no MuJoCo composite (URDF mesh only)"
grep -q "^topdown:" "$SCENE_YAML" || echo "[splat_sim_up] NOTE: no top-down map for this scene — run tools/splat/render_topdown.py (RUN_GUIDE 6a)"

echo
echo "=================================================================="
echo " PHOTOREAL VIEW:  http://localhost:$PORT"
echo " MUJOCO VIEW:     on Windows run  python tools\splat\mujoco_mirror.py --follow"
echo " In the panel: 'Explore the world' (full tour), or Arm + 'Go to goal'."
echo " >>> LEAVE THIS TERMINAL OPEN <<<   (Ctrl-C here = shut it all down)"
echo "=================================================================="
# follow the viewer, but live as long as the SIM does (restarting the viewer alone is fine)
while docker ps --format '{{.Names}}' | grep -q '^m20_sim_run$'; do
  docker logs -f --since 1s m20_splat_view 2>&1 || sleep 3
done
echo "[splat_sim_up] sim exited — cleaning up the rest..."
