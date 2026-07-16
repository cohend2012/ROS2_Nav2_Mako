#!/usr/bin/env bash
# ============================================================================
# Progress video for a checkpoint/merge: run ONE instrumented autonomous goal
# on the current stack, then replay-render the REAL logged trajectory to
#   docs/media/progress/<label>.mp4
# Policy (TEST_PLAN): generate at every merge-to-master / checkpoint tag so
# there is a watchable record of what the stack could do at that commit.
#
# Usage:  bash tools/ci/make_progress_video.sh [label]
#         label defaults to the short git SHA; checkpoints use the tag name.
# Notes:  runs the full stack (~2 min) + host render (~3 min). Same single-
#         session rule as all live tests. Honest artifact: the video replays
#         the actual logged /odom_true trajectory — never a re-simulation.
# ============================================================================
set -e
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
LABEL="${1:-$(cd "$REPO" && git rev-parse --short HEAD)}"
OUT_DIR="$REPO/docs/media/progress"
mkdir -p "$OUT_DIR"

echo "[video] instrumented run for '$LABEL' (bringup + goal + logging)..."
bash "$REPO/tools/nav/bringup_plan_a.sh"
sleep 100   # nav_logger RUN_SECS window inside bringup captures the run

echo "[video] collecting logs..."
TMP="$REPO/tools/nav/out_progress"
rm -rf "$TMP"; mkdir -p "$TMP"
docker cp docker-commander-1:/cfg/out/traj_true.csv "$TMP/" 2>/dev/null || {
  echo "[video] ERROR: no traj_true.csv — did the run/logging fail?"; exit 1; }

echo "[video] rendering replay (host egl, ~3 min)..."
OUTDIR="$TMP" OUTMP4="$OUT_DIR/$LABEL.mp4" python3 "$REPO/tools/nav/replay_render.py"

bash "$REPO/tools/nav/kill_stack.sh" >/dev/null 2>&1 || true
echo "[video] wrote docs/media/progress/$LABEL.mp4"
echo "[video] commit it with the checkpoint: git add docs/media/progress/$LABEL.mp4"
