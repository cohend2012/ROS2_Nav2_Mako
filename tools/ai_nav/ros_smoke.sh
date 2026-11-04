#!/usr/bin/env bash
# M20-Brain ROS 2 integration smoke test: every navigator mode + the agent, on real
# ROS 2 Humble + Nav2, against the kinematic fake world. Wiring, not driving quality.
#
#   docker build -f docker/Dockerfile.ai-verify -t m20_ai_verify:latest .
#   bash tools/ai_nav/ros_smoke.sh                       # all modes, in docker
#   RUNNER=local bash tools/ai_nav/ros_smoke.sh          # current shell's sourced ROS env
#   MODES="standalone" bash tools/ai_nav/ros_smoke.sh
#
# Per mode: fake_world + ai_nav.launch.py mode:=<m> + one NavigateToPose goal to
# (-6,-0.5) whose straight line runs THROUGH skid1 (must route around), then the
# mission server + offline agent run "read the tank 1 gauge and return to where you
# started" (reading must be 64 +- 5, home error < 0.4 m).
# Logs: tools/ai_nav/ros_smoke/out/<mode>/ (persistent, Rule 4).
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
IMG=${IMG:-m20_ai_verify:latest}
MODES=${MODES:-"direct standalone follow_path"}
RUNNER=${RUNNER:-docker}
OUTROOT="$REPO/tools/ai_nav/ros_smoke/out"
fail=0
for MODE in $MODES; do
  echo "=== mode=$MODE ($RUNNER) ==="
  rm -rf "$OUTROOT/$MODE"; mkdir -p "$OUTROOT/$MODE"
  set +e
  if [ "$RUNNER" = "local" ]; then
    MODE=$MODE SMOKE="$REPO/tools/ai_nav/ros_smoke" NAV2_PARAMS="$REPO/src/m20_navigation/config/nav2_params.yaml" \
      LANDMARKS="$REPO/src/m20_agent/config/oil_gas_field_landmarks.yaml" OUT="$OUTROOT/$MODE" \
      AGENT_SRC="$REPO/src/m20_agent" ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77} \
      bash "$REPO/tools/ai_nav/ros_smoke/run_mode.sh"
  else
    docker run --rm --network none \
      -v "$REPO/tools/ai_nav/ros_smoke:/smoke:ro" \
      -v "$REPO/src/m20_navigation/config/nav2_params.yaml:/cfg/nav2_params.yaml:ro" \
      -v "$REPO/src/m20_agent/config:/cfg/agent:ro" -v "$OUTROOT/$MODE:/out" \
      -e MODE="$MODE" -e SMOKE=/smoke -e NAV2_PARAMS=/cfg/nav2_params.yaml \
      -e LANDMARKS=/cfg/agent/oil_gas_field_landmarks.yaml -e OUT=/out -e AGENT_SRC=/ws/src/m20_agent \
      -e ROS_DOMAIN_ID=77 "$IMG" bash -c 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && bash /smoke/run_mode.sh'
  fi
  rc=$?
  set -e
  if [ $rc -eq 0 ]; then echo "mode=$MODE PASS"; else echo "mode=$MODE FAIL (rc=$rc) — logs: $OUTROOT/$MODE/"; fail=1; fi
done
# summary for docs/ai_nav/RESULTS.md (tools/ai_nav/make_results.py)
python3 - "$OUTROOT" "$REPO/docs/ai_nav/smoke.json" "$RUNNER" $MODES <<'PY'
import json, os, sys, time, platform
root, out, runner, modes = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4:]
def rd(m, f):
    p = os.path.join(root, m, f)
    try:
        return json.loads(open(p).read().strip().splitlines()[-1])
    except Exception as e:                      # missing/garbled = FAIL, never skipped
        return {"pass": False, "why": f"{f}: {e}"}
res = {"date": time.strftime("%Y-%m-%d %H:%M"), "env": f"runner={runner}, {platform.platform()}",
       "modes": {m: {"nav": rd(m, "nav.json"), "agent": rd(m, "agent.json")} for m in modes}}
os.makedirs(os.path.dirname(out), exist_ok=True)
json.dump(res, open(out, "w"), indent=2)
print("wrote", out)
PY
exit $fail
