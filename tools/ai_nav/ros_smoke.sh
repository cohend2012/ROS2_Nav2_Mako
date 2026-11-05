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
SITE=${SITE:-outdoor}
OUTROOT="$REPO/tools/ai_nav/ros_smoke/out"
VARIANT=${VARIANT:-}      # classical office variant (src/m20_navigation/config/office_variants/)
RUN_TAG=${RUN_TAG:-}      # repeat index, so repeated runs keep separate logs/results
[ "$SITE" = "outdoor" ] || OUTROOT="$OUTROOT/$SITE"
[ -z "$VARIANT" ] || OUTROOT="$OUTROOT/$VARIANT"
[ -z "$RUN_TAG" ] || OUTROOT="$OUTROOT/run$RUN_TAG"
mkdir -p "$OUTROOT"
PARAMS_SRC="$REPO/src/m20_navigation/config/nav2_params.yaml"
LM_FILE=oil_gas_field_landmarks.yaml
export WORLD=oil
if [ "$SITE" = "office" ]; then
  # office: real footprint overlay, fixed office world, reception -> meeting room
  # (0.85 m door) for the nav goal, "go to the meeting room and come back" for the agent
  PARAMS_SRC="$OUTROOT/nav2_params_office.yaml"
  VFILE=""
  if [ -n "$VARIANT" ] && [ "$VARIANT" != "dwb" ]; then
    VFILE="$REPO/src/m20_navigation/config/office_variants/$VARIANT.yaml"
    [ -f "$VFILE" ] || { echo "ERROR: no office variant $VARIANT ($VFILE)"; exit 1; }
  fi
  python3 "$REPO/tools/nav/apply_overlay.py" "$REPO/src/m20_navigation/config/nav2_params.yaml" \
    "$REPO/src/m20_navigation/config/nav2_office_overlay.yaml" $VFILE "$PARAMS_SRC"
  LM_FILE=office_landmarks.yaml
  export WORLD=office
  read -r RX RY MX MY < <(python3 -c "
import yaml; d={l['name']: l for l in yaml.safe_load(open('$REPO/src/m20_agent/config/office_landmarks.yaml'))['landmarks']}
print(*d['reception']['approach'][:2], *d['meeting_room']['approach'][:2])")
  export START="$RX $RY" NAV_GOAL="$MX $MY"
  # the nav leg ENDS in the meeting room, so the agent goes somewhere else (a no-op
  # "go to the meeting room" made home_err trivially 0.0 — caught 2026-11-05)
  export AGENT_REQ="go to the kitchen and come back" AGENT_EXPECT=-1
fi
export SITE
fail=0
for MODE in $MODES; do
  echo "=== mode=$MODE ($RUNNER) ==="
  rm -rf "$OUTROOT/$MODE"; mkdir -p "$OUTROOT/$MODE"
  set +e
  if [ "$RUNNER" = "local" ]; then
    MODE=$MODE SMOKE="$REPO/tools/ai_nav/ros_smoke" NAV2_PARAMS="$PARAMS_SRC" \
      LANDMARKS="$REPO/src/m20_agent/config/$LM_FILE" OUT="$OUTROOT/$MODE" \
      AGENT_SRC="$REPO/src/m20_agent" AI_NAV_SRC="$REPO/src/m20_ai_nav" ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77} \
      bash "$REPO/tools/ai_nav/ros_smoke/run_mode.sh"
  else
    docker run --rm --network none \
      -v "$REPO/tools/ai_nav/ros_smoke:/smoke:ro" \
      -v "$PARAMS_SRC:/cfg/nav2_params.yaml:ro" \
      -v "$REPO/src/m20_agent/config:/cfg/agent:ro" -v "$OUTROOT/$MODE:/out" \
      -e MODE="$MODE" -e SMOKE=/smoke -e NAV2_PARAMS=/cfg/nav2_params.yaml \
      -e LANDMARKS=/cfg/agent/$LM_FILE -e WORLD -e SITE -e START -e NAV_GOAL -e AGENT_REQ -e AGENT_EXPECT -e OUT=/out -e AGENT_SRC=/ws/src/m20_agent \
      -e ROS_DOMAIN_ID=77 "$IMG" bash -c 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && bash /smoke/run_mode.sh'
  fi
  rc=$?
  set -e
  if [ $rc -eq 0 ]; then echo "mode=$MODE PASS"; else echo "mode=$MODE FAIL (rc=$rc) — logs: $OUTROOT/$MODE/"; fail=1; fi
done
# summary for docs/ai_nav/RESULTS.md (tools/ai_nav/make_results.py)
SUMMARY="$REPO/docs/ai_nav/smoke.json"; [ "$SITE" = "outdoor" ] || SUMMARY="$REPO/docs/ai_nav/smoke_$SITE.json"
if [ -n "$VARIANT" ]; then mkdir -p "$REPO/docs/ai_nav/office_variants"; SUMMARY="$REPO/docs/ai_nav/office_variants/${VARIANT}_run${RUN_TAG:-0}.json"; fi
python3 - "$OUTROOT" "$SUMMARY" "$RUNNER" $MODES <<'PY'
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
