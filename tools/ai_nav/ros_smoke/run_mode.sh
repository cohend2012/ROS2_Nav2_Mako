#!/usr/bin/env bash
# Inner smoke run for ONE mode, inside a sourced ROS 2 Humble env (docker or local).
# Env: MODE, SMOKE (this dir), NAV2_PARAMS, LANDMARKS, OUT, AGENT_SRC
set -uo pipefail
set -m   # job control: every background job gets its OWN process group, so the
         # teardown can kill ros2 run/launch AND their child nodes (killing only the
         # wrapper PIDs left orphan servers on /navigate_to_pose that hijacked the
         # next mode's goals — found 2026-11-04)
: "${MODE:?}" "${SMOKE:?}" "${NAV2_PARAMS:?}" "${LANDMARKS:?}" "${OUT:?}"
WORLD=${WORLD:-oil}; SITE=${SITE:-outdoor}
NAV_GOAL=${NAV_GOAL:-"-6.0 -0.5"}; START=${START:-"0.0 0.0"}
AGENT_REQ=${AGENT_REQ:-"read the tank 1 gauge and return to where you started"}; AGENT_EXPECT=${AGENT_EXPECT:-64.0}
read -r SX SY <<<"$START"
mkdir -p "$OUT"
M20_AI_NAV_SRC="${AI_NAV_SRC:-/ws/src/m20_ai_nav}" M20_AGENT_SRC="${AGENT_SRC:-}" python3 "$SMOKE/fake_world.py" --ros-args -p gauge_value:=64.0 -p world:="$WORLD" -p start_x:="$SX" -p start_y:="$SY" >"$OUT/world.log" 2>&1 &
ros2 launch m20_ai_nav ai_nav.launch.py mode:="$MODE" site:="$SITE" params_file:="$NAV2_PARAMS" >"$OUT/ai_nav.log" 2>&1 &
ros2 run m20_missions mission_server >"$OUT/mission.log" 2>&1 &
ros2 run m20_agent agent --ros-args -p backend:=offline -p landmarks:="$LANDMARKS" -p report_dir:="$OUT/agent" >"$OUT/agent.log" 2>&1 &
sleep 15
# controller state trace for forensics (TRACKING / ALIGNING / BLOCKED / STUCK + detail)
ros2 topic echo /m20/ai_nav/state std_msgs/msg/String >"$OUT/state.log" 2>&1 &
python3 "$SMOKE/scenario.py" nav $NAV_GOAL 150 | tee "$OUT/nav.json"; N=${PIPESTATUS[0]}
python3 "$SMOKE/scenario.py" agent "$AGENT_REQ" 300 "$AGENT_EXPECT" | tee "$OUT/agent.json"; A=${PIPESTATUS[0]}
# stop ONLY our own jobs (SIGINT lets ros2 launch shut its children down cleanly);
# never pkill -f by pattern — it matched and killed the calling shell once.
PG=$(jobs -p)
for p in $PG; do kill -INT -- "-$p" 2>/dev/null; done; sleep 4
for p in $PG; do kill -KILL -- "-$p" 2>/dev/null; done; wait 2>/dev/null
exit $(( N | A ))
