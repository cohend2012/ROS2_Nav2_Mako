---
name: m20-missions
description: Operate and extend the M20 language agent — send missions ("go to the kitchen and come back", "start the mission", "read all the gauges"), teach places, define routines, edit the landmark file, switch the VLM backend (Claude / Cosmos / offline), and check the G4 prompt suite. Use when asked to give the robot a task, add a place or routine, or change what the robot understands.
---

# Language missions: AI picks the waypoints, Nav2 drives

The agent (`src/m20_agent`) turns a request into a plan of named places, validates it
against the landmark file, and runs it ONE leg at a time through `/m20/mission/run`
(commander veto applies). It never publishes velocity.

## Send missions
```bash
bash tools/agent/say.sh "go to the kitchen and come back"
bash tools/agent/say.sh "start the mission"        # the default routine
bash tools/agent/say.sh "read all the gauges and return to where you started"
bash tools/agent/say.sh "stop"                     # cancels the leg in flight, skips the rest
```
The report (JSON + every gauge frame) goes to `/cfg/out/agent/` (sim) or `data/agent/`
(compose).

## Teach places (no file editing)
Tight places are fine (the office set has places with only 0.50-0.54 m of clearance,
less than the 0.53 m corner sweep). The navigation plans the arrival heading, so the
robot drives in already aligned instead of pivoting at the end. Avoid teaching a place
INSIDE a doorway: the robot can't stop and turn there.

Drive the robot to the spot, then: `"this is the kitchen"` / `"remember this spot as the
meeting room"` / `"call this the dock"`. Places go to `taught_places.yaml`, survive
restarts, and can never overwrite a surveyed landmark.

## Landmark file and routines
Shape: `src/m20_agent/config/office_landmarks.yaml`.
- `landmarks:` name, type (`room|area|dock|gauge|tank|…`), aliases, `position`, and
  `approach: [x, y, yaw]` (where the robot stands; the camera faces `yaw`).
- Gauges also need `gauge: {units, min, max}`. Never put the true value in this file.
- `routines:` named step lists (`goto`, `read_gauge`, `inspect`, `wait`, `return_home`),
  plus `default_routine:`, which is what "start the mission" runs. Routines cannot call
  routines.
After editing: `python3 -m pytest tests/test_agent.py -q`, then
`python3 tools/agent/g4_prompts.py --backend offline` (10/10 is the bar). Add a case to
`CASES` there for any new phrasing you rely on.

## Backends
| `M20_AGENT` | Needs | Notes |
|---|---|---|
| `offline` | nothing | keyword planner + classical needle reader; also the automatic fallback |
| `claude` | `ANTHROPIC_API_KEY` | `claude-opus-5-5` (override with `M20_AGENT_MODEL`); strict tools, refusal fallback |
| `cosmos` | `M20_COSMOS_BASE_URL`, `M20_COSMOS_MODEL` | any OpenAI-compatible server (vLLM / NIM), e.g. Cosmos Reason 2 |
Run `python3 tools/agent/g4_prompts.py --backend claude` once with a key before relying
on it. It has only been gated offline.

## Safety rules that must stay true (tests enforce them)
Plans are validated before motion. Unknown places are refused with how to teach them.
STOP bypasses the model. Readings below 0.7 confidence are NEEDS_REVIEW, never values.
Home is recorded at mission start.
