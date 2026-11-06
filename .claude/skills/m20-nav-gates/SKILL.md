---
name: m20-nav-gates
description: Run and interpret the M20 navigation gates — the office batch gate (N door-to-door legs on real ROS 2 + Nav2), the ROS smoke tests for every mode, the classical-variant comparison, and the MuJoCo policy/mission gates. Use before merging any navigation change, when asked "does it work", "is it reliable", "compare controllers", or to decide what to deploy.
---

# M20 navigation gates

Rule zero: **never call a single run a pass.** This project reported a one-off 6/6 once;
across seeds it was 3/6. Report n, the pass count, and the failure modes.

## Gate ladder (cheapest first)
| Gate | Command | Bar | Needs |
|---|---|---|---|
| Unit | `python3 -m pytest tests/test_ai_nav.py tests/test_agent.py -q` | all pass | python only |
| ROS wiring, all modes | `bash tools/ai_nav/ros_smoke.sh` (docker) or `RUNNER=local …` | every mode PASS | ROS 2 Humble + Nav2 |
| **Office batch** | `BATCH=1 SITE=office MODES=nav2 VARIANT=<v> RUN_TAG=<n> bash tools/ai_nav/ros_smoke.sh` | **≥ 9/10 legs** | ROS 2 + Nav2 |
| Office batch, AI | same with `MODES=standalone` (or `follow_path`), optional `MODEL_PATH=` | ≥ 9/10 legs | ROS 2 + Nav2 |
| Policy G1/G2/field | `python3 tools/ai_nav/evaluate.py --model <onnx> --out <json>` | G1 ≥ 95%, ≤ 2% coll | MuJoCo |
| Office O1-O3 | `python3 tools/ai_nav/evaluate.py --model <onnx> --office 40 --out <json>` | ≥ scripted baseline | MuJoCo |
| Gauge mission G5 | `tools/agent/sim_mission.py … --seed N`, **≥ 5 seeds** | ≥ 9/10 missions | MuJoCo + osmesa |
| Find gauges G6 | `MUJOCO_GL=osmesa python3 tools/agent/find_gauges.py --out <dir> --seed N`, ≥ 4 seeds | 0 false positives, 0 confident-wrong; report found/read per seed | MuJoCo + osmesa |
| Dial detector | `MUJOCO_GL=osmesa python3 tools/perception/eval_dial_detector.py` | YOLO ≥ 95 % in its envelope, 0 office false alarms | MuJoCo + osmesa |
| Full stack G3 | `M20_AI_CONTROLLER=… bash tools/ci/batch_nav_test.sh` (owner box) | ≥ 9/10, < 0.5 m | sim images |

Add `CM=1` to any ROS gate to put Nav2's `collision_monitor` (stop/slow polygons,
`src/m20_navigation/config/collision_monitor.yaml`) behind the controller.

## Reading results
- Batch JSON: `docs/ai_nav/office_batch/<variant>_<mode>[_cm]_run<n>.json`, one entry per
  leg with status, time and final error. Smoke JSON: `docs/ai_nav/smoke*.json`.
- Logs: `tools/ai_nav/ros_smoke/out/<site>/<variant>/run<n>/[cm/]<mode>/`. Read
  `ai_nav.log` for Nav2 (count `Failed to make progress`, `collision ahead`,
  `patience exceeded`), `state.log` for the learned controller's state trace, and
  `agent.log` for missions.
- `docs/ai_nav/RESULTS.md` is GENERATED: `python3 tools/ai_nav/make_results.py`. Never
  hand-edit its numbers.

## Known traps (each cost hours once)
- `pkill -f <pattern>` matched and killed the calling shell. Tear down by process
  group (`run_mode.sh` uses `set -m` and `kill -- -PGID`).
- Leftover nodes from a previous run hijack `/navigate_to_pose`. Check with
  `ps -eo args | grep -c "^/root/mamba/root/envs/ros/"` (local) before a run.
- Two Nav2 stacks on one CPU starve MPPI and slow DWB. Run comparisons one at a time.
- Nav2 rejects goals until its lifecycle is active. The harness retries for 60 s; don't
  count that as a navigation failure.
- Agent tests must go somewhere the nav leg didn't end, or they pass trivially.
- Don't edit a bash script while a run is executing it (bash reads incrementally).
- A "tipped over" in MuJoCo on flat ground was the mixer's yaw trim winding up on a
  stalled turn, not terrain. The trim is now bounded (`YAW_TRIM_MAX`, mixer + bridge,
  parity-tested). If tips come back, snapshot with `copy.deepcopy(env)` before the leg
  (pass `{id(core.policy): core.policy}` as memo: ONNX sessions can't be copied) and
  replay it.
