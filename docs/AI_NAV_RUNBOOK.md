# M20-Brain runbook: train, gate, deploy, talk to it

Companion to `docs/AI_NAV_BRIEF.md` (the why). This file covers how to run it.
Every command was run in this repo on 2026-11-04 unless marked *owner box*.

```
operator text ─▶ m20_agent (L3: VLM plans, L2: landmark registry)
                    │ only through /m20/mission/run (commander veto applies)
                    ▼
                 m20_missions ─▶ /navigate_to_pose
                                    │
      ┌─────────────────────────────┼──────────────────────────────┐
      ▼ M20_AI_CONTROLLER=follow_path ▼ =standalone                   ▼ =direct
  Nav2 planner+BT+recoveries     our A* on /map                  no map
  ─▶ /follow_path (m20_ai_nav)   ─▶ m20_ai_nav                   ─▶ m20_ai_nav
      └───────────── learned PPO policy (ONNX, 10 Hz) ─▶ (vx, wz) ─────┘
                    /cmd_vel_nav ─▶ velocity_smoother ─▶ /cmd_vel ─▶ bridge ─▶ robot
```

## 0. One-time setup (sim model + Python deps)
```bash
bash tools/setup_sim.sh                                   # vendor M20 MJCF -> ~/m20_sim
python3 tools/sim/make_gauge_scene.py                     # 6 gauges -> oil_gas_field_gauges.xml
pip install mujoco gymnasium stable-baselines3 torch onnx onnxruntime pyyaml pillow anthropic openai
```
Training and offline gates need only those. The ROS parts need the stack image
(`docker/Dockerfile` now includes onnxruntime + the VLM clients).

## 1. Train the navigator (offline, CPU is enough)
```bash
python3 tools/ai_nav/train.py --out ~/m20_ai_runs/v1 --steps 3000000 --envs 4   # ~95 min on 4 cores
python3 tools/ai_nav/train.py --out ~/m20_ai_runs/v1 --steps 3000000 --resume   # after a crash
tail -f ~/m20_ai_runs/v1/train_log.jsonl                                       # success/collision per 100k
python3 tools/ai_nav/export_onnx.py ~/m20_ai_runs/v1/final.zip src/m20_ai_nav/models/m20_nav_policy.onnx
```
The export verifies ONNX against PyTorch (max |diff| < 1e-4) and fails loudly if
they disagree. The `.onnx.json` next to the model records its source checkpoint, sha256
and `OBS_VERSION`. The runtime refuses a model whose `OBS_VERSION` doesn't match
`spec.py`.

## 2. Gates (in the order they must pass)
| Gate | Command | Bar |
|---|---|---|
| unit | `python3 -m pytest tests/test_ai_nav.py tests/test_agent.py -q` | all pass |
| G1 + G2 | `python3 tools/ai_nav/evaluate.py --model src/m20_ai_nav/models/m20_nav_policy.onnx --out docs/ai_nav/eval_v1.json` | G1 ≥95% success, ≤2% collision, SPL ≥0.80 |
| G2 baseline | `python3 tools/ai_nav/evaluate.py --baseline --out docs/ai_nav/eval_baseline.json` | reference |
| ROS wiring | `bash tools/ai_nav/ros_smoke.sh` (docker) or `RUNNER=local …` | 3/3 modes PASS |
| G4 language | `python3 tools/agent/g4_prompts.py --backend offline\|claude\|cosmos` | 10/10 |
| G5 gauges | `MUJOCO_GL=osmesa python3 tools/agent/sim_mission.py --request "read all the gauges and return to where you started" --out docs/ai_nav/mission_g5` | ≥5/6 within 5% FS, 0 confident-wrong |
| G3 stack | *owner box:* `M20_AI_CONTROLLER=follow_path bash tools/ci/batch_nav_test.sh` | ≥9/10, mean < 0.5 m |

The current numbers are in `docs/ai_nav/RESULTS.md`.

## 3. Run it in the full sim stack (*owner box*, WSL + docker)
```bash
# classical Nav2 (unchanged default)
bash tools/nav/bringup_plan_a.sh
# learned controller under Nav2's planner/BT/recoveries  (recommended first)
M20_AI_CONTROLLER=follow_path M20_STATIC_MAP=maps/oil_gas_field bash tools/nav/bringup_plan_a.sh
# our own A* + learned policy, NO Nav2 at all
M20_AI_CONTROLLER=standalone  M20_STATIC_MAP=maps/oil_gas_field bash tools/nav/bringup_plan_a.sh
# no map at all (reactive only: unmapped ground, map outage)
M20_AI_CONTROLLER=direct bash tools/nav/bringup_plan_a.sh
```
With the agent (start the sim with the camera on, `M20_CAMERA=1`, so gauges can be read):
```bash
M20_AI_CONTROLLER=follow_path M20_AGENT=offline M20_NO_GOAL=1 bash tools/nav/bringup_plan_a.sh
bash tools/agent/say.sh "read all the gauges and return to where you started"
bash tools/agent/say.sh "go to tank 1"
```
Reports (JSON + every gauge frame) land in the container at `/cfg/out/agent/`.
The live controller state is on `/m20/ai_nav/state` (TRACKING / ALIGNING / BLOCKED / STUCK).

**Gauge scene in the stack:** `sim_up.sh` / `bringup_plan_a.sh` load
`oil_gas_field.xml`. For gauge missions, point `M20_MJCF` at
`oil_gas_field_gauges.xml`, which `make_gauge_scene.py` installs next to it.
The plates sit flush on existing equipment, so the saved map stays valid.

## 4. Choosing the brain (VLM backend)
| Backend | Set | Notes |
|---|---|---|
| `claude` (default) | `ANTHROPIC_API_KEY` | `claude-opus-5-5`; override with `M20_AGENT_MODEL`. Server-side refusal fallback is enabled. |
| `cosmos` | `M20_COSMOS_BASE_URL`, `M20_COSMOS_MODEL` (and `M20_COSMOS_API_KEY` for a hosted NIM) | Any OpenAI-compatible endpoint: `vllm serve nvidia/Cosmos-Reason2-8B` on a GPU box, or an NVIDIA NIM. |
| `offline` | nothing | Keyword planner + classical needle reader. Used by CI and as the automatic fallback when the network backend fails. |

`claude` and `cosmos` always run with `offline` behind them (`FallbackBackend`):
if the link drops mid-mission, the robot keeps working and the report records why.

## 5. Safety model (what the AI can and cannot do)
- The VLM produces a **plan**, never motion. `plan.validate()` rejects unknown
  landmarks, reads of non-gauges, waits over 120 s and plans over 40 steps
  **before anything moves**.
- Motion goes only through `/m20/mission/run`, so the commander veto, the mode gate
  and the 1 s bridge watchdog are unchanged.
- The policy's action space is clipped to the velocity-smoother limits.
- NavCore has a hard forward-stop envelope at 0.6 m in a ±35° cone, plus 3 s of
  world-frame obstacle memory, because the LiDAR is blind inside 0.5 m.
- Gauge readings below 0.7 confidence are marked `NEEDS_REVIEW`; an unreadable
  dial is `UNREADABLE`. Neither is reported as a value.

## 6. Troubleshooting (all found the hard way, 2026-11-04)
| Symptom | Cause / fix |
|---|---|
| follow_path: lifecycle waits forever on `local_costmap/...` | Humble's standalone `nav2_costmap_2d` ignores launch name/namespace; its node is always `costmap/costmap`. The launch re-keys params and generates BTs to match. |
| follow_path: every Nav2 node dies, "Will not support aliasing" | rcl's YAML parser rejects anchors. The derived params file is dumped with aliases off. |
| follow_path: bringup aborts on costmap bond | Standalone Costmap2DROS never bonds. It has its own lifecycle manager with `bond_timeout: 0`. |
| follow_path: bt_navigator "follow_path not available" | DDS discovery under load. Derived params set `wait_for_service_timeout: 10000`. |
| direct mode: robot circles near the start | Sparse path (2 points) pinned the carrot. Paths are now densified to 0.1 m. |
| collision while pivoting next to equipment | The body reaches 0.50 m when it pivots; A* lethal radius is now 0.55 m (Nav2's 0.45 is too tight for skid-steer pivots). |
| collision creeping up to a face | The LiDAR is blind inside 0.5 m. NavCore's obstacle memory now keeps near returns. |
| gauge UNREADABLE, dial at frame edge | Heading error at arrival. The executor now turns to the object's true bearing before capture, with a 0.10 rad tolerance. |
