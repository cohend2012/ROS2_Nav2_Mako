---
name: m20-train-policy
description: Train, export, evaluate and ship the M20 learned navigation policy (PPO in MuJoCo, ONNX on the robot) — including shielded office fine-tuning, the evaluation gates, forensics on failed episodes, and the rules for promoting a model. Use when asked to retrain, improve the AI driver, add a training scenario, or decide which policy to ship.
---

# Learned navigator: train → gate → ship

Contract: `src/m20_ai_nav/m20_ai_nav/spec.py` defines a 151-dim observation and the action
(vx, wz) within the velocity-smoother limits. Changing it invalidates every model: bump
`OBS_VERSION` and retrain. The runtime refuses mismatched models.

## Setup (offline box, CPU is enough)
`bash tools/setup_sim.sh` (vendor M20 MJCF), `python3 tools/sim/make_office_scene.py`, and
`pip install mujoco gymnasium stable-baselines3 torch onnx onnxruntime pyyaml pillow`.

## Train (chunked and resumable, Engineering Rule 4)
```bash
# outdoor from scratch (v1 recipe): 3M steps, ~95 min on 4 cores
python3 tools/ai_nav/train.py --out ~/m20_ai_runs/vN --steps 3000000 --envs 4
# office + outdoor fine-tune INSIDE the safety shield (v3 recipe)
python3 tools/ai_nav/train.py --out ~/m20_ai_runs/vN --steps 2500000 --envs 4 \
  --init ~/m20_ai_runs/v1/final.zip --p-office 0.6 --density0 1.0 --shield
# after a crash or reboot: same command + --resume
tail -f ~/m20_ai_runs/vN/train_log.jsonl      # office_success / office_collision per 100k
python3 tools/ai_nav/export_onnx.py ~/m20_ai_runs/vN/final.zip /path/vN.onnx   # parity-checked
```
Always train with `--shield` for anything that will run behind NavCore: the policy has
to learn inside the same safety layer it is deployed with.

## Gate (every one, before shipping; see m20-nav-gates)
G1/G2/field and office O1-O3 via `tools/ai_nav/evaluate.py`; the G5 mission over ≥ 5
seeds; the office batch with `MODES=standalone MODEL_PATH=…`. Compare against **v1, v3
and the scripted baseline** under the SAME code. Re-measure the old models too whenever
NavCore or the shield changes.

## When training plateaus: diagnose, don't retrain blindly
Twice a "flat curve" was a bug, not learning:
1. A sampler put starts and goals outside the building.
2. The shield replaced a blocked pivot with STOP.
Do this first:
- Run 10 failed episodes through `drive_navcore` (tools/ai_nav/evaluate.py), counting
  states and BLOCKED reasons.
- Render top-down traces (planned path vs actual track) with `mujoco.Renderer(480x480)`.
- Ask whether ANY controller, including the scripted baseline, can solve these episodes.
  If all controllers hit the same ceiling, it's the setup.

## Ship
Put the model in `src/m20_ai_nav/models/` with its `.onnx.json`. The default
`m20_nav_policy.onnx` changes only when the new model beats the current default on the
mission that matters for that site (multi-seed). Otherwise add it alongside
(`m20_nav_policy_vN.onnx`) and document it in `docs/AI_NAV_RUNBOOK.md` §3b. Regenerate
`docs/ai_nav/RESULTS.md` with `tools/ai_nav/make_results.py`.
