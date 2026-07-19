# World Models for the M20 — Adoption Guide (2026-07-16)

Deep-research pass: 27 sources, 135 claims extracted, 25 adversarially verified —
**15 confirmed (3-0 votes), 1 refuted, 9 unverified** (verification round was cut by a
rate limit; unverified claims are marked ⚠ and treated as *plausible, not established*).
Question: where do learned world models add real value to a 2-person wheel-legged
inspection-robot project running MuJoCo + ROS 2 + Nav2, and what's hype?

## 1. What "world models" actually are (verified taxonomy)

Two fundamentally different families (ACM Computing Surveys 2025 taxonomy, confirmed):

| Family | Named systems | What they do | Robotics readiness |
|---|---|---|---|
| **Implicit / latent** — predict in representation space, never render pixels | Dreamer V1-V3, TD-MPC2, PlaNet, V-JEPA / V-JEPA 2, DINO-WM | Policy learning via "imagined rollouts" in a learned latent model; representations for downstream tasks | **Real, evidenced on physical quadrupeds** |
| **Explicit / generative** — predict future video | Sora, Genie-class, iVideoGPT, Cosmos-class | Generate video futures; learned simulators | **Not reliable for control yet** (confirmed: they hallucinate physics — objects appearing/disappearing/deforming) |

Two verified cautions that frame everything:
- Whether video-pretrained backbones beat same-size VLM backbones for robot control is
  **an open empirical question** — "suggestive evidence," not an established result.
- A video world model only adds value if it **preserves action consequences and physical
  regularities** — visual quality is not the relevant metric.

## 2. The evidence that matters for a quadruped (verified)

- **DayDreamer (confirmed, arXiv 2206.14176):** Dreamer trained a *physical* quadruped
  to roll off its back, stand, and walk **from scratch in 1 hour, no simulator, no
  resets** — and after training, it adapted to pushes within 10 minutes. Same
  hyperparameters worked across 4 different physical robots.
- **DreamTIP (confirmed, arXiv 2604.02911):** Dreamer-based transfer improved quadruped
  policy transfer +28.1% avg across 8 sim tasks; on a real Unitree Go2, **100% success
  on a 52 cm climb where baseline managed 10%**. (Its sample-efficiency claim — "only ~5
  real trajectories needed" — was **REFUTED** on verification; treat adaptation cost as
  unknown, not tiny.)
- **TD-MPC2 (confirmed):** decoder-free latent world model + MPPI planning, evaluated
  across 104 continuous-control tasks. ⚠ unverified: that it uniquely survives
  38-dim legged tasks where SAC/DreamerV3 blow up; that multi-task training cost ~33
  GPU-days on an RTX 3090.
- ⚠ **MuJoCo Playground (unverified):** open-source MJX/JAX framework claiming
  minutes-scale quadruped policy training on 2×4090 and zero-shot sim-to-real on Go1 and
  others. If true, directly relevant — it's MuJoCo, like our sim.
- ⚠ **Inspection analytics (unverified):** a world-model-backbone video classifier
  claiming >90% accuracy classifying gauge-inspection outcomes (success / known failure /
  anomaly), demoed on a Boston Dynamics Spot; ETH's gauge-reading pipeline claiming <2%
  relative reading error on unseen analog gauges with no per-gauge calibration
  (arXiv 2404.08785).

## 3. Compute reality for OUR robot (RK3588, no NVIDIA GPU)

- **Training is always offboard.** Every evidenced training run used desktop/server GPUs
  (⚠ 33 GPU-days for multi-task TD-MPC2; minutes-to-hours for single-task quadruped
  policies on 4090-class). The RK3588 trains nothing.
- **Onboard inference is possible only for small policies.** Single-task TD-MPC2-class
  policies are ~1-5 M params — int8-quantized via RKNN toolkit on the RK3588's ~6 TOPS
  NPU is the plausible path (⚠ our sizing estimate, not verified benchmarks). A learned
  *gait policy* could run onboard; a video world model cannot.
- **Camera analytics run offboard** (backpack PC or cloud): inspection imagery is not
  latency-critical — a gauge reading that arrives 2 s later is still a gauge reading.

## 4. Ranked adoption plan for us (value ÷ effort, 2-person project)

1. **Camera inspection analytics — adopt FIRST, and it isn't a world model.**
   The near-term value in "world models" for inspection turns out to be ordinary
   specialized vision (⚠ ETH gauge reader) run offboard on camera frames. Pairs exactly
   with our camera task: sim cameras → frame pipeline → gauge/anomaly models later.
   *Effort: low-medium. Blocked by: cameras (in progress).*
2. **Procedural scene randomization for CI gates — adopt the IDEA, skip the ML.**
   Diverse test scenes (obstacle layouts, weather params we already have) generated
   procedurally in MuJoCo harden our gates today; generative world models for this are
   overkill and physics-unreliable (confirmed hallucination risk).
   *Effort: low. Timing: with the 95% gate upgrade.*
3. **Learned rough-terrain gait via model-based RL — the real prize, later.**
   Evidence (DayDreamer confirmed, DreamTIP confirmed, ⚠ Playground) says
   Dreamer/TD-MPC2-class training in MuJoCo is the credible route to the M20's legged
   mode over pipes/stairs — our deferred gait-arbitration phase. Train offboard (MJX),
   distill to a small policy, int8 on the RK3588 NPU.
   *Effort: high. Timing: after phases A-C; needs a GPU box or cloud credits.*
4. **V-JEPA-class features for anomaly detection on patrol video — watch, don't build.**
   ⚠ The Spot demo suggests it works; revisit when our camera pipeline produces real
   patrol footage to classify.
5. **Do NOT adopt: generative video world models as simulators** (Genie/Sora/
   Cosmos-class). Confirmed physics hallucination + open backbone question + our MuJoCo
   already does this job deterministically. Re-evaluate in ~a year.

**Bottom line:** our hand-built MuJoCo sim IS our world model, and for navigation it
beats anything learned we could deploy today. The learned-model wins for us are (a)
cameras + specialized vision for the actual inspection product, and (b) eventually
offboard-trained gait policies — both consistent with the existing roadmap, neither
requiring us to bet the stack on generative models.

## Sources (primary)
- https://arxiv.org/html/2605.00080v1 (robot world-model survey) · https://github.com/tsinghua-fib-lab/World-Model (ACM CSUR survey repo)
- https://arxiv.org/abs/2206.14176 (DayDreamer) · https://arxiv.org/html/2604.02911v1 (DreamTIP) · https://arxiv.org/html/2310.16828v2 (TD-MPC2)
- https://arxiv.org/html/2601.07823v1 (video models as policy evaluators; hallucination)
- ⚠ https://arxiv.org/pdf/2502.08844 (MuJoCo Playground) · ⚠ https://arxiv.org/abs/2404.08785 (ETH gauge reading) · ⚠ https://arxiv.org/abs/2602.16182 (Spot inspection world-model classifier)
