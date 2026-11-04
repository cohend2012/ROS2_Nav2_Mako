# Technical Brief: A Learned Navigation System for the M20

**Status:** APPROVED 2026-11-04 (§9) and BUILT (Phases 1-3). How to run it: `docs/AI_NAV_RUNBOOK.md`. Measured gate results: `docs/ai_nav/RESULTS.md`.
**Date:** 2026-11-04
**Scope:** MuJoCo sim → ROS 2 Humble stack (this repo) → Jetson-class deployment
**Supersedes:** the "Cloud-VLM language missions" item filed under *Later* in `ROADMAP.md`

---

## 1. Executive summary

We propose to build **M20-Brain**, a three-layer AI navigation system for the Lynx M20.
Each layer has one job, and each one can be tested and replaced on its own:

| Layer | What it is | Runs at | Output |
|---|---|---|---|
| **L3: Mission agent** | A **VLM** (vision-language model) that calls a fixed set of tools | ~0.1–1 Hz, offboard or cloud | `RunMission` goals, inspection reports |
| **L2: Semantic memory** | A language-indexed map of site objects ("tank 2", "gauge PG-101", "home") | event-driven | map-frame poses keyed by name |
| **L1: Learned local navigator** | An **RL policy** (PPO), trained in our MuJoCo sim | 10 Hz, onboard | **body velocity `(vx, wz)`** on `/cmd_vel` |

The vendor locomotion controller sits **below** L1. We never command joints. This is
what you asked for: the AI's output is a velocity command, the same interface Nav2
uses today and the same one the real robot's vendor controller accepts (ADR-015, OQ-3).

**What kind of system is this?** It is **not an end-to-end VLA**. The system as a whole
is a *hierarchical* vision-language-action system. A VLM plans, and a learned policy
turns those plans into velocity commands. NaVILA [3] uses the same structure for legged
robots: a VLA running at low frequency emits mid-level actions, and an RL locomotion
policy executes them. We start hierarchical on purpose, for the reasons in §3.

**Cosmos:** yes, but as one interchangeable provider for L3, not as the foundation.
Cosmos Reason [8b, 8c] is NVIDIA's open-weight physical-reasoning VLM. It can be served
as an OpenAI-compatible endpoint, so it fits behind the same interface as Claude or any
other VLM. We do **not** adopt Cosmos's generative world models as our simulator. Our
own research found they hallucinate physics, and MuJoCo already does that job
deterministically (`WORLD_MODELS_RESEARCH.md` §4.5).

**The three missions you named, and how far this plan takes each one:**

| Mission | How it works | In this plan? |
|---|---|---|
| "Run the mission and return to where you started" | L3 records the start pose as `home`, runs the waypoints, then sends `goto(home)` | **Yes, Phase 2** |
| "Find the gauges and read the values" | L2 lists the gauges; L1 drives to a stand-off pose facing each one; L3 reads the camera crop with the VLM | **Yes, Phase 3** (sim gauges with known ground truth) |
| "Find the kitchen" (an object or place not in the map) | L3 explores frontiers ranked by VLM relevance, as in VLFM [5], and stores what it finds in L2 | **Phase 4, stretch** |

---

## 2. Where we are starting from (measured, not assumed)

- **The velocity interface already exists.** `/cmd_vel` → `m20_locomotion_bridge` →
  wheel mixer with closed-loop yaw (sim) or vendor SDK (robot). L1 plugs into this chain
  unchanged. Commander veto (ADR-005), the 1 s watchdog, and the velocity smoother all
  stay in place.
- **Classical Nav2 is the baseline to beat:** 10/10 at 0.33 m on single goals
  (checkpoint-05), but the patrol gate fails at 4/5 because of localization drift (M1a).
  **A learned controller does not fix localization drift.** It addresses a *different*
  known failure: DWB declining tight gaps ("No valid trajectories", the skid1 gap and the
  eastern corridor, which is parked).
- **Training on CPU is feasible.** Benchmark run 2026-11-04: the vendor M20 MJCF in
  `oil_gas_field.xml`, driven through the same velocity→wheel mixer as `SimVendorSDK`,
  runs at **~28,500 physics steps/s on one CPU core**. That is 57× real time at 500 Hz,
  or about 570 policy decisions/s per core at 10 Hz. With 4 workers, 1 M decisions takes
  roughly 10–15 min. Local-navigation RL needs a few million, which is hours, not days.
  We do not need a GPU for L1 (we have none: Iris Xe, OQ in Second Brain rev 43).
- **Our mission layer is ready to be driven by an agent.** `m20_missions` serves
  `RunMission` (GOTO / WAIT / INSPECT) with commander veto. L3 becomes a *client* of it.
  It never bypasses it.
- **Finding during benchmarking:** `mujoco_sim.py` computes the spawn height from
  `geom_xpos[:,2].min()`, which includes the floor (z = 0). That spawns the base at
  1.03 m, a ~0.57 m drop. In a driven test from that spawn the body ended at z ≈ 0.07
  (collapsed). From 0.6 m it settles at 0.46 m as documented. The training environment
  will use robot-only geoms. Fixing the shipped sim is filed for owner review, because it
  touches every gate.

## 3. Why hierarchical rather than an end-to-end VLA

End-to-end VLAs (RT-2 [1], OpenVLA [2], Uni-NaVid [4]) map pixels and text straight to
actions. They are the research frontier, but they are the wrong first step for us:

1. **Compute.** OpenVLA is 7B parameters [2]. Uni-NaVid reaches ~5 Hz only with token
   merging on a GPU [4]. Our onboard target is a Jetson Orin NX (or RK3588 NPU), and the
   velocity loop needs ≥10 Hz with a hard watchdog.
2. **Data.** These models train on 10⁵–10⁶ demonstrations [2, 4]. We have none for this
   robot. RL in our own sim produces unlimited, physics-accurate experience for the
   low-level skill [10a, 10c].
3. **Safety and auditability.** For oil and gas inspection, every motion must stay inside
   commander veto and costmap safety. A hierarchy keeps the slow, non-deterministic model
   (the VLM) *outside* the 10 Hz loop. It can only propose goals, which the mission server
   and commander can reject. SayCan [14] showed that grounding language in a robot's
   actual capabilities roughly doubles success compared with ungrounded plans.
4. **Precedent on legged robots.** NaVILA [3] (VLA → mid-level actions → RL velocity
   policy on Unitree Go2/H1) and LLM-planner-over-RL quadruped work [18a] both use this
   split. Skill-Nav [18b] argues that velocity-tracking error is a weakness of the
   interface. We address that by closing the loop on *goals*, not on velocity, at L1.

An end-to-end VLA stays a **Phase 5 research option** (§6). It becomes possible once
L1–L3 have produced logged trajectories we can distill from.

## 4. Layer designs

### 4.1 L1: Learned local navigator (`m20_ai_nav`)

**Task:** reach a local goal within 2–4 m without collision. This is PointNav [17a] with
the goal given relative to the robot.
- **Action:** `(vx, wz)` ∈ [−0.2, 0.4] m/s × [−0.7, 0.7] rad/s. These are the existing
  `velocity_smoother` limits, so the robot cannot be asked for anything Nav2 could not
  already ask for. `vy = 0` (skid-steer wheeled mode).
- **Observation:** 72-bin 2D range scan (the same `/scan` that `pointcloud_to_laserscan`
  produces), goal in the robot frame `(d, sin θ, cos θ)`, last action, measured
  `(vx, wz)`, and a 3-frame history.
- **Algorithm:** PPO [9a] via Stable-Baselines3 [12]. Small MLP (~100 k params).
  Exported to **ONNX**, so the runtime needs only `onnxruntime`, with no PyTorch on the
  robot.
- **Training environment:** a Gymnasium environment around the *real vendor M20 MJCF*,
  driven through a mixer identical to `SimVendorSDK` at 500 Hz. The scene is procedural:
  randomized pipes, skids, and boxes, and a gap-width curriculum. Domain randomization
  [16] covers friction, payload mass, action latency (0–100 ms), scan noise and dropout
  (matching `LIDAR_REALISM`), and odometry slip.
- **Reward:** progress toward the goal, a success bonus, a collision or tip-over penalty,
  and a small smoothness penalty.
- **Integration (owner decision: build all three):** one node, `m20_ai_controller`,
  with three modes selected by `M20_AI_CONTROLLER`. Each mode tracks a **carrot point**
  (1.5 m lookahead) on a path and publishes into the existing velocity smoother.
  - `follow_path`: hybrid under Nav2. Nav2's planner, BT and recoveries stay; our node
    serves `/follow_path` in place of controller_server.
  - `standalone`: our own global planner (A* on `/map`) plus the policy; serves
    `/navigate_to_pose`, with no Nav2 running.
  - `direct`: no map; the policy chases the goal and avoids obstacles reactively.
  With the flag empty, the stack runs master's classical DWB unchanged.
- **Safety envelope:** a hard check outside the policy stops forward motion when any
  return in the ±35° forward cone is closer than 0.60 m. It cannot be disabled. The
  value is 0.60 m because the LiDAR's blind radius is 0.5 m. Below that, an obstacle
  returns nothing, so 3 s of world-frame obstacle memory backs the envelope up.

### 4.2 L2: Semantic memory (`m20_semantics`)

- In v1, a YAML **landmark registry** maps a name to a map-frame pose, an approach pose,
  and object attributes (type: gauge/tank/valve, units, range). It is seeded from the
  site survey, or in sim from `oil_gas_field.xml`.
- `home` is written automatically when a mission starts.
- In v2, perception adds entries at runtime: a detection plus a map pose, merged by
  distance. This is the object-centric path of ConceptGraphs [6b] and VLMaps [6a],
  scaled down to a site inventory.

### 4.3 L3: Mission agent (`m20_agent`)

- The agent uses tools against the existing stack: `list_landmarks()`, `goto(name)`,
  `inspect(name)` (go to the approach pose, capture a frame, ask the VLM),
  `read_gauge(image, spec)`, `return_home()`, and `report()`.
- Every motion tool builds `MissionItem`s and calls `/m20/mission/run`. The agent
  **cannot** publish velocity commands or change modes (ADR-005 holds).
- **Provider-agnostic VLM client**, one interface with three backends:
  - **Claude** (Anthropic API, cloud), the default for development.
  - **Cosmos Reason 2 / Cosmos 3 Reasoner** through any OpenAI-compatible endpoint:
    `vllm serve` or an NVIDIA NIM [8b–8d]. This is how we "hook up Cosmos". The 2B/8B
    variants [8c] are the Jetson-class on-device candidates.
  - **Offline rule-based planner** with no model at all. CI uses it, and the robot falls
    back to it when the link drops.
- **Gauge reading:** a VLM reads the cropped dial and returns the value, units, and
  confidence. Every reading goes in the report with its image. Below a confidence
  threshold, the reading is flagged for a human; it is never silently accepted. The ETH
  pipeline [13] (<2% relative error, no per-gauge calibration) is the specialized
  upgrade path if VLM accuracy falls short.

## 5. Evaluation: gates in the style of this repo

| Gate | Measured on | Pass bar |
|---|---|---|
| **G1: Policy in training sim** | 500 held-out procedural episodes | ≥95% success, ≤2% collision, SPL [17a] ≥ 0.80 |
| **G2: Policy vs DWB, offline** | same seeds, scripted baseline controller | success ≥ baseline on the gap-width sweep; strictly better on gaps < 1.2 m |
| **G3: In the stack** | existing `batch_nav_test.sh` with `M20_AI_CONTROLLER=1` | ≥9/10, mean true error < 0.5 m (the same bar classical Nav2 had to clear) |
| **G4: Language missions** | 10 scripted prompts including "patrol and return home" | 10/10 parsed to valid missions; home error < 0.5 m |
| **G5: Gauge reading** | 6 sim gauges with known needle values, random approach | ≥5/6 within 5% of full scale; 0 unflagged wrong readings |

## 6. Phased plan and how far we go

| Phase | Deliverable | Verifiable in this environment? |
|---|---|---|
| **1: L1 navigator** | Gymnasium env, PPO training script, trained model + ONNX, evaluation report (G1, G2), ROS 2 controller node + unit tests | **Yes, fully.** CPU training and evaluation run here. |
| **2: Agent + memory** | landmark registry, VLM client (Claude / Cosmos-compatible / offline), agent tools over `RunMission`, "patrol and return home" | Logic and unit tests yes (offline backend). Live VLM calls need your API key or endpoint. |
| **3: Gauges** | procedurally textured gauge geoms in a sim scene, `inspect` / `read_gauge`, G5 harness | Scene and harness yes. G5 needs a VLM endpoint. |
| **4: Open-vocabulary exploration** ("find the kitchen") | VLFM-style [5] frontier scoring + runtime landmark insertion | stretch, designed only |
| **5: End-to-end VLA research** | distill logged L1–L3 trajectories into a NaVILA/Uni-NaVid-style model [3, 4] | needs a GPU box. Not now. |

**What "deploy" means for you:** G3 and G4 run on your WSL box with the existing
bringup. The ONNX policy and the agent go into the `m20_autonomy` image as two new
colcon packages, with no new heavy dependencies: `onnxruntime`, plus `anthropic` or
`openai` used only by the agent. Training is a separate offline script, kept out of the
runtime image (Engineering Rule 4: long jobs are chunked and resumable, with outputs on
persistent storage).

## 7. Risks

| Risk | Mitigation |
|---|---|
| Sim-to-real gap in L1 (skid-steer slip is hard to model) | domain randomization [16]; Nav2/DWB stays one flag away; L1 only produces velocity commands, which the vendor controller tracks |
| Learned controller is worse than DWB in open field | G2/G3 require it to *beat* the baseline before it ships; the hybrid design lets us use it only in tight spaces if needed |
| VLM hallucinates a reading or a goal | tools only accept registry names; readings carry confidence + image; commander veto |
| Cloud dependency in the field | offline planner fallback; Cosmos Reason 2B/8B on-device path [8c] |
| Training cost on CPU grows | a GPU path exists without redesign: MuJoCo Playground / MJX [11] |
| Licence | Cosmos weights: NVIDIA Open Model License (Reason 1/2) [8b, 8c]; Cosmos 3 may be OpenMDW 1.1 (unconfirmed [8d]). Check before commercial use. |

## 8. What this does *not* fix

The M1a patrol drift is a **localization** problem. No navigation policy fixes it, and
missions that "return to start" inherit it. Its fix (runtime drift bounding) stays the
M1 blocker in `ROADMAP.md`. The AI work runs as a parallel track, the same way the
two-track plan did in July.

## 9. Decisions requested from the owner

Decided by the owner on 2026-11-04:
1. **Scope:** build Phases 1–3. Phase 4 ("find the kitchen") is designed but not built.
2. **VLM:** both providers behind one interface, with Claude as the default and Cosmos
   through an OpenAI-compatible endpoint. The offline backend is always present as a
   fallback.
3. **Integration:** "do both, hybrid, local and global". All three modes are built
   (§4.1): under Nav2, standalone with our own global planner, and direct.

---

## References

Every entry was checked against the publisher, arXiv, JMLR, or vendor pages on
2026-11-04. Items marked (†) are confirmed by a single secondary source.

1. Brohan, A. et al. *RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control.* CoRL 2023, PMLR 229. arXiv:2307.15818. Source of the term "VLA".
2. Kim, M. J. et al. *OpenVLA: An Open-Source Vision-Language-Action Model.* 2024. arXiv:2406.09246.
3. Cheng, A.-C. et al. *NaVILA: Legged Robot Vision-Language-Action Model for Navigation.* RSS 2025. arXiv:2412.04453. († exact velocity set used by its low-level policy)
4. Zhang, J. et al. *Uni-NaVid: A Video-based Vision-Language-Action Model for Unifying Embodied Navigation Tasks.* RSS 2025. arXiv:2412.06224.
5. Yokoyama, N. et al. *VLFM: Vision-Language Frontier Maps for Zero-Shot Semantic Navigation.* ICRA 2024. arXiv:2312.03275.
6. (a) Huang, C. et al. *Visual Language Maps for Robot Navigation.* ICRA 2023. arXiv:2210.05714. (b) Gu, Q. et al. *ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning.* ICRA 2024. arXiv:2309.16650.
7. (a) Shah, D. et al. *ViNT: A Foundation Model for Visual Navigation.* CoRL 2023. arXiv:2306.14846. (b) Sridhar, A. et al. *NoMaD: Goal Masked Diffusion Policies for Navigation and Exploration.* ICRA 2024. arXiv:2310.07896.
8. (a) NVIDIA. *Cosmos World Foundation Model Platform for Physical AI.* arXiv:2501.03575. (b) NVIDIA. *Cosmos-Reason1: From Physical Common Sense to Embodied Reasoning.* arXiv:2503.15558. (c) NVIDIA. Cosmos Reason 2 (2B/8B/32B). https://github.com/nvidia-cosmos/cosmos-reason2. (d) NVIDIA. *Cosmos 3* press release, 2026-06-01 (Nano 16B / Super 64B; Reasoner NIM). († licence)
9. (a) Schulman, J. et al. *Proximal Policy Optimization Algorithms.* arXiv:1707.06347, 2017. (b) Haarnoja, T. et al. *Soft Actor-Critic.* ICML 2018. arXiv:1801.01290.
10. (a) Rudin, N. et al. *Learning to Walk in Minutes Using Massively Parallel Deep RL.* CoRL 2021. arXiv:2109.11978. (b) Rudin, N. et al. *Advanced Skills by Learning Locomotion and Local Navigation End-to-End.* IROS 2022. arXiv:2209.12827. (c) Hoeller, D. et al. *Learning a State Representation and Navigation in Cluttered and Dynamic Environments.* RA-L 2021. arXiv:2103.04351.
11. Zakka, K. et al. *MuJoCo Playground.* RSS 2025. arXiv:2502.08844. MJX docs: https://mujoco.readthedocs.io/en/stable/mjx.html
12. Raffin, A. et al. *Stable-Baselines3: Reliable Reinforcement Learning Implementations.* JMLR 22(268), 2021.
13. Reitsma, M. et al. *Under Pressure: Learning-Based Analog Gauge Reading in the Wild.* ICRA 2024. arXiv:2404.08785.
14. Ahn, M. et al. *Do As I Can, Not As I Say: Grounding Language in Robotic Affordances.* CoRL 2022. arXiv:2204.01691.
15. Macenski, S. et al. *The Marathon 2: A Navigation System.* IROS 2020. arXiv:2003.00368.
16. Tobin, J. et al. *Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World.* IROS 2017. arXiv:1703.06907.
17. (a) Anderson, P. et al. *On Evaluation of Embodied Navigation Agents.* arXiv:1807.06757, 2018 (SPL). (b) Batra, D. et al. *ObjectNav Revisited.* arXiv:2006.13171, 2020.
18. (a) *Adaptive Interactive Navigation of Quadruped Robots using Large Language Models.* arXiv:2503.22942, 2025. (b) *Skill-Nav: Enhanced Navigation with Versatile Quadrupedal Locomotion via Waypoint Interface.* arXiv:2506.21853, 2025.
19. Bjorck, J. et al. *GR00T N1: An Open Foundation Model for Generalist Humanoid Robots.* arXiv:2503.14734, 2025. A humanoid manipulation VLA, cited for classification only.
