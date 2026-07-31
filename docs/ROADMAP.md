# M20 Execution Roadmap — the single tracking document

> Rule that keeps us honest: work happens ONLY inside the CURRENT milestone,
> except (a) a red gate anywhere, (b) the user redirects. Every session starts
> by reading this file and ends by updating the checkboxes. If it isn't a
> milestone task, it's a distraction — file it under Later and move on.
> (SECOND_BRAIN §5 keeps the 10-phase vision; THIS file is what we execute.)

**CURRENT MILESTONE: M1 — Site Tour**

---

## M1 — Site Tour: drive well, everywhere (sim)
The gate goal has been (-6,-0.5) for two weeks — tuning may be overfit to one
route. Prove the whole site.
- [ ] Patrol gate script (`tools/ci/patrol_gate.sh`): one MISSION of 5 diverse
      goals (W field, NW, N lane, E field, home) via the mission server;
      per-leg true error logged
- [ ] **GATE M1a: 5/5 legs complete, mean true error < 0.5 m, no collisions**
- [ ] Two-panel tour video (replay | planner view) — the "driving around" demo
- [ ] Eastern corridor (7.5, 4.0): one attempt per session, tracked not forced;
      promote to gate only when it passes twice
- [ ] **GATE M1b: L3.3 (goal behind a skid, clearance > 0.15 m) + L3.4
      (obstacle appears mid-run, recovers) measured**

## M2 — Robustness: the boring gate that makes outdoor possible (sim)
- [ ] 25-run mixed-goal batch (goals sampled from the M1 set): ≥ 92% success,
      mean < 0.5 m — catches the ~1-in-10 excursion class for real
- [ ] Excursion forensics from archived per-run logs; fix or bound the class
- [ ] Camera-vs-control-loop A/B batch (camera on vs off, same goals)
- [ ] GPS-denied map build scored (carried from queue) — denied-mode posture
      number in SECOND_BRAIN
- [ ] **GATE M2: the 25-run bar above, with camera ON**

## M3 — Deploy pipeline: morning sim, afternoon robot (infra)
Per docs/SIM_TO_REAL.md; partially started.
- [x] Staleness assertion in bringup (commander repo-vs-installed)
- [ ] Image rebuild completes + FS_TIPOVER verified in baked artifact (in flight)
- [ ] CI: image build + `make unit` + L1 smoke on every merge to master
- [ ] `platform:=sim|robot` launch surface (one flag → backend/sensors/overlay)
- [ ] /clock + use_sim_time wiring
- [ ] buildx multi-arch (amd64+arm64) + registry push (Jetson prep)
- [ ] **GATE M3: fresh clone → one command → green L1 smoke, twice in a row**

## M4 — Hardware bench (robot arrives; tethered, indoor)
Entry requires M2 + M3 green. This is Phase 6's front door.
- [ ] OQ-3 closed: on-robot `ros2 topic list` — does the vendor expose a
      cmd_vel tier, or do we command via drdds joint tier + our mixer?
- [ ] OQ-4 closed: physical e-stop state visible on the bus; commander consumes it
- [ ] Bridge C++ port (P1) if the joint tier is the path (200 Hz)
- [ ] L1.1–L1.6 replayed ON HARDWARE, tethered, e-stop supervised
- [ ] **GATE M4: teleop + standup + single indoor goal on hardware, all
      failsafes demonstrated live**

## M5 — Outdoor testing (the answer to "are we ready?")
Entry requires M4 green. NOT before — see readiness assessment below.
- [ ] Site survey + GPS datum; real map built on-site (build_map pipeline,
      landmark gate against surveyed references)
- [ ] robot_localization dual-EKF + navsat_transform live (Phase A hardware form)
- [ ] Outdoor L3.2 batch, tethered → untethered on gate pass
- [ ] **GATE M5: 10/10 outdoor goals, geofenced, e-stop drill passed**

## Later (filed, deliberately not now)
Cloud-VLM language missions (Phase 7 opener — after M1, it demos on the tour) ·
MPPI controller trial · slow-align tuning · commander C++ port (P2) ·
Foxglove custom panels · fleet items (Phase 10)

---

## Outdoor readiness: honest answer as of 2026-07-31
**No — we are 3 gates away (M2, M3, M4), and that is the correct order.**
What outdoor needs that we don't have: any hardware integration at all (OQ-3/
OQ-4 open), a deploy pipeline that guarantees the code on the robot is the
code we tested (M3 — the L1.6 staleness incident is exactly what outdoor
cannot tolerate), robustness proof beyond one goal (M2), real-GPS frame
conversion, and terrain — the sim field is FLAT; slopes/grass/rain are
Phase-B territory and outdoor site selection must respect that.
What we DO have that outdoor needs: gate-proven nav (10/10 @ 0.33 m), a
QA'd map pipeline, commander failsafes with a passed tip-over drill, an
operator station, and the mission layer. The path is real; it's M2→M3→M4.
