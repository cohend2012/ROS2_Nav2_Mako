# Sim-to-Real: morning sim, afternoon deploy

Goal: simulate a change in the morning, deploy the SAME change to the robot in
the afternoon, with the stack knowing — explicitly — which world it is in.
Target robot compute: NVIDIA Jetson (Orin NX class, ARM64, JetPack/L4T).

## The six rules

1. **One artifact.** The Docker image is the unit of deployment (ADR-011). CI
   rebuilds it on every merge to master; the robot pulls by digest. Nothing on
   the robot path runs from docker-cp'd files — the /cfg staging pattern in
   bringup_plan_a.sh is a DEV convenience only. (Enforced need: the 2026-07-28
   L1.6 bench test found the image-baked commander PREDATED the tip-over
   failsafe — an inverted robot stayed armed. The artifact must contain the
   code; freshness must be checked, not assumed.)

2. **Platform is a launch argument, not a code branch.** `platform:=sim|robot`
   selects exactly three things:
   - bridge backend (`sdk_backend:=sim` ↔ real drdds)
   - sensor source (MuJoCo sim node ↔ vendor drivers)
   - a per-platform param overlay (`config/sim/`, `config/robot/`) for the few
     gains that legitimately differ
   Any other code that asks "am I in sim?" is a design smell. The topic
   contract is IDENTICAL by construction — that is the entire point of the sim
   speaking the vendor interface.

3. **Declare, then assert.** At bringup the stack verifies the declared
   platform against reality and DIES LOUDLY on mismatch:
   - platform=robot but no /JOINTS_DATA within 5 s → abort
   - platform=sim but the vendor's onboard DDS participants visible → abort
   - dev bringup: repo source vs installed source hash per m20 node → warn/abort
   Never guess the platform from heuristics; declaration + assertion only.

4. **Adopt /clock (`use_sim_time`).** Today sim runs wall-clock at ~1×, so we
   get away with `use_sim_time:=false`. That breaks the moment sim ≠ 1× (CI on
   fast hardware, replay debugging, slow Jetson sim). Sim publishes /clock;
   every node takes use_sim_time from the platform flag. This is currently an
   OPEN GAP — file it with any timing-related weirdness.

5. **Jetson/ARM64 build discipline.**
   - `docker buildx` multi-arch: linux/amd64 (dev box, CI) + linux/arm64
     (Jetson); perception layers based on l4t/JetPack CUDA images (OQ-13 pins
     the exact JetPack).
   - A container registry (ghcr) sits between CI and the robot. Deploy =
     `docker pull` + `docker compose up -d`. Rollback = previous digest.
   - No x86-only wheels (onnxruntime-gpu, opencv builds): every dependency
     must resolve on both arches or live behind the platform overlay.

6. **Gates make same-day deploys safe.** Morning: the change passes the same
   ladder in sim (L1.x smoke → single goal → L3.2 batch if control-path).
   Afternoon: deploy only on green; the robot re-runs L1.1/L1.2/L1.4/L1.5
   (cheap hardware smoke) before any mission. A red gate blocks the deploy —
   no exceptions, that is the contract that makes speed safe.

## Migration debts (tracked)
- [ ] CI image rebuild on merge + digest-pinned deploys (replaces ad-hoc builds)
- [ ] Staleness assertion in bringup (repo vs installed, per m20 node)
- [ ] /clock + use_sim_time wiring
- [ ] platform:=sim|robot launch surface (bringup flags → one argument)
- [ ] buildx multi-arch + registry
- [ ] Per-platform param overlays (config/sim, config/robot)
