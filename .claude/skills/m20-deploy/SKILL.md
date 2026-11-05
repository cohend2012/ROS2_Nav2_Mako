---
name: m20-deploy
description: Build the m20_autonomy Docker image and bring up the M20 stack with docker compose for a site (outdoor or office), including perception (SLAM), navigation (classical or AI), and the language agent; check health; follow the robot-day order. Use when asked to "deploy", "build the container", "compose up", "put it on the robot", or "prepare for the office".
---

# Deploy the M20 stack in Docker

One image (`m20_autonomy:latest`, `docker/Dockerfile`) is the artifact. CI builds it and
the robot runs the same image (ADR-011). Services live in `docker/compose.yaml` (host
network, CycloneDDS, `ROS_DOMAIN_ID=42`).

## 1. Build
```bash
docker compose -f docker/compose.yaml build          # ~10 min first time; needs packages.ros.org
python3 -m pytest tests/test_ai_nav.py tests/test_agent.py -q   # before shipping anything
```
The image contains: Nav2 and every office variant's plugins (Smac, RPP, rotation shim,
MPPI, collision monitor), slam_toolbox, pointcloud_to_laserscan, all `m20_*` packages,
onnxruntime + the VLM clients, and the trained policies in
`m20_ai_nav/share/models/`. Training tools (torch, MuJoCo) are deliberately NOT included.

## 2. Configure, then bring up (env vars, or a `docker/.env` file)
| Var | Values | Meaning |
|---|---|---|
| `M20_SITE` | `outdoor` \| `office` | office = real 0.91 x 0.53 m footprint, 0.3 m/s, collision monitor on |
| `M20_MAP` | e.g. `office` | localize on `maps/office.posegraph`; unset = mapping mode |
| `M20_NAV_MODE` | `nav2` (default) \| `follow_path` \| `standalone` | classical Nav2, or the learned controller |
| `M20_NAV_VARIANT` | `dwb` (default) \| `rpp_cm` \| `shim_rpp` \| `lattice_rpp` \| `mppi` | classical variant |
| `M20_MODEL` | path to `.onnx` | AI modes; default `m20_nav_policy.onnx` (v1); v3 is `m20_nav_policy_v3.onnx` |
| `M20_AGENT` | `offline` \| `claude` \| `cosmos` | language backend (+ `ANTHROPIC_API_KEY` / `M20_COSMOS_*`) |
| `M20_LANDMARKS` | `office_landmarks.yaml` or `/data/<file>` | the agent's place list and routines |

```bash
M20_SITE=office M20_MAP=office M20_AGENT=offline M20_LANDMARKS=office_landmarks.yaml \
  docker compose -f docker/compose.yaml up -d
docker compose -f docker/compose.yaml ps        # commander/bridge/mission/behavior/agent: (healthy)
docker compose -f docker/compose.yaml logs -f nav
```
Persisted on the host: `maps/` (mounted into `perception`) and `data/` (taught places
and mission reports, mounted into `agent`).

## 3. Recommended configurations (from measured gates; see `m20-nav-gates`)
- **Outdoor missions:** `M20_SITE=outdoor M20_NAV_MODE=nav2`. Checkpoint-05: 10/10.
- **Office:** `M20_SITE=office M20_NAV_MODE=nav2` with the variant the latest office
  batch gate chose. Read `docs/ai_nav/office_variants/RESULTS.md` and
  `docs/ai_nav/office_batch/`.
- **AI picks the waypoints, Nav2 drives:** any of the above plus `M20_AGENT`.
- **Learned controller (`M20_NAV_MODE=standalone|follow_path`):** only under supervision
  until its batch gate passes.

## 4. Robot day (docs/OFFICE_DEPLOYMENT.md §2), in order. Do not skip.
1. **OQ-3 bridge.** `VendorSDK` in `m20_locomotion_bridge/bridge_node.py` is still a stub,
   so nothing moves until it's implemented. Test 0.1 m/s in teleop and the 1 s watchdog.
2. **OQ-4 e-stop on the bus**, consumed by the commander.
3. Sensors: look at one `/scan` sample by eye. Find the real camera topic (OQ-14).
4. Map by teleop (perception with `M20_MAP` unset), then `bash tools/map/save_map.sh office`.
5. Classical Nav2, single goals, operator walking alongside.
6. Teach places: `bash tools/agent/say.sh "this is the kitchen"` at each spot.
7. Agent missions. The learned controller comes last, supervised.

## 5. Health and rollback
- `docker/healthcheck.py` checks for a `/m20/health` heartbeat (commander, bridge,
  mission server, behaviors, agent, AI controller). Compose restarts any that go silent.
- Rollback = the previous image tag. Never hot-patch files inside a running container.
