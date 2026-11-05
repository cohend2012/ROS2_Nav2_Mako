---
name: m20-sim
description: Bring up the M20 MuJoCo + ROS 2 sim stack (outdoor oil field or office) with classical Nav2, the learned controller, and/or the language agent, then drive it. Use when asked to "run the sim", "start the stack", "test in sim", "send the robot to X in sim", or to reproduce a navigation behaviour.
---

# Bring up and drive the M20 sim

Everything runs in Docker inside WSL2 Ubuntu on the owner's box (README "Running the
system"). Container `docker-commander-1` hosts the stack; `m20_sim:latest` runs MuJoCo.
`tools/nav/bringup_plan_a.sh` is the ONE bringup (engineering rule: no forked copies).
Every option below is a flag on it; with no flags you get master's behaviour.

## 1. Pick the configuration (environment flags)
| Want | Flags |
|---|---|
| Outdoor site, classical Nav2 (the gated default) | `M20_STATIC_MAP=maps/oil_gas_field` |
| Office, classical Nav2 (recommended indoors) | `M20_SITE=office M20_STATIC_MAP=maps/office` |
| A classical office variant | add `M20_NAV_VARIANT=dwb\|shim_rpp\|rpp_cm\|lattice_rpp\|mppi` |
| Learned controller under Nav2 | add `M20_AI_CONTROLLER=follow_path` |
| Learned controller + our A*, no Nav2 | add `M20_AI_CONTROLLER=standalone` |
| Language agent | add `M20_AGENT=offline\|claude\|cosmos` (+ `ANTHROPIC_API_KEY` for claude) |
| Gauge reading | start the sim with `M20_CAMERA=1` and the gauge scene (`tools/sim/make_gauge_scene.py`) |
| Stack up, no goal sent | `M20_NO_GOAL=1` |

Rules the script enforces: `M20_AI_CONTROLLER=direct` is refused with `M20_SITE=office`
(no map means it can't route through doors), and `M20_EKF_TF` with `M20_STATIC_MAP`
is refused.

## 2. Run
```bash
# from WSL, repo root (Windows: wsl -d Ubuntu -- bash -c "cd <repo> && ...")
M20_SITE=office M20_STATIC_MAP=maps/office M20_AGENT=offline M20_NO_GOAL=1 bash tools/nav/bringup_plan_a.sh
bash tools/agent/say.sh "go to the kitchen and come back"      # agent missions
bash tools/ros2.sh action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"
bash tools/nav/kill_stack.sh                                    # always tear down when done
```
Logs land in the container at `/cfg/out/*.log`: `nav2.log` or `ai_nav.log`, plus
`agent.log`, `mission.log`, `slam.log`. Copy them out with
`docker cp docker-commander-1:/cfg/out tools/nav/out`.

## 3. Watch
- `bash tools/ros2.sh topic echo /m20/ai_nav/state`: TRACKING / ALIGNING / BLOCKED / STUCK
  plus the reason. Lots of `BLOCKED footprint guard` means the shield is doing the driving.
- `bash tools/ros2.sh topic echo /m20/agent/report`: the mission report (JSON).
- Foxglove at `ws://localhost:8765`.

## 4. Before you claim anything works
Quiet box (`uptime` load < 0.5). One experiment per session. Archive the logs. Never
call a single run "passing". Use the gates in the `m20-nav-gates` skill.
