# Operator Station (Phase 1.5 — ADR-013)

The robot serves two websockets whenever the stack is up (`bringup_plan_a.sh`
starts them by default; `M20_STATION=0` disables):

| Port | Protocol | Client |
|---|---|---|
| 8765 | Foxglove WebSocket | Foxglove Studio (live 3D view, plots, panels) |
| 9090 | rosbridge JSON | future web UI + AI agent gateway (ADR-013/014) |

Both are reachable from Windows at `localhost` (WSL2 forwards them).

## One-time setup (Windows)

1. Install Foxglove Studio: https://foxglove.dev/download
   (or `winget install Foxglove.Studio` in a Windows terminal)
2. Open Studio → **Open connection** → **Foxglove WebSocket** →
   `ws://localhost:8765`
3. **Layout → Import from file** → `src/m20_station/config/m20_operator_layout.json`

## What you get

- **3D panel**: live `/map` (the known site map), global/local costmaps, TF,
  `/scan` ring, `/plan` line, robot pose — the same picture the planner sees
- **State panels**: `/m20/mode` (mode + armed), `/m20/failsafe`,
  `/m20/behavior/status`, `/m20/health`
- **Image panel**: `/camera/image_raw` (once the sim camera lands, #11)
- **Teleop panel**: publishes `/cmd_vel` — only honored in MODE_TELEOP
  (Commander enforces; ADR-013)

## Sending a goal from the couch

Foxglove's 3D panel can publish `/goal_pose`; Nav2's bt_navigator subscribes to
it in our launch. Click **Publish pose** in the 3D panel toolbar, click the map
where the robot should go. For multi-waypoint missions use the mission CLI:

    tools/dev/m20 mission "-6,-0.5" "2,4" "0.5,-1"    # patrol, in order
    tools/dev/m20 goto -6 -0.5                        # single goal via mission

## Notes

- The bridge is monitoring-grade video/telemetry. Low-latency drive cam for
  teleop-at-speed is OQ-14 (WebRTC/RTSP relay) — do not teleop fast on the
  Foxglove image feed.
- Fleet rule (ADR-014): everything the station reads/writes goes over these two
  gateways; a future fleet server connects to N of them and never joins DDS.
