# Taking the M20 into an office

What is ready in software, what has to happen on the day with the robot, and what
people will actually ask it to do. Companion to `AI_NAV_RUNBOOK.md` (how to run
things) and `ai_nav/RESULTS.md` (measured numbers, including the office gates).

## 0. Honest status
- **Recommended indoor configuration: classical Nav2 (DWB) + `M20_SITE=office` + the
  agent.** Verified on real ROS 2 Humble + Nav2 in the office world: reception → meeting
  room through a **0.85 m door** (0.16 m final error), plus the agent mission. With the
  outdoor params (0.45 m circle), the same run fails: DWB "Failed to make progress" at
  the door, repeatedly. The office profile is what makes doors possible.
- **The learned controller indoors is EXPERIMENTAL.** It is gated, not assumed; the
  numbers are in `ai_nav/RESULTS.md` (office section). Use it only after it beats the
  classical run in your office, as step E below.
- **Also verified:** the footprint-aware safety shield, door-straight path planning,
  taught places, routines and STOP (unit tests plus the ROS smoke).
- **Hardware: not connected yet.** The bridge's real-robot backend (`VendorSDK` in
  `m20_locomotion_bridge/bridge_node.py`) is still the Phase-0 stub. On the real M20
  the whole stack runs and nothing moves. That is safe, but it isn't a deployment.
  Closing **OQ-3** (what velocity/gait interface the vendor exposes on the robot) is
  step 1 below, and no AI work substitutes for it.

## 1. Why the office needed its own profile (measured)
| Fact | Consequence | What we changed |
|---|---|---|
| The M20 is **0.91 m long x 0.53 m wide** at stance (vendor MJCF) | Nav2's 0.45 m circle needs 0.9 m of clearance, so every 0.8-0.9 m door is a wall | `M20_SITE=office`: the real rectangle as the Nav2 footprint, inflation 0.35 m |
| A pivot sweeps the corners through a **0.53 m** circle | Turning beside a wall clips it | A footprint trajectory guard (real rectangle, 0.6 s look-ahead) in `core.shield()` |
| The v1 policy never saw corridors, doors or people | Procedural offices: 40% success, 40% collisions | Office procedural training **inside the shield** (v2) |
| No map means no route through doors | `direct` mode walks into walls indoors | `direct` is refused with `M20_SITE=office` |
| No GPS indoors | The oil-field map builder is GPS-anchored | Teleop mapping + `tools/map/save_map.sh` |
| People share the corridor | | 0.3 m/s cap; walking people in training (O3 gate) |

## 2. Office day: in order, nothing skipped
**A. Bench (robot on a stand or in a cleared room, e-stop in hand)**
1. **OQ-3, the bridge.** On the robot, run `ros2 topic list` and read the vendor SDK
   docs. Implement `VendorSDK.connect / send_velocity / stop` (one file, already
   isolated). Start the bridge with `sdk_backend:=real`. *Test:* a 0.1 m/s command
   in teleop moves the robot 10 cm/s, and the 1 s watchdog stops it when commands
   stop.
2. **OQ-4, the physical e-stop on the bus.** The commander must see it. *Test:* the
   e-stop latches ESTOP and disarms; re-arming needs an operator.
3. **Sensors.** The real `/LIDAR/POINTS` gives a `/scan` with the robot NOT in it.
   Check one sample by eye (engineering rule 2). For the camera, find the real topic
   and encoding (OQ-14) and point the agent's `/camera/image_raw` at it.

**B. Mapping (teleop only, at walking pace)**
```bash
M20_SITE=office M20_NO_GOAL=1 bash tools/nav/bringup_plan_a.sh   # live SLAM, no goal
# drive every corridor and through every door you'll use; come back past the start
bash tools/map/save_map.sh office                                   # -> maps/office.*
```
Check `maps/office.pgm` by eye: are the walls straight, the doors open, and no
doubled corridors?

**C. Classical first (the AI is not the first thing to move the robot indoors)**
```bash
M20_SITE=office M20_STATIC_MAP=maps/office bash tools/nav/bringup_plan_a.sh   # DWB
```
Send single goals through doors with an operator walking alongside.

**D. Teach the places (by voice or text, standing at each spot)**
```bash
M20_SITE=office M20_STATIC_MAP=maps/office M20_AGENT=offline M20_NO_GOAL=1 bash tools/nav/bringup_plan_a.sh
# teleop the robot to the kitchen, then:
bash tools/agent/say.sh "this is the kitchen"
bash tools/agent/say.sh "remember this spot as the meeting room"
bash tools/agent/say.sh "call this the dock"
```
Taught places are saved to `/cfg/out/agent/taught_places.yaml` and survive restarts.
They can never overwrite a surveyed landmark. For your real office, start from an
empty landmark file or copy `office_landmarks.yaml` and replace its rooms.

**E. The learned controller (after C works)**
```bash
M20_SITE=office M20_STATIC_MAP=maps/office M20_AI_CONTROLLER=follow_path M20_AGENT=offline bash tools/nav/bringup_plan_a.sh
bash tools/agent/say.sh "go to the kitchen and come back"
bash tools/agent/say.sh "stop"          # always: interrupts immediately, never goes through a model
```
Watch `/m20/ai_nav/state`. A lot of `BLOCKED footprint guard` means the shield is
doing the driving. Stop and report it; don't push on.

## 3. What people actually ask (supported today)
| Say | What happens |
|---|---|
| "this is the kitchen" / "remember this spot as X" / "call this X" | Saves the CURRENT pose as a place. The robot doesn't move. |
| "go to the kitchen" / "take me to the meeting room" | Drives there. An unknown place is refused: *"I don't know where 'printer' is. Drive me there and say 'this is the printer'."* |
| "go to the kitchen and come back" | Goes there and returns to where this mission started. |
| "start the mission" / "do your rounds" | Runs the default routine (office: look into every room, end at the dock). |
| "run the coffee run" | Runs a named routine from the landmark file. |
| "check the server room" / "inspect the meeting room" | Goes there, takes a photo and describes it. The VLM backend gives real answers; offline only saves the photo. |
| "stop" / "cancel" / "halt" | Cancels the motion in flight and skips the rest of the mission, including the return home. |
| "find the kitchen" (never taught) | Refused with the how-to-teach message. Exploring an unknown building is Phase 4 (designed, not built). |

Routines live in the landmark YAML under `routines:` with `default_routine:`; see
`src/m20_agent/config/office_landmarks.yaml`.

## 4. Known gaps (do not discover these in front of people)
- **Glass** walls and doors are invisible to the LiDAR, and to the map. Open item: a
  Nav2 keepout mask. Until then, tape the glass at 0.1-1.0 m or keep routes away.
- **Stairs and steps:** wheeled mode only. Treat any level change as a wall (keepout).
- **Thin and overhanging things:** chair legs give sparse returns; desk tops above
  1.0 m and hanging objects are outside the scan slice (0.10-1.00 m).
- **People:** trained with pedestrians who stop politely near the robot. Running
  children, crowds and people stepping into it at the last moment are not modelled.
  The 0.3 m/s cap and the shield are the mitigation.
- **Docking and charging:** "dock" is a named place only. There is no auto-docking.
- **Lifts and automatic doors:** not supported.
