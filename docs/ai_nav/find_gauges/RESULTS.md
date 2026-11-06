# Find the gauges with no prior positions (gate G6)

**Date:** 2026-11-05. **Scene:** `oil_gas_field_gauges.xml` (vendor M20 physics, 6 dial
gauges). **Driver:** A* + NavCore + learned policy v1 (`m20_nav_policy.onnx`). **Reader:**
offline classical needle fit. **Code under test:** the agent's own
`m20_agent.gauge_search.search_and_read` (the `find_gauges` plan action); only the
robot interface is the simulator's.

The robot starts with **every gauge removed from its landmark file** and is scored
against `tools/sim/gauges/truth.json`, which it never sees.

```bash
MUJOCO_GL=osmesa python3 tools/agent/find_gauges.py --out <dir> --seed N
```

## What it does
1. **Survey.** Coverage viewpoints over the free map, one per 2 m cell, nearest-first (29 here).
2. **Look around.** At each viewpoint the robot turns to 8 headings and takes a camera frame at each.
3. **Detect.** `detect_dials` looks for round, bright faces with a red needle.
4. **Localize.** Pixel → bearing; the LiDAR range along that ray gives a map point. Two
   more rays at ±1.5° give the face normal. Sightings within 0.6 m are clustered into one candidate.
5. **Verify and read.** The robot drives to a spot 1.2 m in front of the face, aims, and
   confirms a dial near the image centre. If none is confirmed the candidate is REJECTED
   and never saved. It then reads the dial, retrying at 0.9 m if the reading isn't confident.
6. **Save and report.** Each confirmed dial becomes `found_gauge_N` in the taught overlay
   (`seed5/taught.yaml`), with a viewing spot, so "read all the gauges" works on later missions.

## Results, 7 seeds (randomized friction, mass, latency, LiDAR noise)

| Seed | Found (localized < 0.6 m) | Read within 5 % FS | False positives | Confident-wrong | Survey points reached | Sim time |
|---|---|---|---|---|---|---|
| 1 | 5/6 | 5/6 | 0 | 0 | 28/29 | 1180 s |
| 2 | 6/6 | 6/6 | 0 | 0 | 29/29 | 1108 s |
| 3 | 6/6 | 6/6 | 0 | 0 | 29/29 | 1179 s |
| 4 | 6/6 | 6/6 | 0 | 0 | 29/29 | 1196 s |
| 5 | 6/6 | 6/6 | 0 | 0 | 29/29 | 1184 s |
| 6 | 6/6 | 5/6 | 0 | 0 | 29/29 | 1238 s |
| 7 | 5/6 | 5/6 | 0 | 0 | 29/29 | 1091 s |
| **Total** | **40/42 (95 %)** | **39/42 (93 %)** | **0** | **0** | 202/203 | |

- Position error of every found gauge: about 0.01 m. Worst reading error: 0.67 % of full scale.
- Misses:
  - Seed 1, `gauge_fl401`: never sighted; survey point 22 next to it was unreachable (stuck 0.31 m short).
  - Seed 7, `gauge_pg101`: never sighted.
  - Seed 6, `gauge_ti201`: localized, but the drive to its viewing spot stalled at the
    start of a 10.8 m leg, so it was reported UNREACHED (not read, not guessed).
- 0 collisions and 0 tip-overs across all seven runs.
- Per-seed rows: `seeds.json`. Seed-5 up-close frames: `seed5/`.

## Fixes this gate forced (each visible in the history above)
| Symptom | Cause | Fix |
|---|---|---|
| Robot "tipped over" on flat ground (2 of 5 early seeds) | A stalled turn wound the mixer's yaw trim up to ~4× the request, and the wheel torque rolled the legs | Bounded trim (±0.6 rad/s) with anti-windup, in the training mixer and the sim bridge (parity-tested). Reproduced 4/4 from a snapshot, 0 after. Policy gates improved: G1 98.8 % (tipped 1.0 → 0 %), field 96 % (tipped 2 → 0 %) |
| Only ~5 sightings per run; 3–4/6 found | Needle test `R > 110` missed the darker needle on dials seen ≥ 30° off-axis | Relative redness (`R > 1.6·G, R > 1.6·B`). Dials now detected up to 30° off-axis |
| A real gauge scored as a false positive | Unreached candidates didn't advance the name counter, so two were both `found_gauge_4` | Name per candidate |
| Every leg after one collision failed instantly | The robot stayed in contact | Sim harness backs off (Nav2 BackUp equivalent) |

## Limits
- On a real site, a dial with no red needle, or one more than ~50° off-axis from every
  survey point, is not detected. A VLM detector pass on the survey frames would cover both.
- The offline reader reports a **fraction of full scale**: with no scale on file it can't
  know the units. The Claude and Cosmos backends are told to read the units and range off
  the dial face (`prompts.GAUGE_INSTRUCTION_UNKNOWN`). That path is not gated yet.
- On ROS, `range_along` uses the 2-D `/scan` (`pointcloud_to_laserscan`, 0.10–1.00 m above
  ground, 1° bins), so dials must sit in that band. The scan keeps the nearest return per
  bin, so something low in front of a dial gives a short range. The up-close verification
  then rejects the candidate rather than saving a wrong position.
- A survey takes ~20 min of sim time for a 15 × 9 m site, dominated by the 8 headings × 29 points.

## End-to-end demo: the whole agent, from the spoken request
```bash
MUJOCO_GL=osmesa python3 tools/agent/sim_mission.py --hide-gauges --seed 3 \
    --request "find the gauges and come back to where you started" --out <dir>
python3 tools/agent/plot_find_gauges.py <dir> --seed 3
```
Request → offline planner → plan `find_gauges → return_home` → executor → robot. The
mission SUCCEEDED: 6/6 gauges found (≤ 0.014 m position error), 6/6 read within 0.67 % of
full scale, all OK, 0 false positives, 29/29 survey points reached, 0 collisions, and it
ended back within 0.25 m of the start. Files are in `demo_seed3/`: `find_gauges_map.png`
(path, hidden truth vs found), `find_gauges_photos.png` (the up-close frames the readings
came from), `report.json`, and `taught.yaml` (the gauges it saved).

### How long it takes (seed 3, from `events.json`, robot time)
| Phase | Time |
|---|---|
| Survey: driving to 29 points | 4.1 min |
| Survey: looking around (8 in-place turns × 29 points) | 12.8 min |
| Going back to read the 6 gauges | 2.7 min |
| Return home | 0.6 min |
| **Total** | **20.2 min** |

Looking around takes most of the survey. Fewer headings (the camera covers 60°, so 6
are enough), or skipping headings where the LiDAR sees nothing within ~4 m, would roughly
halve it.

Video, recorded live from the physics:
```bash
MUJOCO_GL=osmesa python3 tools/agent/sim_mission.py --hide-gauges --seed 3 \
    --request "find the gauges and come back to where you started" --out <dir> --video <dir>/mission.mp4
```

## Driving quality: learned policy vs classical pursuit (2026-11-05)
The learned driver's video looked wild, and the numbers agree. Both runs below are the same
mission with 8 look headings swept once, skipping headings with no LiDAR surface within 4.5 m.
"Off route" is measured against the path planned at the start of each leg, not the 1 Hz replan
(a replan starts from wherever the robot already is, so it would hide wandering).

| Driver | Seed | Found / read | Collisions | > 0.3 m off route | Driven ÷ planned | At full turn rate | Time |
|---|---|---|---|---|---|---|---|
| **pursuit** (RPP law, 0.6 m look-ahead) | 3 | 6/6, 6/6 | 0 | 9 % | 1.17 | 26 % | 16.7 min |
| **pursuit** | 5 | 6/6, 6/6 | 0 | 10 % | 1.16 | 25 % | 17.6 min |
| learned v1 (1.5 m carrot) | 3 | 5/6, 5/6 | 2 | 33 % | 1.29 | 73 % | 13.1 min |
| learned v1 | 5 | 6/6, 6/6 | 0 | 33 % | 1.35 | 72 % | 14.4 min |

- The learned policy drives "bang-bang", at full turn rate about 3/4 of the time. With a
  1.5 m carrot it cuts every corner of the A* path, so it spends a third of its time more than
  0.3 m off the route. Smoothing its output wouldn't fix that: it wanders, it isn't only jerky.
- With a 1.5 m look-ahead, the pursuit law was as bad (44 % off route, 3 collisions). With
  0.6 m (Nav2 RPP's range) it stays on route. Look-ahead, not the learning, is most of the gap.
- 6 headings instead of 8 cut the survey to ~12.7 min but lost a gauge per run: a dial at the
  edge of the frame isn't detected. 8 headings is the default.
- **Decision:** `sim_mission.py` now drives with `--driver pursuit` by default. On the robot the
  default has always been classical Nav2. The learned driver stays experimental: it isn't used
  for missions until it is < 15 % of the time > 0.3 m off route and < 40 % saturated, with 0
  collisions, on this mission across ≥ 3 seeds.

## No more spinning: planned views (2026-11-05, current default)
Spinning 8 headings at 29 grid points (~230 turns) was mostly wasted: gauges are on
equipment, and the map already shows where every surface is. `gauge_finder.view_plan`
picks views that see each mapped surface head-on: 1.0–2.8 m away, within ±18° of the image
centre, ≤ 40° off the face, with clear line of sight. It's a greedy set cover, toured
nearest-first. The robot drives to each view, makes one short turn to face it, and takes ONE
photo. 42 views on this site; the plan takes 0.4 s.

| Seed | Found / read | Collisions | Time | > 0.3 m off route | At full turn rate |
|---|---|---|---|---|---|
| 3 | 6/6, 6/6 | 0 | 14.4 min | 2.3 % | 20 % |
| 5 | 6/6, 6/6 | 0 | 14.5 min | 2.0 % | 19 % |
| 7 | 6/6, 6/6 | 0 | 13.6 min | 1.8 % | 19 % |

That's down from 20.2 min (spin + learned driver) and 16.7 min (spin + classical driver).
Bugs found on the way:
- The two faces of a 0.1 m rack beam were merged as "duplicates", so the face with FL-401
  on it was never planned. Samples now merge only if they face the same way.
- Arriving at a fixed heading made the planner loop round for a straight approach (2× the
  path). Drive there first, then turn.
- The first plan accepted views with the dial 17° off-centre at 2.7 m (marginal for the
  detector). The coverage rule is tighter now.

## Other requests
| Request | Site | What happens |
|---|---|---|
| "find the kitchen and come back" | office | `goto kitchen → return_home`, 1.0 min, 0 collisions |
| "find the kitchen" | oil site | doesn't move: "I don't know where 'kitchen' is. Drive me there and say 'this is the kitchen' to teach me." |
| "read the oil pipe gauge and come back" | oil site | resolves to FL-401 (flow gauge on the pipe rack), reads 3.3 bar (true 3.3), 1.6 min |
| "what does the gauge on tank 1 say" | oil site | the gauge nearest tank 1 → PG-101 |
| "read the pressure gauge" | oil site | asks "Which gauge do you mean: gauge_pg101, gauge_pg102?" |
| "read the oil pipe gauge" | office (no gauges mapped) | "No gauges are on this map yet. Say 'find the gauges' and I'll search for them first." |

The kitchen run showed three more problems, now fixed:
1. The front stop was a ±35° cone, which saw both door jambs and stopped the robot in
   doorways. It's now a lane box (body width + 0.10 m), like Nav2's collision_monitor. At
   ≤ 0.10 m/s it may close to 6 cm; the footprint guard still vetoes contact.
2. Pure pursuit near a wall: w = v·k vanished as v was regulated down, so the robot crept
   into the jamb without turning. At low speed it now steers by the angle (RPP's rotate-to-heading idea).
3. office.xml's people never moved, and one stood in the kitchen doorway forever. The robot
   correctly stopped 6 cm short and gave up. People now step aside (`--people step-aside`,
   the default); `--people static` keeps the old blocked-doorway case.

## Faster: read during the survey (2026-11-06, current default)
Of the 14.4 min, 3.2 min went on driving BACK to each gauge for a close-up. But a planned
view's photo is already 1–2.8 m from the dial and nearly head-on. Changes:
- **Read on survey:** if the dial in a survey photo passes the same up-close test, it is
  read from a crop around just that dial (so a second dial in the frame can't be read
  instead). If the dial is within 3 m but off-centre, the robot turns to it on the spot
  (a few seconds) and reads it from there. It only drives back if neither works.
- **The reader's needle test** was still the absolute `R > 110` that the detector had
  already dropped. Survey photos see the needle off-axis and darker ("no needle found,
  0–5 red px"). It is relative now, like the detector.
- **Route:** 2-opt over the nearest-first tour (57 m vs 60 m; a property test checks it
  never lengthens or drops a view). The first version had an off-by-one that tripled
  the route to 199 m (22 min); caught by the timing table before it shipped.
- Gauges that still need a visit are visited nearest-first.

| Seed | Found / read | Time | Drives back for a close-up | Worst error |
|---|---|---|---|---|
| 3 | 6/6, 6/6 | 10.3 min | 0 | 0.67 % FS |
| 5 | 6/6, 6/6 | 10.7 min | 0 | 1.0 % FS |
| 7 | 6/6, 6/6 | 9.8 min | 0 | 1.0 % FS |

The progression on this site: 20.2 min (spin, learned driver) → 16.7 (spin, classical) →
14.4 (planned views) → **~10 min** (read on survey). 0 collisions, 0 false positives,
0 confident-wrong in every run.

Once the gauges have been found, the daily "read all the gauges and return" on the saved
landmarks takes **3.4 min** (G5 6/6, 0 confident-wrong, re-checked with the new reader).

## Camera always on + YOLO dial detector with asset tags (2026-11-06, current default)
**Always-on camera.** `search_and_read` registers a frame listener with the robot interface
(`io.set_frame_listener`: the sim calls it every 0.5 s of robot time while driving or
turning; `RosRobotIO` runs it on its own thread from `/camera/image_raw`). Every frame:
1. detect dials, place each with camera bearing + LiDAR range, and give a new one an
   **asset tag** (`DIAL-001`, …), saved as an alias of the found gauge;
2. read the dial if it is close and centred enough;
3. mark the mapped surfaces the frame saw, judged by the detector's measured envelope.
   A planned view whose surfaces were all seen on the way is **skipped**.

**YOLO detector.** A YOLO11n detector, trained only on sim-rendered frames:
`tools/perception/make_dial_dataset.py` places the robot camera at random free spots,
projects every gauge plate into the image for an exact box, and ray-checks occlusion.
Dial faces are re-randomized (value, range, units, label, brightness) so it learns "a dial",
not these six. Office frames are the negatives. 1600 frames; trained in 29 min on 4 CPU cores:
precision 0.96, recall 0.97, mAP50 0.993 (held-out 15 %). It runs as ONNX on onnxruntime,
the same runtime as the driving policy; no PyTorch on the robot.

`tools/perception/eval_dial_detector.py` (`detector_eval.json`), 1010 poses not used in training:

| | Detection rate | ms / frame | False alarms, 300 site / 300 office frames |
|---|---|---|---|
| classical | 55 % within 3 m head-on; 4 % at 3.5 m; 5 % at 50° off the face | 116 | 0 / 0 |
| **YOLO** | **100 % at every pose: 1–6 m, 0–65° off the face, up to 24° off-centre** | **16** | 6 / 0 (plate backs / edge-on plates: real gauge positions, never read or saved) |

Find-the-gauges, seeds 3 / 5 / 7:

| Camera | Detector | Time | Planned views skipped | Found / read | Collisions, false positives, confident-wrong |
|---|---|---|---|---|---|
| at stops only (before) | classical | 10.3 / 10.7 / 9.8 min | 0 / 42 | 6/6 | 0 |
| always on | classical | 9.2 / 9.8 / 8.6 min | 3–5 / 42 | 6/6 | 0 |
| **always on** | **YOLO** | **6.9 / 6.9 / 6.3 min** | **18–20 / 42** | **6/6** | **0** |

All 6 dials are spotted and read while the robot is on its way (0 drives back); worst
reading 1.0 % FS. Progression on this site: 20.2 → 16.7 → 14.4 → 10.3 → **6.6 min**.

**Limits, plainly:** the YOLO numbers are in the sim, on the same 6 plate geometries it was
trained around (new poses, randomized faces). On a real site it needs real labelled frames
from the robot's camera, fine-tuned from this model (`train_dial_yolo.py --init best.pt`).
Real cameras also blur while the robot moves: on hardware, read only when the robot is
slow, and use moving frames for spotting and tagging.
