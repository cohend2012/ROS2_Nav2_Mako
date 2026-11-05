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
