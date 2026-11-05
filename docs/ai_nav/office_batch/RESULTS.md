# Office batch gate: 10 door-to-door legs on real ROS 2 Humble + Nav2

Kinematic office world (box-exact LiDAR), office overlay (real 0.91 x 0.53 m footprint).
The tour runs reception → kitchen → meeting room → open office → server room →
manager office → reception, twice. Doors are 0.85-1.15 m. Two places are TIGHT: the
meeting room (0.50 m clearance) and the open office (0.54 m), less than the robot's
0.53 m corner sweep, so it cannot pivot there. **Bar: ≥ 9/10 legs.** Run with
`BATCH=1 SITE=office MODES=nav2 VARIANT=<v> [CM=1] bash tools/ai_nav/ros_smoke.sh`.

| configuration | legs passed | mean passing leg | failures |
|---|---|---|---|
| NavFn + DWB (`dwb`, office default until now) | **6/10** | 62 s | meeting room ↔ open office ×2 each (timeouts) |
| **Smac Lattice (diff, reverse) + RPP + collision monitor (`lattice_cm`, CM=1)** | **7/10** | 150 s | kitchen → meeting room ×2 (stuck at the 0.85 m door off the corridor end), open office → server room ×1 |
| `rpp_cm` (NavFn + RPP + collision monitor) | not finished (stopped when the places were restored) | | |

AI driver (MuJoCo, vendor physics, v3 policy, standalone A* with approach vector + shield),
"go to <tight place> and come back", 5 seeds each: **0/10 missions**. Deterministic run:
meeting room reached and parked (0.04 m). Randomized runs: collisions at the open office
door (−5.2, 0.9) ×3, stuck en route ×3, timeouts.

## Diagnosis (lattice_cm logs)
- 33× "Failed to make progress", 8× "Starting point in lethal space": after squeezing
  past a door jamb, the robot's centre sits inside the inflated lethal band, so the
  planner refuses to plan from there and the path comes back empty.
- The collision monitor is not the blocker: 8 stops and 36 slowdowns in the whole run.

## Next (in order)
1. Recovery for "start in lethal": BT back-up / clear-costmap before replanning, and
   start-pose tolerance on the planner.
2. Inflation that matches the footprint (inscribed 0.27 m): try 0.30 with steeper
   scaling, so doors stay plannable and the start is never "lethal" after a near pass.
3. Re-run `lattice_cm` + `rpp_cm` batches. A configuration ships when it reaches ≥ 9/10
   on two batches.
