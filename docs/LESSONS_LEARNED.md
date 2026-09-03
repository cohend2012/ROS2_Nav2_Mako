# Lessons Learned

Every entry here cost real time. Each is written as **symptom → root cause →
fix → the rule**, because the symptom is what you will recognize first and the
rule is what you should carry to the next project.

---

## 1. ROS 2 mechanics

### 1.1 QoS mismatch is silent
**Symptom:** A subscriber never receives anything. No error, no warning. The
topic exists, `ros2 topic hz` from a *different* terminal shows data.
**Cause:** Sensor publishers use `SensorDataQoS` (BEST_EFFORT). A RELIABLE
subscriber is simply never matched. DDS does not consider this an error.
**Fix:** Match QoS explicitly. For CLI probes pass
`--qos-reliability best_effort`.
**Rule:** When a topic looks dead, suspect QoS before suspecting the producer.
`ros2 topic info -v <topic>` shows both sides.

### 1.2 Latched (`transient_local`) topics need a matching subscriber
**Symptom:** The behavior engine denied a request with "mode not armed" while
the commander was demonstrably armed.
**Cause:** `/m20/mode` is published latched and only on change. A VOLATILE
subscriber that starts *after* the last change never learns the mode.
**Fix:** Subscribe with `TRANSIENT_LOCAL` durability for latched state topics.
**Rule:** State topics are latched; event topics are volatile. Match the
publisher's intent, and remember that late-joining subscribers are the normal
case in a stack that starts nodes at different times.

### 1.3 Cross-process latched delivery is not dependable
**Symptom:** After a host suspend/resume, `/tf_static` stopped reaching other
containers while continuously-published topics flowed fine — silently killing
the entire perception chain.
**Fix:** Publish static transforms in the *consumer's* process/container.
**Rule:** Never depend on cross-boundary latched delivery for anything
critical. Publish it where it is consumed.

### 1.4 Read the interface definition before writing the consumer
**Symptom:** Logger crashed on the first message:
`'JointsData' object has no attribute 'joints_data'`.
**Cause:** The field was nested one level deeper (`m.data.joints_data`). I
wrote it from memory.
**Fix:** `ros2 interface show <pkg>/msg/<Type>` takes ten seconds.
**Rule:** Never write against a message type you have not printed.

### 1.5 Lifecycle nodes do nothing until activated
**Symptom:** Nav2 processes running, goals accepted, nothing happens.
**Cause:** A node stuck in `inactive`.
**Rule:** "Process is running" ≠ "node is working." Check lifecycle state.

### 1.6 `PYTHONPATH` prepend, never overwrite
**Symptom:** `ModuleNotFoundError: No module named 'rclpy'` when launching a
node from a staged directory.
**Cause:** `PYTHONPATH=/cfg` clobbered ROS's own path.
**Fix:** `PYTHONPATH=/cfg:$PYTHONPATH`.
**Rule:** Environment variables that are lists get appended to, not replaced.

---

## 2. Localization and mapping

### 2.1 A map is an artifact with a QA gate, not a by-product
**Symptom:** Localization looked excellent, yet the robot consistently stopped
0.5–0.6 m from goals.
**Cause:** The mapping run's odometry drift was **frozen into the pose graph** —
up to 0.65 m of region-dependent warp. The robot was accurately navigating a
subtly wrong map.
**Fix:** Bound drift during mapping (see 2.2), then *measure the map*: fit
known landmarks (tank centres) against true geometry and refuse any map whose
landmarks are off by more than tolerance (`tools/map/check_map_landmarks.py`).
**Rule:** If you cannot state your map's error in metres, you do not know
whether your localization problem is a map problem.

### 2.2 Anchoring the mapping prior must be *smooth*
**Symptom:** Feeding GPS-corrected pose as the odometry prior made maps
**worse** (0.27 m → 0.95 m landmark error).
**Cause:** 5 Hz GPS jitter entered every scan registration. Scan matchers need
a locally-consistent prior far more than an absolute one.
**Fix:** A complementary filter — integrate wheel+IMU smoothly, let the GPS
correction leak in rate-limited (0.05 m/s). Faster (0.08) destabilized mapping
entirely.
**Rule:** Absolute references belong in slow, smooth corrections. Never as
jumps into anything a matcher consumes.

### 2.3 GPS cannot observe heading — and a rotated map looks like a warped one
**Symptom:** Landmark errors grew linearly with distance from origin (0.20 /
0.21 / 1.01 m) — the signature of a *rotated* map, not a translated one.
**Cause:** Gyro bias slowly rotated the odometry frame; GPS position updates
cannot correct yaw.
**Fix:** A course-over-ground heading aid — when the robot moves forward more
than a threshold, the direction of GPS displacement is a yaw measurement.
**Rule:** Position fixes do not bound heading drift. If your errors scale with
distance, you have an angular problem.

### 2.4 Residual error is random per build — so build several
**Symptom:** Identical procedures produced maps ranging 0.03–0.95 m landmark
error.
**Fix:** Build N maps, score all with the same gate, ship the best; commit the
winning score alongside the map.
**Rule:** For a one-time artifact, selection beats tuning. But score every
candidate identically — otherwise you are gate-shopping, not selecting.

### 2.5 Seed the localizer explicitly
**Symptom:** Intermittently, a mission's first leg never moved at all.
**Cause:** `map_start_at_dock: true` is **silently unsupported** in
slam_toolbox localization mode — the log says "correctly not supported." We
ran unseeded for weeks; sometimes it self-converged from identity, sometimes
it never localized.
**Fix:** Publish `/initialpose` explicitly after the node has a subscriber.
**Rule:** Read your dependency's startup log. Unsupported options do not
always fail loudly.

### 2.6 Long missions expose mis-lock that single goals never will
**Symptom:** 10/10 success on one goal for two weeks; the first five-location
patrol scored 4/5 with 2 m errors.
**Cause:** In feature-sparse areas the scan matcher lost its anchor, jumped
~2 m, and locked confidently wrong. It never recovered. The robot then fought
phantom geometry for 203 seconds, which *looks* exactly like being stuck
behind an obstacle.
**Fix:** A supervisory node watching the residual between the localizer and a
smoothed absolute reference, re-seeding `/initialpose` on a sustained
excursion (`tools/nav/anchor_guardian.py`).
**Rule:** Reliability is measured over **missions**, not goals. Any test that
ends before the failure matures is not a test of that failure.

### 2.7 Detector thresholds come from measured distributions
**Symptom:** First guardian version never fired. Second fired eleven times in
one mission and scored 0/5.
**Cause:** A guessed 1.0 m threshold sat *inside* the healthy-driving noise
band (p95 ≈ 0.9 m, max ≈ 1.2 m).
**Fix:** Measure the residual distribution in both healthy and failed runs;
place the threshold in the gap (arm 1.5, disarm 1.0). Add hysteresis — a
single threshold flickers across the line and never accumulates hold time.
**Rule:** Any detector needs two numbers you have actually measured: what
normal looks like, and what the failure looks like.

### 2.8 A frame correction legitimately aborts the goal it interrupts
**Symptom:** Legs aborted seconds after a successful re-anchor.
**Cause:** Re-seeding jumps the `map` frame; Nav2's in-flight goal is now
inconsistent and it aborts. Correct behaviour.
**Fix:** The mission layer retries the leg on the corrected frame.
**Rule:** Corrections have consequences one layer up. Design the retry when
you design the correction.

---

## 3. Nav2 tuning

### 3.1 Controller choice is empirical
Regulated Pure Pursuit rotated toward a heading ~165° off-goal and dithered
forever on this platform, through six layers of investigation. Swapping to DWB
— a one-line parameter change — produced a successful goal immediately.
**Rule:** Before deep-diving a controller's internals, try the other one. It
is a one-line experiment.

### 3.2 Goal tolerance is a silent accuracy ceiling
Ours sat at 0.45 m — a value inherited from a noisier-localization era — long
after localization improved. It was the single largest term in our error
metric. Note it must be set in **two** places (goal checker and controller).
**Rule:** Revisit tolerances whenever the thing they were sized against
changes.

### 3.3 Inflation had to *shrink* when the map got better
An accurate map rendered a corridor at its true narrow width; the existing
inflation closed it and DWB reported "No valid trajectories out of 251!"
until patience expired.
**Rule:** Inflation is a *preference* for clearance. Collision safety comes
from `robot_radius`/footprint. Improving one input can invalidate a parameter
tuned against the older, blurrier input.

### 3.4 Progress checkers measure translation
The default `movement_time_allowance` aborted goals during the robot's slow
rotate-to-align phase, because pure rotation is zero position progress.
**Rule:** Know what your watchdog actually measures.

### 3.5 Acceleration limits must beat drivetrain stiction — everywhere
A controller that ramps from the *measured* rate can never escape a dead zone
wider than `accel × dt`. The limit must agree across the controller, behavior
server, and velocity smoother, or the most restrictive silently wins.
**Rule:** When the same physical quantity appears in several configs, they are
one setting with several copies. Change them together.

### 3.6 Capture Nav2's stdout
Two days went into "why does it abort?" while bringup discarded the logs with
`docker exec -d`. The answer was in the controller warnings the entire time.
**Rule:** Any node launched detached must redirect to a file. No exceptions.

---

## 4. Sensors

### 4.1 Verify sensors by what they SEE, not that they publish
Our simulated camera faced **backwards for weeks**. `ros2 topic hz` was green
the whole time. It was caught only when a behavior that needed to *find*
something failed against ground truth.
**Rule:** A rate check proves plumbing. Only content proves correctness. Look
at one frame. Count the pixels of the thing you expect.

### 4.2 A detector will happily lock onto your own robot
`camera_scan` reported SUCCESS at 177° from the target: the chase camera kept
the robot in frame, and the vendor model contains one pure-red geometry —
which the red-target detector found first.
**Rule:** Check what is in the sensor's field of view besides the world.
Egocentric sensors should not see the robot.

### 4.3 Ground truth is for scoring, never for consuming
The stack under test only ever consumes honest signals (drifting odometry,
noisy GPS, live scans). `/odom_true` has exactly one subscriber: the logger.
**Rule:** If any control-path node subscribes to ground truth, your results
are fiction.

---

## 5. Testing and verification

### 5.1 Score against truth, not against the robot's estimate
A mis-located robot reports SUCCESS while standing metres away. Nav2 is not
lying — it reached the goal *in its own frame*.
**Rule:** Success is a position measured externally, not a status enum.

### 5.2 Three observers must agree
We once had Nav2 report SUCCEEDED with the robot parked (6 m error) and a
logger recording a motionless robot during runs that demonstrably travelled.
**Fix:** `observer_trust.sh` — PASS only if the action result, the in-run
logger, and a live probe all agree.
**Rule:** One observer is an opinion. Instrumentation needs its own test.

### 5.3 Log incrementally, not at the end
The "blind logger" wrote CSVs only when its window closed; every mid-run copy
therefore read the *previous* run's data. This produced a fake crisis about
the estimator that took a full session to unwind.
**Rule:** Any long-running logger dumps incrementally.

### 5.4 Contamination detection belongs in the harness
Host sleep produced 5,000-second "runs." Machine load starved the control loop
and produced navigation failures with no navigation cause. Both now trip
explicit warnings.
**Rule:** Your test harness should know when its own results are worthless.

### 5.5 Archive per-run logs
The one anomalous run in ten is the interesting one, and it is unforensicable
after teardown.
**Rule:** Batches archive each run's logs before cleaning up.

### 5.6 A failing gate is information, not an obstacle
Every gate failure in this project produced a genuine fix: the warped map, the
mis-lock, the unseeded localizer, the stale artifact. Failing gates were the
most productive events in the schedule.
**Rule:** Do not tune the gate to pass. Fix the thing the gate found.

---

## 6. Infrastructure

### 6.1 The artifact must contain the code you tested
The tip-over failsafe existed in git for two weeks while the running container
carried a pre-failsafe build. An inverted robot stayed armed. Nothing but a
physical test could have caught it.
**Fix:** Bringup now hashes the installed commander against the repo and
refuses to run stale safety code.
**Rule:** Verify installed-vs-source at startup. "It is committed" is not "it
is running."

### 6.2 WSL will delete your environment out from under you
The VM idles out after the last session closes, killing every container. Host
sleep restarts containers, removes `--rm` ones, and skews clocks.
**Rules:** One experiment = one self-contained session. Never assume a
container from a previous command still exists. Disable host sleep before long
tests.

### 6.3 Long-job outputs go to persistent storage, immediately
Three render cycles were lost writing to `/tmp` across VM restarts.
**Rule:** If a job takes more than a few minutes, its outputs live on durable
storage from the first byte.

### 6.4 Two identical environmental failures means redesign, not retry
Repeated crashes under memory pressure are a signal to chunk the work, not to
run it again harder.
**Rule:** Change the *shape* of the plan, not the number of attempts.

### 6.5 Line endings will break your scripts
A merge rewrote shell scripts as CRLF; `set -e` became `set -e\r` and every
path grew a trailing carriage return. Bash errors were baffling.
**Fix:** `.gitattributes` pinning `*.sh text eol=lf`.
**Rule:** On a Windows/WSL split, pin line endings before they bite.

---

## 7. Engineering practice

These four rules are enforced repo-wide (`docs/ENGINEERING_RULES.md`), each
written after a defect review found the same class recurring:

1. **No silent failure paths.** `set -euo pipefail`. No `| tail` on a command
   whose exit code matters — a pipe returns the *last* command's status, and
   this masked three separate failures. No `2>/dev/null` on anything that can
   fail meaningfully. A check that cannot find its input fails loudly rather
   than computing on emptiness (`md5sum` with no argument hashes empty stdin
   and returns a valid-looking hash).
2. **Verify the contract before writing the consumer.** Message definition,
   publisher QoS, and one actual sample — all before the first line of the
   consumer.
3. **Smoke-test on tiny input before embedding in a chain.** A new script goes
   into a 20-minute pipeline only after a 30-second run on a fixture. One
   renderer was accidentally O(n²) and only revealed it on a 67,000-row log,
   inside a chain, at the end.
4. **Two identical environmental failures = redesign.** See 6.4.

### 7.1 Architecture discipline paid off repeatedly
- **Contracts first** (`m20_msgs` before nodes) meant every component swap was
  a drop-in.
- **One implementation per function** — a single bringup script, a single
  teardown — after a forked copy diverged and produced "never moved" runs
  while manual bringup was green. When the batch's teardown pattern drifted
  from the canonical one, a stray node survived an entire ten-run batch.
- **The vendor boundary** (one bridge node owns the SDK) means hardware
  migration is a backend swap, not a rewrite.
- **Flag-guarded features** meant no experiment ever destabilized the known-
  good path; every new capability shipped behind an env flag defaulting off.

### 7.2 Write down why, at the point of the decision
Nearly every parameter in this repo carries a comment naming the incident that
set it. When a later session is tempted to "clean up" a strange value, the
comment stops it. Two of those comments record settings that were tried,
measured as worse, and reverted — which is exactly the knowledge that
otherwise gets rediscovered the hard way.

---

## 8. The five most expensive lessons, ranked

1. **The map is a tested artifact.** An unvalidated map makes every downstream
   measurement meaningless, and the symptom points everywhere except the map.
2. **Reliability is measured over missions.** Two weeks of green single-goal
   gates hid a systematic failure that appeared within 150 seconds of the
   first real tour.
3. **Verify sensors by content.** A backwards camera passed every plumbing
   check for weeks.
4. **The artifact must contain the tested code.** Safety code in git is not
   safety code on the robot.
5. **Detector thresholds come from measured distributions.** A guessed
   threshold turned a working fix into a worse failure than the bug.
