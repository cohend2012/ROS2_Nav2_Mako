# Engineering rules — written after the user said "write much better software"

He was right. 2026-07/08 defect review: most bugs were preventable authoring
mistakes, not robotics. Four classes, four rules. These bind every future
script and node in this repo.

## 1. No silent failure paths — ever
- `set -euo pipefail` in every bash script. No `| tail`/`| grep` on a command
  whose exit code matters (pipe exit = last command; it masked three failures).
- No `2>/dev/null` on anything that can fail meaningfully. Capture to a log
  file instead. `docker exec -d` output ALWAYS goes to a file (July lesson,
  relearned in August — never again).
- A check that can't find its input FAILS with a message; it never computes
  on emptiness (the md5-of-empty-stdin staleness bug).

## 2. Verify the contract before writing the consumer
- Touching a message type? `ros2 interface show` FIRST (drdds nesting bug).
- Subscribing to a topic? Check the publisher's QoS FIRST (`ros2 topic info
  -v`) (latched /m20/mode bug).
- Producing sensor data? Look at ONE actual sample before declaring it works
  (backward camera: verified by Hz for weeks, wrong by 180° the whole time).

## 3. Smoke-test every script on tiny input before embedding it in a chain
- New script → 30-second run on the smallest real input, output eyeballed,
  THEN wired into batches/gates. (Patrol gate shipped with its own bringup
  failure invisible; renderer went O(n²) on first real-size input.)
- New flag on an old script → run the old path once too (regression).

## 4. Two identical environmental failures = redesign, not retry #3
- Outputs of long jobs go to PERSISTENT storage from the start, never /tmp.
- Long pipelines are chunked and resumable; each stage checks its inputs
  exist before running.
- If the box keeps dying, change the plan's shape (smaller stages, lower
  memory), don't relaunch the same shape harder.

## Standing debt this file supersedes silently accumulating again
Existing scripts get retrofitted to rules 1 and 3 as they're next touched —
not in one heroic pass. Every retrofit is named in its commit message.
