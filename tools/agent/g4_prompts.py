#!/usr/bin/env python3
"""Gate G4: language -> valid grounded mission, on 10 operator prompts.

    python3 tools/agent/g4_prompts.py --backend offline
    ANTHROPIC_API_KEY=... python3 tools/agent/g4_prompts.py --backend claude
    M20_COSMOS_BASE_URL=http://gpu:8000/v1 M20_COSMOS_MODEL=nvidia/Cosmos-Reason2-8B \
        python3 tools/agent/g4_prompts.py --backend cosmos

Each prompt has a CHECK on the validated plan: what must be visited, what must
not be, and whether it must end at home. The plan must pass plan.validate() (grounded
in the registry); then the check runs. No robot is needed: this gates the
planner, which is the only part of the agent a model controls. Execution of
the plan is gated by tools/agent/sim_mission.py (MuJoCo) and tools/ai_nav/ros_smoke.sh.
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "src", "m20_agent"))

from m20_agent.executor import make_backend  # noqa: E402
from m20_agent.landmarks import LandmarkRegistry  # noqa: E402
from m20_agent.plan import validate, PlanError  # noqa: E402

GAUGES = {"gauge_pg101", "gauge_pg102", "gauge_ti201", "gauge_pg301", "gauge_pg302", "gauge_fl401"}

# (prompt, must_read (set of gauges read), must_visit (set, any action), home_last, forbid_motion)
CASES = [
    ("Read all the gauges and return to where you started.", GAUGES, set(), True, False),
    ("Run the mission: check every pressure and temperature gauge on site, then come back.", GAUGES, set(), True, False),
    ("Go to tank 1.", set(), {"tank1"}, False, False),
    ("Read PG-301 and PG-302.", {"gauge_pg301", "gauge_pg302"}, set(), False, False),
    ("What does the wellhead temperature gauge say?", {"gauge_ti201"}, set(), False, False),
    ("Drive to the pipe rack, then the east field, then come back home.", set(), {"pipe_rack", "east_field"}, True, False),
    ("Inspect skid 1 for leaks and return to start.", set(), {"skid1"}, True, False),
    ("Read the tank 2 gauge, wait 10 seconds, read it again, then go home.", {"gauge_pg102"}, set(), True, False),
    ("Patrol the west field, north lane and east field.", set(), {"west_field", "north_lane", "east_field"}, False, False),
    ("Find the kitchen.", set(), set(), False, True),   # not on this site: must not move
    ("Search the site for gauges you don't know about yet, read them, and come back.",
     set(), set(), True, False, True),                  # find_gauges (no prior positions)
]


def check(steps, must_read, must_visit, home_last, forbid_motion, must_find=False):
    read = {s["target"] for s in steps if s["action"] == "read_gauge"}
    visited = {s.get("target") for s in steps if s["action"] in ("goto", "inspect", "read_gauge")}
    moves = [s for s in steps if s["action"] not in ("wait",)]
    errs = []
    if must_find and not any(s["action"] == "find_gauges" for s in steps):
        errs.append("should search for gauges (find_gauges)")
    if forbid_motion and moves:
        errs.append(f"should not move, plan has {[s['action'] for s in moves]}")
    if not must_read <= read:
        errs.append(f"missing reads {sorted(must_read - read)}")
    if not must_visit <= visited:
        errs.append(f"missing visits {sorted(must_visit - visited)}")
    if home_last and steps[-1]["action"] != "return_home":
        errs.append("does not end with return_home")
    if not home_last and any(s["action"] == "return_home" for s in steps):
        errs.append("returns home though not asked")
    return errs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", default="offline")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    reg = LandmarkRegistry.load(os.path.join(REPO, "src", "m20_agent", "config",
                                             "oil_gas_field_landmarks.yaml"))
    reg.set_home(0.0, 0.0, 0.0)
    be = make_backend(a.backend, reg, offline_fallback=False)
    rows, passed = [], 0
    for prompt, *crit in CASES:
        t0 = time.time()
        try:
            raw = be.plan(prompt, reg.describe())
            steps = validate(raw, reg)
            errs = check(steps, *crit)
        except (PlanError, RuntimeError) as e:
            steps, errs, raw = [], [f"{type(e).__name__}: {e}"], {}
        ok = not errs
        passed += ok
        rows.append({"prompt": prompt, "pass": ok, "errors": errs,
                     "plan": [f"{s['action']}:{s.get('target', s.get('seconds', ''))}" for s in steps],
                     "rationale": raw.get("rationale", ""), "latency_s": round(time.time() - t0, 2)})
        print(f"{'PASS' if ok else 'FAIL'}  {prompt}\n      -> {rows[-1]['plan']}"
              + (f"\n      !! {errs}" if errs else ""))
    res = {"backend": be.name, "passed": passed, "of": len(CASES), "pass": passed == len(CASES),
           "date": time.strftime("%Y-%m-%d"), "rows": rows}
    print(f"G4 {be.name}: {passed}/{len(CASES)} {'PASS' if res['pass'] else 'FAIL'}")
    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        json.dump(res, open(a.out, "w"), indent=2)
    sys.exit(0 if res["pass"] else 1)


if __name__ == "__main__":
    main()
