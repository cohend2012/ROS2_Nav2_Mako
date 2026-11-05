#!/usr/bin/env python3
"""Office BATCH gate: one stack bringup, N door-to-door legs, scored per leg.

    python3 office_batch.py <landmarks.yaml> <out.json> [legs=10] [leg_timeout_s=240]

The tour visits every room's approach pose in turn (each leg crosses >= 1 doorway,
most cross 2: room -> corridor -> room), starting from reception. A leg PASSES when
/navigate_to_pose returns SUCCEEDED and the robot is within 0.35 m of the goal.
Goals Nav2 rejects while its lifecycle activates are retried for 60 s (startup only).
Exit 0 iff >= 9/10 of legs pass (the bar); the JSON has every leg either way.
"""
import json
import math
import sys
import time

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry


class Batch(Node):
    def __init__(self):
        super().__init__("office_batch")
        self.pose = None
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.ac = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _odom(self, m):
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y)

    def spin_for(self, cond, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout and not cond():
            rclpy.spin_once(self, timeout_sec=0.1)
        return cond()

    def leg(self, x, y, yaw, timeout, first):
        g = NavigateToPose.Goal(); g.pose.header.frame_id = "map"
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        t_end = time.time() + (60.0 if first else 5.0)
        while True:
            f = self.ac.send_goal_async(g)
            self.spin_for(f.done, 10)
            gh = f.result()
            if gh is not None and gh.accepted:
                break
            if time.time() > t_end:
                return {"status": "REJECTED", "time_s": 0.0}
            self.spin_for(lambda: False, 2.0)
        t0 = time.time()
        rf = gh.get_result_async()
        self.spin_for(rf.done, timeout)
        if not rf.done():
            gh.cancel_goal_async(); self.spin_for(lambda: False, 2.0)
            st = "TIMEOUT"
        else:
            st = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}.get(rf.result().status, str(rf.result().status))
        self.spin_for(lambda: False, 0.5)
        err = math.hypot(self.pose[0] - x, self.pose[1] - y) if self.pose else None
        return {"status": st, "time_s": round(time.time() - t0, 1),
                "final_err_m": round(err, 3) if err is not None else None}


def main():
    lm_path, out = sys.argv[1], sys.argv[2]
    n_legs = int(sys.argv[3]) if len(sys.argv) > 3 else 10
    timeout = float(sys.argv[4]) if len(sys.argv) > 4 else 240.0
    lms = {l["name"]: l for l in yaml.safe_load(open(lm_path))["landmarks"]}
    rooms = [n for n, l in lms.items() if l["type"] == "room"]
    tour = (rooms + ["reception"]) * 4
    rclpy.init()
    b = Batch()
    if not b.ac.wait_for_server(timeout_sec=90.0):
        print(json.dumps({"pass": False, "why": "no /navigate_to_pose"})); sys.exit(1)
    b.spin_for(lambda: b.pose is not None, 30)
    legs, prev = [], "reception"
    for i, name in enumerate(tour[:n_legs]):
        x, y, yaw = lms[name]["approach"]
        r = b.leg(x, y, yaw, timeout, first=(i == 0))
        r.update(leg=f"{prev}->{name}", ok=bool(r["status"] == "SUCCEEDED" and r.get("final_err_m") is not None
                                              and r["final_err_m"] < 0.35))
        legs.append(r); prev = name
        print(json.dumps(r), flush=True)
    passed = sum(l["ok"] for l in legs)
    ok_t = [l["time_s"] for l in legs if l["ok"]]
    res = {"passed": passed, "of": len(legs), "pass": passed >= math.ceil(0.9 * len(legs)),
           "mean_leg_time_s": round(sum(ok_t) / len(ok_t), 1) if ok_t else None, "legs": legs}
    json.dump(res, open(out, "w"), indent=2)
    print(json.dumps({k: res[k] for k in ("passed", "of", "pass", "mean_leg_time_s")}))
    rclpy.shutdown()
    sys.exit(0 if res["pass"] else 1)


if __name__ == "__main__":
    main()
