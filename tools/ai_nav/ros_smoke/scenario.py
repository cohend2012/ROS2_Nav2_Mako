#!/usr/bin/env python3
"""Smoke scenario client: one NavigateToPose goal, then (optionally) one agent mission.

    python3 scenario.py nav  <gx> <gy> <timeout_s>
    python3 scenario.py agent "<request>" <timeout_s> <expected_gauge_value>

Exit 0 on PASS, 1 on FAIL. Prints one JSON line with the measured outcome.
"""
import json
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class Client(Node):
    def __init__(self):
        super().__init__("smoke_client")
        self.pose = None
        self.report = None
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(String, "/m20/agent/report", self._rep, 10)

    def _odom(self, m):
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y)

    def _rep(self, m):
        self.report = json.loads(m.data)


def spin_until(node, cond, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout and not cond():
        rclpy.spin_once(node, timeout_sec=0.1)
    return cond()


def nav(node, gx, gy, timeout):
    ac = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    if not ac.wait_for_server(timeout_sec=60.0):
        return {"pass": False, "why": "/navigate_to_pose server never appeared"}
    g = NavigateToPose.Goal(); g.pose.header.frame_id = "map"
    g.pose.pose.position.x, g.pose.pose.position.y = gx, gy
    g.pose.pose.orientation.w = 1.0
    f = ac.send_goal_async(g)
    spin_until(node, f.done, 10)
    gh = f.result()
    if gh is None or not gh.accepted:
        return {"pass": False, "why": "goal rejected"}
    rf = gh.get_result_async()
    t0 = time.time()
    spin_until(node, rf.done, timeout)
    if not rf.done():
        return {"pass": False, "why": f"no result in {timeout} s", "pose": node.pose}
    st = rf.result().status
    spin_until(node, lambda: False, 0.5)
    err = math.hypot(node.pose[0] - gx, node.pose[1] - gy) if node.pose else None
    return {"pass": st == 4 and err is not None and err < 0.35, "status": st,
            "final_err_m": round(err, 3) if err is not None else None,
            "duration_s": round(time.time() - t0, 1)}


def agent(node, request, timeout, expected):
    pub = node.create_publisher(String, "/m20/agent/command", 10)
    spin_until(node, lambda: node.pose is not None, 20)
    time.sleep(2.0)
    for _ in range(3):                         # discovery: publish until heard
        pub.publish(String(data=request)); spin_until(node, lambda: False, 1.0)
        if node.report is not None:
            break
    spin_until(node, lambda: node.report is not None, timeout)
    r = node.report
    if r is None:
        return {"pass": False, "why": f"no agent report in {timeout} s"}
    rd = [x for x in r.get("readings", []) if "gauge" in x]
    val = rd[0]["value"] if rd else None
    home = math.hypot(node.pose[0] - r["home"][0], node.pose[1] - r["home"][1]) if r.get("home") else None
    ok = r["success"] and val is not None and abs(val - expected) <= 5.0 and home is not None and home < 0.4
    return {"pass": bool(ok), "success": r["success"], "detail": r["detail"],
            "plan": [s["action"] + ":" + s.get("target", "") for s in r.get("plan", [])],
            "reading": val, "expected": expected,
            "home_err_m": round(home, 3) if home is not None else None}


def main():
    rclpy.init()
    n = Client()
    if sys.argv[1] == "nav":
        out = nav(n, float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
    else:
        out = agent(n, sys.argv[2], float(sys.argv[3]), float(sys.argv[4]))
    print(json.dumps(out))
    rclpy.shutdown()
    sys.exit(0 if out["pass"] else 1)


if __name__ == "__main__":
    main()
