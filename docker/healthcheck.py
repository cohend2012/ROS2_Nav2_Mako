#!/usr/bin/env python3
"""Container healthcheck: healthy means the node named in $HEALTH_NODE has
published m20_msgs/HealthReport on /m20/health within $HEALTH_MAX_AGE seconds
with status OK or WARN. Liveness = actually publishing, not merely running.
Exit 0 healthy, 1 unhealthy (Docker restarts per compose policy)."""
import os, sys, time
import rclpy
from m20_msgs.msg import HealthReport

NODE = os.environ.get("HEALTH_NODE", "")
MAX_AGE = float(os.environ.get("HEALTH_MAX_AGE", "5.0"))

def main():
    rclpy.init()
    node = rclpy.create_node("healthcheck_probe")
    seen = {"ok": False}

    def cb(msg):
        if msg.node_name == NODE and msg.status <= HealthReport.STATUS_WARN:
            seen["ok"] = True

    node.create_subscription(HealthReport, "/m20/health", cb, 10)
    deadline = time.time() + MAX_AGE
    while time.time() < deadline and not seen["ok"]:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()
    sys.exit(0 if seen["ok"] else 1)

if __name__ == "__main__":
    main()
