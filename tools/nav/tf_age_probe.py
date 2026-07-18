#!/usr/bin/env python3
"""Measure map->odom TF AGE once per second for 60 s, while a goal runs.
Prints: t, tf_age (now - latest map->odom stamp), robot true x, scan count seen."""
import time
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class P(Node):
    def __init__(self):
        super().__init__("tf_age_probe")
        self.buf = Buffer(); self.tfl = TransformListener(self.buf, self)
        self.scans = 0; self.x = float("nan")
        self.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "scans", self.scans + 1), 10)
        self.create_subscription(Odometry, "/odom_true", lambda m: setattr(self, "x", m.pose.pose.position.x), 10)

def main():
    rclpy.init(); n = P()
    t0 = time.time(); last = 0.0
    while time.time() - t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() - last >= 1.0:
            last = time.time()
            try:
                tr = n.buf.lookup_transform("map", "odom", rclpy.time.Time())  # latest available
                stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                age = time.time() - stamp
                print(f"t={time.time()-t0:4.1f}s tf_age={age:+7.2f}s scans={n.scans} x={n.x:.2f}", flush=True)
            except Exception as e:
                print(f"t={time.time()-t0:4.1f}s NO-TF ({type(e).__name__}) scans={n.scans} x={n.x:.2f}", flush=True)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
