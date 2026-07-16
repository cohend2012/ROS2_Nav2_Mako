#!/usr/bin/env python3
"""Sample true yaw vs dead-reckoned odom yaw vs SLAM-estimated yaw (map->base_link)
at 2 Hz for DURATION seconds. Prints CSV to stdout. Run inside the commander."""
import math, os, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

DUR = float(os.environ.get("DUR", "60"))

def qyaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

class P(Node):
    def __init__(self):
        super().__init__("yaw_probe")
        self.true_yaw = None; self.odom_yaw = None
        self.create_subscription(Odometry, "/odom_true",
            lambda m: setattr(self, "true_yaw", qyaw(m.pose.pose.orientation)), 10)
        self.create_subscription(Odometry, "/odom",
            lambda m: setattr(self, "odom_yaw", qyaw(m.pose.pose.orientation)), 10)
        self.buf = Buffer(); self.tfl = TransformListener(self.buf, self)
    def est_yaw(self):
        try:
            t = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
            return qyaw(t.transform.rotation)
        except Exception:
            return None

def main():
    rclpy.init(); n = P()
    t0 = time.time(); last = 0.0
    print("t,true_yaw_deg,odom_yaw_deg,est_yaw_deg")
    while rclpy.ok() and time.time() - t0 < DUR:
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() - last >= 0.5:
            last = time.time()
            e = n.est_yaw()
            row = [f"{time.time()-t0:.1f}"]
            for v in (n.true_yaw, n.odom_yaw, e):
                row.append("" if v is None else f"{math.degrees(v):.1f}")
            print(",".join(row), flush=True)
    n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
