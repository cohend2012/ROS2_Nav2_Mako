#!/usr/bin/env python3
"""Verify the dead-reckoned wheel /odom tracks motion but DRIFTS vs /odom_true."""
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

class Chk(Node):
    def __init__(self):
        super().__init__("odom_drift_check")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom = None; self.true = None
        self.create_subscription(Odometry, "/odom", lambda m: setattr(self, "odom", m), 10)
        self.create_subscription(Odometry, "/odom_true", lambda m: setattr(self, "true", m), 10)
    def drive(self, vx, wz, secs):
        t = time.time()
        while time.time()-t < secs and rclpy.ok():
            m = Twist(); m.linear.x=float(vx); m.angular.z=float(wz); self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.02); time.sleep(0.05)
    def report(self, tag):
        o, t = self.odom, self.true
        ox, oy, oth = o.pose.pose.position.x, o.pose.pose.position.y, yaw(o.pose.pose.orientation)
        tx, ty, tth = t.pose.pose.position.x, t.pose.pose.position.y, yaw(t.pose.pose.orientation)
        d = math.hypot(ox-tx, oy-ty)
        print(f"{tag:12s} odom=({ox:+.2f},{oy:+.2f},{math.degrees(oth):+.0f}d)  "
              f"true=({tx:+.2f},{ty:+.2f},{math.degrees(tth):+.0f}d)  pos_err={d:.2f}m")

def main():
    rclpy.init(); n = Chk()
    while (n.odom is None or n.true is None) and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
    n.report("start")
    n.drive(0.3, 0.0, 4.0); n.drive(0,0,0.5); n.report("after fwd")
    n.drive(0.0, 0.5, 6.0); n.drive(0,0,0.5); n.report("after turns")
    n.drive(0.3, 0.3, 6.0); n.drive(0,0,0.5); n.report("after arc")
    n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
