#!/usr/bin/env python3
"""Quick sanity: does /cmd_vel drive the sim robot with correct signs?
Reads ground-truth /odom, commands forward then in-place turn, prints deltas."""
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Chk(Node):
    def __init__(self):
        super().__init__("drive_check")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pose = None
        self.create_subscription(Odometry, "/odom", self.cb, 10)
    def cb(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)
    def wait_pose(self):
        while self.pose is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose
    def drive(self, vx, wz, secs):
        t = time.time()
        while time.time()-t < secs and rclpy.ok():
            m = Twist(); m.linear.x = float(vx); m.angular.z = float(wz)
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.05)
    def stop(self, secs=1.0):
        self.drive(0.0, 0.0, secs)

def main():
    rclpy.init(); n = Chk()
    p0 = n.wait_pose()
    print(f"START      x={p0[0]:+.2f} y={p0[1]:+.2f} yaw={math.degrees(p0[2]):+.0f}deg")
    n.drive(0.3, 0.0, 3.0); n.stop()
    p1 = n.pose
    print(f"after FWD  x={p1[0]:+.2f} y={p1[1]:+.2f} yaw={math.degrees(p1[2]):+.0f}deg  "
          f"(dx={p1[0]-p0[0]:+.2f} dy={p1[1]-p0[1]:+.2f}) -> expect +x")
    n.drive(0.0, 0.5, 3.0); n.stop()
    p2 = n.pose
    dyaw = math.degrees(p2[2]-p1[2])
    print(f"after TURN yaw={math.degrees(p2[2]):+.0f}deg  (dyaw={dyaw:+.0f}deg) -> expect +CCW")
    n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
