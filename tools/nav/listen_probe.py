#!/usr/bin/env python3
"""Listen-only during a nav goal: /cmd_vel values, wheel cmd, true pose+yaw, 1 Hz for 50 s."""
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from drdds.msg import JointsDataCmd

class P(Node):
    def __init__(self):
        super().__init__("listen_probe")
        self.cv = None; self.jc = None; self.x = self.y = self.yaw = float("nan")
        self.create_subscription(Twist, "/cmd_vel", lambda m: setattr(self, "cv", m), 10)
        self.create_subscription(JointsDataCmd, "/JOINTS_CMD", lambda m: setattr(self, "jc", m), 10)
        self.create_subscription(Odometry, "/odom_true", self.on_true, 10)
    def on_true(self, m):
        self.x = m.pose.pose.position.x; self.y = m.pose.pose.position.y
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def main():
    rclpy.init(); n = P()
    t0 = time.time(); last = 0.0
    while time.time() - t0 < 50:
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() - last >= 2.0:
            last = time.time()
            cv = "none" if n.cv is None else f"vx={n.cv.linear.x:+.2f} wz={n.cv.angular.z:+.2f}"
            w3 = "none" if n.jc is None else f"{n.jc.data.joints_data[3].velocity:+.2f}"
            print(f"t={time.time()-t0:4.1f}s cmd[{cv}] wheelFL={w3} pose=({n.x:+.2f},{n.y:+.2f},{math.degrees(n.yaw):+.0f}d)", flush=True)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
