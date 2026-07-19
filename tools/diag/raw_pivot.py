#!/usr/bin/env python3
"""Raw pivot: publish /JOINTS_CMD directly (NO bridge) — left wheels -1.4, right +1.4
rad/s (CCW pivot), legs held at stance. 15 s. Prints true yaw before/after.
Isolates pure sim physics from all control layers."""
import math, time
import rclpy
from rclpy.node import Node
from drdds.msg import JointsDataCmd
from nav_msgs.msg import Odometry

STANCE = [0.0,-0.7,1.4,0.0, 0.0,-0.7,1.4,0.0, 0.0,0.7,-1.4,0.0, 0.0,0.7,-1.4,0.0]
WHEELS = (3, 7, 11, 15); LEFT = (3, 11)

def qyaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

class P(Node):
    def __init__(self):
        super().__init__("raw_pivot")
        self.pub = self.create_publisher(JointsDataCmd, "/JOINTS_CMD", 10)
        self.yaw = None
        self.create_subscription(Odometry, "/odom_true",
            lambda m: setattr(self, "yaw", qyaw(m.pose.pose.orientation)), 10)
    def cmd(self, vl, vr):
        m = JointsDataCmd()
        for i in range(16):
            j = m.data.joints_data[i]
            if i in WHEELS:
                j.kp, j.kd = 0.0, 5.0
                j.velocity = vl if i in LEFT else vr
            else:
                j.kp, j.kd = 200.0, 4.0
                j.position = STANCE[i]
        self.pub.publish(m)

def main():
    rclpy.init(); n = P()
    while n.yaw is None and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
    y0 = n.yaw
    print(f"before: yaw={math.degrees(y0):+.1f} deg")
    t0 = time.time(); unwrapped = y0; prev = y0
    while time.time() - t0 < 15 and rclpy.ok():
        n.cmd(-1.4, 1.4)                     # CCW pivot (mixer convention: -vx sign)
        rclpy.spin_once(n, timeout_sec=0.02)
        d = n.yaw - prev
        if d > math.pi: d -= 2*math.pi
        if d < -math.pi: d += 2*math.pi
        unwrapped += d; prev = n.yaw
        time.sleep(0.03)
    n.cmd(0.0, 0.0)
    total = math.degrees(unwrapped - y0)
    print(f"after 15 s of +/-1.4 rad/s wheels: total rotation = {total:+.1f} deg "
          f"(expect ~+430 deg for a clean pivot at R_eff=0.072, B=0.40)")
    n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
