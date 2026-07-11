#!/usr/bin/env python3
"""Quick teleop test: publish JointsDataCmd to drive the sim M20 on flat ground.
Holds the leg joints at their stance (PD) and spins the 4 wheels at a velocity
target -> the wheel-legged M20 rolls. Proves the ROS -> sim actuation path.

Env: WHEEL_VEL (rad/s, default 6; negative = reverse)
"""
import os
import rclpy
from rclpy.node import Node
from drdds.msg import JointsDataCmd

NJ = 16
WHEELS = (3, 7, 11, 15)
STANCE = [0.0, -0.7, 1.4, 0.0, 0.0, -0.7, 1.4, 0.0,
          0.0, 0.7, -1.4, 0.0, 0.0, 0.7, -1.4, 0.0]
VEL = float(os.environ.get("WHEEL_VEL", "6.0"))


class Drive(Node):
    def __init__(self):
        super().__init__("sim_drive_test")
        self.pub = self.create_publisher(JointsDataCmd, "/JOINTS_CMD", 10)
        self.create_timer(0.02, self.tick)          # 50 Hz
        self.get_logger().info(f"driving wheels at {VEL} rad/s (legs holding stance)")

    def tick(self):
        m = JointsDataCmd()
        m.header.stamp = self.get_clock().now().to_msg()
        for i in range(NJ):
            j = m.data.joints_data[i]
            if i in WHEELS:
                j.kp, j.kd = 0.0, 5.0               # velocity control (kd=5 → enough torque to turn)
                j.velocity, j.position, j.torque = VEL, 0.0, 0.0
            else:
                j.kp, j.kd = 200.0, 4.0            # stiff position hold at standing stance
                j.position, j.velocity, j.torque = STANCE[i], 0.0, 0.0
        self.pub.publish(m)


def main():
    rclpy.init()
    try:
        rclpy.spin(Drive())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
