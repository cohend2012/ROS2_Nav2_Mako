#!/usr/bin/env python3
"""test_driver: publish cmd_vel test patterns live on the ROS 2 bus.

MODE env: circle | figure8 | straight. Drives the (armed) robot through the pattern via
/cmd_vel -> bridge -> joints. Open-loop patterns for movement/turning regression.
"""
import math
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

MODE = os.environ.get("MODE", "circle")
V = float(os.environ.get("V", "0.5"))
W = float(os.environ.get("W", "1.0"))


class TestDriver(Node):
    def __init__(self):
        super().__init__("test_driver")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.t0 = None
        self.create_timer(0.05, self.tick)          # 20 Hz
        self.get_logger().info(f"test_driver up: mode={MODE} v={V} w={W}")

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        cmd = Twist()
        if MODE == "circle":
            cmd.linear.x, cmd.angular.z = V, W
        elif MODE == "figure8":
            cmd.linear.x = V
            cmd.angular.z = W if (t % 20.0) < 10.0 else -W
        elif MODE == "straight":
            cmd.linear.x = V
        self.pub.publish(cmd)


def main():
    rclpy.init()
    try:
        rclpy.spin(TestDriver())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
