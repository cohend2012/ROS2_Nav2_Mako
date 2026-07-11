#!/usr/bin/env python3
"""course_follower: drive the M20 through a waypoint course using TRAJECTORY GENERATION
(a Catmull-Rom spline through the waypoints) + a PURE-PURSUIT tracker — smooth path
following, not reactive point-to-point control.

Reads pose from ODOM_TOPIC (/odom = sim ground truth, or /odom_filtered = EKF estimate),
publishes geometry_msgs/Twist to /cmd_vel. Validated in tools/traj_benchmark.py
(~0.45 m cross-track on an 8-WP loop). The bridge (armed) turns Twist into joint commands.
"""
import math
import os
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

WAYPOINTS = [(2.0, 0.0), (2.0, 2.5), (-1.5, 2.5), (-1.5, -1.0), (0.5, -1.0)]
ODOM_TOPIC = os.environ.get("ODOM_TOPIC", "/odom")   # /odom (truth) or /odom_filtered (EKF)
LOOKAHEAD = 0.6
SPEED = 0.5


def catmull_rom(pts, n=40):
    """Smooth spline through the waypoints (open path, endpoints duplicated)."""
    P = [pts[0]] + list(pts) + [pts[-1], pts[-1]]
    path = []
    for i in range(1, len(P) - 2):
        p0, p1, p2, p3 = (np.array(P[j], float) for j in (i - 1, i, i + 1, i + 2))
        for t in np.linspace(0, 1, n, endpoint=False):
            path.append(0.5 * ((2 * p1) + (-p0 + p2) * t
                        + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t
                        + (-p0 + 3 * p1 - 3 * p2 + p3) * t * t * t))
    path.append(np.array(pts[-1], float))
    return path


class CourseFollower(Node):
    def __init__(self):
        super().__init__("course_follower")
        self.path = catmull_rom(WAYPOINTS)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 10)
        self.x = self.y = self.yaw = 0.0
        self.have_pose = False
        self.i0 = 0
        self.done = False
        self.create_timer(0.05, self.tick)          # 20 Hz control
        self.get_logger().info(
            f"course_follower up: spline trajectory ({len(self.path)} pts) through "
            f"{len(WAYPOINTS)} waypoints, pure-pursuit, pose from {ODOM_TOPIC}")

    def on_odom(self, m: Odometry):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.have_pose = True

    def tick(self):
        if not self.have_pose or self.done:
            self.pub.publish(Twist())
            return
        # progress: nearest path point in a forward window (monotonic)
        end = min(self.i0 + 80, len(self.path))
        dists = [math.hypot(self.path[j][0] - self.x, self.path[j][1] - self.y)
                 for j in range(self.i0, end)]
        self.i0 = self.i0 + int(np.argmin(dists))
        # completion: at the end of the path and near the last waypoint
        if self.i0 >= len(self.path) - 2 and \
                math.hypot(self.path[-1][0] - self.x, self.path[-1][1] - self.y) < 0.3:
            self.done = True
            self.pub.publish(Twist())
            self.get_logger().info("COURSE COMPLETE")
            return
        # lookahead point LOOKAHEAD metres ahead along the path
        j = self.i0
        s = 0.0
        while s < LOOKAHEAD and j < len(self.path) - 1:
            s += math.hypot(self.path[j + 1][0] - self.path[j][0],
                            self.path[j + 1][1] - self.path[j][1])
            j += 1
        tx, ty = self.path[j]
        alpha = math.atan2(math.sin(math.atan2(ty - self.y, tx - self.x) - self.yaw),
                           math.cos(math.atan2(ty - self.y, tx - self.x) - self.yaw))
        t = Twist()
        t.linear.x = SPEED
        t.angular.z = max(-2.5, min(2.5, 2 * SPEED * math.sin(alpha) / LOOKAHEAD))  # pure-pursuit
        self.pub.publish(t)


def main():
    rclpy.init()
    try:
        rclpy.spin(CourseFollower())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
