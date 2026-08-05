#!/usr/bin/env python3
"""anchor_guardian: runtime drift bounding for localization-mode missions.

The M1a patrol gate measured slam localization MIS-LOCKING mid-mission (est
jumped ~2 m crossing the sparse N field and never re-converged; robot then
fought phantom geometry for 203 s). This node is the validated fix:

  watch   residual = | slam estimate (TF map->base_link) - smoothed GPS |
  fire    when residual > THRESH sustained HOLD_S  ->  publish /initialpose
          at (smoothed GPS position, current slam yaw) so slam_toolbox
          localization re-seeds and re-converges
  cool    COOLDOWN_S between fires

Thresholds validated offline against the recorded 2026-07-31 patrol failure:
first fire at t=147 s exactly as the mis-lock formed; zero false fires in the
clean 147 s before it (tools/ci/out + commit e1d2510 analysis).

GPS position sigma is ~0.8 m; a 1-s moving average brings the reference to
~0.35 m — good enough to detect a >1 m mis-lock, nowhere near good enough to
degrade a healthy scan-match (which stays cm-level between fires).
Yaw is NOT corrected (GPS can't observe it; mis-lock is translational).
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener

THRESH = float(os.environ.get("GUARDIAN_THRESH", "1.0"))     # m
HOLD_S = float(os.environ.get("GUARDIAN_HOLD", "3.0"))       # sustained secs
COOLDOWN_S = float(os.environ.get("GUARDIAN_COOLDOWN", "15.0"))
GPS_WINDOW = 5                                               # ~1 s at 5 Hz


class AnchorGuardian(Node):
    def __init__(self):
        super().__init__("anchor_guardian")
        self.gps_buf = []
        self.over_since = None
        self.last_fire = 0.0
        self.create_subscription(Odometry, "/gps", self.on_gps, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.tfbuf = Buffer()
        self.tfl = TransformListener(self.tfbuf, self)
        self.create_timer(0.5, self.tick)                    # 2 Hz residual check
        self.get_logger().info(
            f"anchor_guardian up: thresh {THRESH} m, hold {HOLD_S} s, cooldown {COOLDOWN_S} s")

    def on_gps(self, m):
        self.gps_buf.append((m.pose.pose.position.x, m.pose.pose.position.y))
        if len(self.gps_buf) > GPS_WINDOW:
            self.gps_buf.pop(0)

    def tick(self):
        if len(self.gps_buf) < GPS_WINDOW:
            return
        try:
            t = self.tfbuf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return
        ex, ey = t.transform.translation.x, t.transform.translation.y
        gx = sum(p[0] for p in self.gps_buf) / len(self.gps_buf)
        gy = sum(p[1] for p in self.gps_buf) / len(self.gps_buf)
        residual = math.hypot(ex - gx, ey - gy)
        now = time.time()
        if residual <= THRESH:
            self.over_since = None
            return
        if self.over_since is None:
            self.over_since = now
            self.get_logger().warn(f"residual {residual:.2f} m > {THRESH} — watching")
            return
        if now - self.over_since < HOLD_S or now - self.last_fire < COOLDOWN_S:
            return
        # FIRE: re-seed slam localization at GPS position, keep current slam yaw
        # (translational mis-lock; GPS cannot observe yaw)
        p = PoseWithCovarianceStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = "map"
        p.pose.pose.position.x = gx
        p.pose.pose.position.y = gy
        p.pose.pose.orientation = t.transform.rotation
        p.pose.covariance[0] = p.pose.covariance[7] = 0.25   # 0.5 m sigma
        p.pose.covariance[35] = 0.05                          # yaw kept, small var
        self.pub.publish(p)
        self.last_fire = now
        self.over_since = None
        self.get_logger().warn(
            f"RE-ANCHOR FIRED: residual {residual:.2f} m, slam ({ex:.2f},{ey:.2f}) "
            f"-> gps ({gx:.2f},{gy:.2f})")


def main():
    rclpy.init()
    try:
        rclpy.spin(AnchorGuardian())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
