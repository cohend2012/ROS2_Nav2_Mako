#!/usr/bin/env python3
"""Kinematic fake world for the M20-Brain ROS 2 smoke test (NOT a physics sim).

Purpose: prove the ROS WIRING of every mode on real ROS 2 + Nav2 (action servers,
TF, LaserScan binning, OccupancyGrid planning, velocity-smoother chain, mission server,
agent camera path) anywhere, without MuJoCo or the vendor model. Driving QUALITY is
gated in MuJoCo (tools/ai_nav/evaluate.py, tools/agent/sim_mission.py).

Publishes: TF map->odom (identity) + odom->base_link, /odom, /scan (360 x 1 deg,
analytic ray-circle), /map (latched), /camera/image_raw (a rendered gauge whose
value is the `gauge_value` param), /m20/mode (latched: armed AUTONOMOUS).
Subscribes /cmd_vel (unicycle integration, first-order lag like a real base).
"""
import io
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from m20_msgs.msg import RobotMode

# circles (x, y, r) roughly matching the oil_gas_field footprints
OBST = [(-5.0, 4.0, 1.4), (-1.0, 4.5, 1.4), (-4.0, -1.0, 0.8), (0.0, 1.5, 0.55),
        (6.0, -1.0, 0.55), (5.0, 2.0, 0.15), (7.0, 2.0, 0.15)]


def office_boxes(seed=7):
    """The SAME fixed office as tools/sim/make_office_scene.py (seed 7), as 2D boxes
    (x, y, hx, hy, yaw)."""
    sys.path.insert(0, os.environ.get("M20_AI_NAV_SRC", "/ws/src/m20_ai_nav"))
    from m20_ai_nav.sim import office
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "sim"))
    rooms = ["kitchen", "meeting_room", "open_office", "server_room", "manager_office", "print_room"]
    L = office.generate(np.random.default_rng(seed), n_people=0, room_names=rooms)
    return [(b.x, b.y, b.hx, b.hy, b.yaw) for b in L.boxes]


class FakeWorld(Node):
    def __init__(self):
        super().__init__("fake_world")
        self.declare_parameter("gauge_value", 64.0)
        self.declare_parameter("world", "oil")          # oil | office
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.world = self.get_parameter("world").value
        self.boxes = office_boxes() if self.world == "office" else []
        self.circles = [] if self.world == "office" else OBST
        self.x = float(self.get_parameter("start_x").value)
        self.y = float(self.get_parameter("start_y").value)
        self.th = 0.0
        self.v = self.w = 0.0
        self.cmd = (0.0, 0.0)
        self.tfb = TransformBroadcaster(self)
        st = StaticTransformBroadcaster(self)
        t = TransformStamped(); t.header.frame_id = "map"; t.child_frame_id = "odom"
        t.header.stamp = self.get_clock().now().to_msg(); t.transform.rotation.w = 1.0
        st.sendTransform(t)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", latched)
        self.cam_pub = self.create_publisher(Image, "/camera/image_raw", 3)
        self.mode_pub = self.create_publisher(RobotMode, "/m20/mode", latched)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        m = RobotMode(); m.armed = True; m.mode = RobotMode.MODE_AUTONOMOUS
        self.mode_pub.publish(m)
        self.publish_map()
        self.dt = 0.02
        self.create_timer(self.dt, self.step)
        self.create_timer(0.1, self.publish_scan)
        self.create_timer(0.5, self.publish_cam)
        self.get_logger().info("fake world up")

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)

    def step(self):
        a = self.dt / 0.15                                  # 150 ms first-order lag
        self.v += a * (self.cmd[0] - self.v); self.w += a * (self.cmd[1] - self.w)
        self.th = math.atan2(math.sin(self.th + self.w * self.dt), math.cos(self.th + self.w * self.dt))
        self.x += self.v * math.cos(self.th) * self.dt; self.y += self.v * math.sin(self.th) * self.dt
        now = self.get_clock().now().to_msg()
        t = TransformStamped(); t.header.stamp = now; t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(self.th / 2), math.cos(self.th / 2)
        self.tfb.sendTransform(t)
        o = Odometry(); o.header = t.header; o.child_frame_id = "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation = t.transform.rotation
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(o)

    def publish_scan(self):
        n = 360
        ang = -math.pi + np.arange(n) * (2 * math.pi / n)
        wa = self.th + ang
        dx, dy = np.cos(wa), np.sin(wa)
        r = np.full(n, np.inf)
        for (bx, by, hx, hy, byaw) in self.boxes:      # ray vs oriented box (slab test)
            c, sn = math.cos(byaw), math.sin(byaw)
            ox, oy = self.x - bx, self.y - by
            lox, loy = c * ox + sn * oy, -sn * ox + c * oy
            ldx, ldy = c * dx + sn * dy, -sn * dx + c * dy
            with np.errstate(divide="ignore", invalid="ignore"):
                t1x, t2x = (-hx - lox) / ldx, (hx - lox) / ldx
                t1y, t2y = (-hy - loy) / ldy, (hy - loy) / ldy
            tmin = np.maximum(np.minimum(t1x, t2x), np.minimum(t1y, t2y))
            tmax = np.minimum(np.maximum(t1x, t2x), np.maximum(t1y, t2y))
            ok = (tmax >= np.maximum(tmin, 0)) & (tmin > 0)
            r = np.where(ok & (tmin < r), tmin, r)
        for (cx, cy, cr) in self.circles:
            ox, oy = self.x - cx, self.y - cy
            b = ox * dx + oy * dy
            c = ox * ox + oy * oy - cr * cr
            disc = b * b - c
            hit = disc >= 0
            t = -b - np.sqrt(np.where(hit, disc, 0))
            ok = hit & (t > 0)
            r = np.where(ok & (t < r), t, r)
        r[r > 12.0] = np.inf
        s = LaserScan(); s.header.stamp = self.get_clock().now().to_msg(); s.header.frame_id = "base_link"
        s.angle_min, s.angle_max = -math.pi, math.pi - 2 * math.pi / n
        s.angle_increment = 2 * math.pi / n; s.range_min, s.range_max = 0.5, 12.0
        s.scan_time = 0.1
        s.ranges = [float(v) for v in r]
        self.scan_pub.publish(s)

    def publish_map(self):
        res, half = 0.05, 10.0
        n = int(2 * half / res)
        xs = -half + (np.arange(n) + 0.5) * res
        X, Y = np.meshgrid(xs, xs)
        occ = np.zeros((n, n), bool)
        for (cx, cy, cr) in self.circles:
            occ |= np.hypot(X - cx, Y - cy) <= cr
        for (bx, by, hx, hy, byaw) in self.boxes:
            c, sn = math.cos(byaw), math.sin(byaw)
            lx, ly = c * (X - bx) + sn * (Y - by), -sn * (X - bx) + c * (Y - by)
            occ |= (np.abs(lx) <= hx) & (np.abs(ly) <= hy)
        g = OccupancyGrid(); g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.resolution = res; g.info.width = g.info.height = n
        g.info.origin.position.x = g.info.origin.position.y = -half
        g.info.origin.orientation.w = 1.0
        g.data = np.where(occ, 100, 0).astype(np.int8).reshape(-1).tolist()
        self.map_pub.publish(g)

    def publish_cam(self):
        sys.path.insert(0, os.environ.get("M20_AGENT_SRC", "/ws/src/m20_agent"))
        from m20_agent.gauge_dial import render_dial
        from PIL import Image as PImage
        frame = PImage.new("RGB", (320, 240), (120, 118, 110))
        dial = render_dial(float(self.get_parameter("gauge_value").value), 0, 100, "psi", "PG-101", 140)
        frame.paste(dial, (90, 50))
        a = np.asarray(frame)
        m = Image(); m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = "base_link"
        m.height, m.width = a.shape[:2]; m.encoding = "rgb8"; m.step = a.shape[1] * 3
        m.data = a.tobytes()
        self.cam_pub.publish(m)


def main():
    rclpy.init()
    n = FakeWorld()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
