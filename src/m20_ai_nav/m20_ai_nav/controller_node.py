"""m20_ai_controller: the learned navigator on the ROS 2 bus — three integration modes.

  mode:=follow_path   HYBRID under Nav2. Serves nav2_msgs/action/FollowPath on
                      /follow_path, impersonating controller_server: Nav2's
                      planner + BT + recoveries stay, the RL policy replaces DWB.
                      Launch: m20_ai_nav nav2_ai.launch.py (Nav2 without
                      controller_server).
  mode:=standalone    OUR OWN global planner (A* on /map) + RL local policy. Serves
                      nav2_msgs/action/NavigateToPose on /navigate_to_pose, so the
                      mission server and agent work unchanged with NO Nav2 running.
  mode:=direct        No map: RL policy chases the goal itself (straight-line carrot)
                      and avoids obstacles reactively from /scan. Same action server
                      as standalone. For unmapped ground / map-server outages.

Output: geometry_msgs/Twist on `cmd_vel_out` (default /cmd_vel_nav -> the existing
velocity_smoother -> /cmd_vel -> bridge). Commander veto and the bridge watchdog are
untouched. Pose: TF global_frame -> base_frame. Scan: /scan (LaserScan, base_link).
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav2_msgs.action import FollowPath, NavigateToPose
from m20_msgs.msg import HealthReport
import tf2_ros

from m20_ai_nav import spec
from m20_ai_nav.core import NavCore, SUCCEEDED, STUCK
from m20_ai_nav.geometry import yaw_from_quat
from m20_ai_nav.grid_planner import GridPlanner
from m20_ai_nav.policy import OnnxPolicy

MODES = ("follow_path", "standalone", "direct")


class AiController(Node):
    def __init__(self):
        super().__init__("m20_ai_controller")
        p = self.declare_parameter
        p("mode", "follow_path"); p("model_path", ""); p("global_frame", "map")
        p("base_frame", "base_link"); p("cmd_vel_out", "/cmd_vel_nav")
        p("lookahead", 1.5); p("xy_goal_tolerance", 0.20); p("yaw_goal_tolerance", 0.10)
        p("stop_range", 0.60); p("stuck_timeout_s", 12.0); p("replan_period_s", 1.0)
        p("scan_timeout_s", 0.5)
        p("planner_robot_radius", 0.55)   # standalone A* lethal radius; office profile: 0.32
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.mode = g("mode")
        if self.mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}, got {self.mode!r}")
        if not g("model_path"):
            raise ValueError("model_path is required (the exported .onnx policy)")
        self.gf, self.bf = g("global_frame"), g("base_frame")
        self.scan_timeout, self.replan_period = g("scan_timeout_s"), g("replan_period_s")
        self.core = NavCore(OnnxPolicy(g("model_path")), lookahead=g("lookahead"),
                            xy_tol=g("xy_goal_tolerance"), yaw_tol=g("yaw_goal_tolerance"),
                            stop_range=g("stop_range"), stuck_timeout_s=g("stuck_timeout_s"))
        self.cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.bins, self.scan_t = None, 0.0
        self.vx_meas = self.wz_meas = 0.0
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data,
                                 callback_group=self.cb)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10, callback_group=self.cb)
        self.cmd_pub = self.create_publisher(Twist, g("cmd_vel_out"), 10)
        self.state_pub = self.create_publisher(String, "/m20/ai_nav/state", 10)
        self.planner = None
        if self.mode == "standalone":
            self.planner = GridPlanner(robot_radius=g("planner_robot_radius"),
                                       inflation_radius=max(0.8, g("planner_robot_radius") + 0.5))
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.map_ok = False
            self.create_subscription(OccupancyGrid, "/map", self.on_map, latched,
                                     callback_group=self.cb)
        if self.mode == "follow_path":
            self.server = ActionServer(self, FollowPath, "follow_path", self.exec_follow_path,
                                       goal_callback=self.on_goal,
                                       cancel_callback=lambda gh: CancelResponse.ACCEPT,
                                       callback_group=self.cb)
        else:
            self.server = ActionServer(self, NavigateToPose, "navigate_to_pose",
                                       self.exec_navigate, goal_callback=self.on_goal,
                                       cancel_callback=lambda gh: CancelResponse.ACCEPT,
                                       callback_group=self.cb)
        self.health_pub = self.create_publisher(HealthReport, "/m20/health", 10)
        self.create_timer(1.0, self.heartbeat, callback_group=self.cb)
        self.active = None
        self._last_fp = None          # (gx, gy, t) of the last preempted follow_path goal
        self.get_logger().info(f"AI controller up: mode={self.mode} "
                               f"model={g('model_path')} -> {g('cmd_vel_out')}")

    # ---------------- inputs ----------------
    def on_scan(self, msg):
        try:
            b = spec.bin_scan(msg.ranges, msg.angle_min, msg.angle_increment)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return
        with self.lock:
            self.bins, self.scan_t = b, time.monotonic()

    def on_odom(self, msg):
        self.vx_meas = msg.twist.twist.linear.x
        self.wz_meas = msg.twist.twist.angular.z

    def on_map(self, msg):
        i = msg.info
        t0 = time.monotonic()
        self.planner.set_map(msg.data, i.width, i.height, i.resolution,
                             i.origin.position.x, i.origin.position.y)
        self.map_ok = True
        self.get_logger().info(f"map {i.width}x{i.height}@{i.resolution} inflated in "
                               f"{time.monotonic() - t0:.2f} s")

    def on_goal(self, _req):
        # one goal at a time; a new goal preempts the old one (Nav2 semantics)
        if self.active is not None:
            # remember what we were driving to BEFORE the old loop unwinds, so the
            # incoming goal can tell "replan of the same goal" from "new goal"
            gx, gy = self.core.tracker.path[-1]
            self._last_fp = (float(gx), float(gy), time.monotonic())
            self.active.abort_requested = True
        else:
            self._last_fp = None
        return GoalResponse.ACCEPT

    def pose(self):
        try:
            t = self.tf_buf.lookup_transform(self.gf, self.bf, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return tr.x, tr.y, yaw_from_quat(q.x, q.y, q.z, q.w)

    # ---------------- the shared drive loop ----------------
    def drive(self, gh, feedback_fn, replan_fn=None):
        """10 Hz loop until SUCCEEDED / STUCK / cancel / preempt. Returns (ok, why)."""
        gh.abort_requested = False
        t_wait = time.monotonic()
        while self.active is not None and time.monotonic() - t_wait < 1.0:
            time.sleep(0.02)            # let a preempted loop unwind: one publisher at a time
        self.active = gh
        period = 1.0 / spec.CONTROL_HZ
        last_replan = time.monotonic()
        try:
            while rclpy.ok():
                t0 = time.monotonic()
                if gh.is_cancel_requested:
                    return False, "canceled"
                if gh.abort_requested:
                    return False, "preempted"
                pose = self.pose()
                with self.lock:
                    bins, age = self.bins, time.monotonic() - self.scan_t
                if pose is None or bins is None or age > self.scan_timeout:
                    self.publish(0.0, 0.0)          # no pose/scan -> no motion, ever
                    self.state("WAITING", "no TF" if pose is None else f"scan age {age:.1f}s")
                else:
                    if replan_fn and t0 - last_replan > self.replan_period:
                        last_replan = t0
                        if not replan_fn(pose):
                            return False, "no path to goal"
                    cmd = self.core.tick(t0, *pose, bins, self.vx_meas, self.wz_meas)
                    self.publish(cmd.vx, cmd.wz)
                    self.state(cmd.state, cmd.detail)
                    if cmd.state == SUCCEEDED:
                        return True, cmd.detail
                    if cmd.state == STUCK:
                        return False, cmd.detail
                    feedback_fn(pose)
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
            return False, "shutdown"
        finally:
            self.publish(0.0, 0.0)
            if self.active is gh:
                self.active = None

    def heartbeat(self):
        """1 Hz HealthReport (docker/healthcheck.py; compose restarts us if it stops).
        WARN while the scan is stale: the node is alive but cannot drive."""
        h = HealthReport()
        h.header.stamp = self.get_clock().now().to_msg()
        h.node_name = self.get_name()
        with self.lock:
            age = time.monotonic() - self.scan_t if self.bins is not None else float("inf")
        h.status = HealthReport.STATUS_OK if age < 2.0 else HealthReport.STATUS_WARN
        h.message = f"{self.mode}; scan age {age:.1f}s; {'goal active' if self.active else 'idle'}"
        self.health_pub.publish(h)

    def publish(self, vx, wz):
        t = Twist(); t.linear.x = float(vx); t.angular.z = float(wz)
        self.cmd_pub.publish(t)

    def state(self, s, detail):
        m = String(); m.data = f"{self.mode}:{s}:{detail}"
        self.state_pub.publish(m)

    # ---------------- mode: follow_path (hybrid under Nav2) ----------------
    def exec_follow_path(self, gh):
        poses = gh.request.path.poses
        if not poses:
            gh.abort(); return FollowPath.Result()
        q = poses[-1].pose.orientation
        xy = [(ps.pose.position.x, ps.pose.position.y) for ps in poses]
        gyaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        # Nav2's BT replans at 1 Hz and delivers each new path as a PREEMPTING goal.
        # Same destination as the goal we were just driving = a replan: keep the
        # stuck timer and the blind-zone memory (a full reset every second would
        # make STUCK impossible and forget obstacles inside the LiDAR blind radius).
        t_wait = time.monotonic()
        while self.active is not None and time.monotonic() - t_wait < 1.0:
            time.sleep(0.02)            # old loop must stop ticking before we touch core
        last = self._last_fp
        if (last is not None and time.monotonic() - last[2] < 3.0
                and math.hypot(xy[-1][0] - last[0], xy[-1][1] - last[1]) < 0.3):
            self.core.update_path(xy)
            self.core.goal_yaw = gyaw
        else:
            self.core.set_path(xy, goal_yaw=gyaw)

        def fb(pose):
            f = FollowPath.Feedback()
            f.distance_to_goal = float(self.core.tracker.remaining(pose[0], pose[1]))
            f.speed = float(self.vx_meas)
            gh.publish_feedback(f)
        ok, why = self.drive(gh, fb)
        if why != "preempted":
            self.get_logger().info(f"follow_path {'OK' if ok else 'FAIL'}: {why}")
        return self.finish(gh, ok, FollowPath.Result())

    # ---------------- modes: standalone / direct (we ARE /navigate_to_pose) ----------
    def exec_navigate(self, gh):
        goal = gh.request.pose
        gx, gy = goal.pose.position.x, goal.pose.position.y
        q = goal.pose.orientation
        gyaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        pose = None
        for _ in range(20):                     # wait up to 2 s for TF
            pose = self.pose()
            if pose is not None:
                break
            time.sleep(0.1)
        if pose is None:
            self.get_logger().error(f"no TF {self.gf}->{self.bf}; rejecting goal")
            gh.abort(); return NavigateToPose.Result()
        replan = None
        if self.mode == "standalone":
            if not self.map_ok:
                self.get_logger().error("standalone mode: no /map received; aborting goal")
                gh.abort(); return NavigateToPose.Result()
            path = self.planner.plan(pose[0], pose[1], gx, gy)
            if path is None:
                self.get_logger().error("A*: no path to goal")
                gh.abort(); return NavigateToPose.Result()
            self.core.set_path(path, goal_yaw=gyaw)

            def replan(p):
                new = self.planner.plan(p[0], p[1], gx, gy)
                if new is None:
                    return False
                self.core.update_path(new)
                return True
        else:  # direct: straight-line carrot, the policy does the avoiding
            self.core.set_path([(pose[0], pose[1]), (gx, gy)], goal_yaw=gyaw)
        t_start = time.monotonic()

        def fb(p):
            f = NavigateToPose.Feedback()
            f.current_pose.header.frame_id = self.gf
            f.current_pose.pose.position.x, f.current_pose.pose.position.y = p[0], p[1]
            f.distance_remaining = float(self.core.tracker.remaining(p[0], p[1]))
            f.navigation_time.sec = int(time.monotonic() - t_start)
            gh.publish_feedback(f)
        ok, why = self.drive(gh, fb, replan)
        self.get_logger().info(f"navigate_to_pose ({self.mode}) {'OK' if ok else 'FAIL'}: {why}")
        return self.finish(gh, ok, NavigateToPose.Result())

    def finish(self, gh, ok, result):
        if ok:
            gh.succeed()
        elif gh.is_cancel_requested:
            gh.canceled()
        else:
            gh.abort()
        return result


def main():
    rclpy.init()
    node = AiController()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
