"""m20_agent node: language missions on the ROS 2 bus.

  ros2 run m20_agent agent --ros-args -p backend:=claude \
      -p landmarks:=/path/oil_gas_field_landmarks.yaml -p report_dir:=/cfg/out/agent

  ros2 topic pub --once /m20/agent/command std_msgs/String \
      "data: 'read all the gauges and return to where you started'"
  ros2 topic echo /m20/agent/report        # JSON report when the mission ends

Motion goes ONLY through /m20/mission/run (one GOTO/WAIT item per step), so the
commander veto, the mission server's mode gate, and the navigation stack underneath
(Nav2 DWB, Nav2 + AI controller, or the standalone AI navigator) all apply unchanged.
"""
import io
import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
import tf2_ros

from m20_msgs.action import RunMission
from m20_msgs.msg import MissionItem, HealthReport

from m20_agent.executor import Executor, make_backend
from m20_agent.landmarks import LandmarkRegistry


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RosRobotIO:
    def __init__(self, node, frame="map", nav_timeout_s=300.0):
        self.n, self.frame, self.timeout = node, frame, nav_timeout_s
        self.cb = ReentrantCallbackGroup()
        self.mission = ActionClient(node, RunMission, "/m20/mission/run", callback_group=self.cb)
        self.tf = tf2_ros.Buffer()
        self._tfl = tf2_ros.TransformListener(self.tf, node)
        self._img, self._img_t = None, 0.0
        node.create_subscription(Image, "/camera/image_raw", self._on_img, 3,
                                 callback_group=self.cb)
        # find_gauges: the map (survey route), the scan (range along a camera ray) and
        # the camera intrinsics (pixel -> bearing)
        self._map, self._scan, self._planner = None, None, None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(OccupancyGrid, "/map", self._on_map, latched, callback_group=self.cb)
        node.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data,
                                 callback_group=self.cb)
        node.create_subscription(CameraInfo, "/camera/camera_info", self._on_info, 1,
                                 callback_group=self.cb)

    def _on_img(self, msg):
        self._img, self._img_t = msg, time.monotonic()

    def _on_map(self, msg):
        self._map, self._planner = msg, None          # planner rebuilt on next survey

    def _on_scan(self, msg):
        self._scan = msg

    def _on_info(self, msg):
        if msg.k[0] > 0:
            from m20_agent import gauge_finder
            gauge_finder.set_camera(msg.width, msg.height, msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    # --- find_gauges hooks (gauge_search.py) ---
    def survey_viewpoints(self, spacing=3.0, clear=0.9):
        from m20_agent.gauge_finder import survey_viewpoints
        from m20_ai_nav.grid_planner import GridPlanner
        m, p = self._map, self.pose()
        if m is None or p is None:
            return []
        if self._planner is None:
            i = m.info
            self._planner = GridPlanner(robot_radius=0.45, inflation_radius=0.9)
            self._planner.set_map(m.data, i.width, i.height, i.resolution,
                                  i.origin.position.x, i.origin.position.y)
        return survey_viewpoints(self._planner, p[:2], spacing=spacing, clear=clear)

    def view_poses(self):
        """find_gauges: planned views of every mapped surface (gauge_finder.view_plan)."""
        from m20_agent.gauge_finder import view_plan
        p = self.pose()
        if self._map is None or p is None:
            return None
        self.survey_viewpoints()                       # builds self._planner from /map
        return view_plan(self._planner, p[:2]) or None

    def _scan_hit(self, bearing):
        sc, p = self._scan, self.pose()
        if sc is None or p is None:
            return None
        i = int(round((bearing - sc.angle_min) / sc.angle_increment))
        if not 0 <= i < len(sc.ranges):
            return None
        r = sc.ranges[i]
        if not (sc.range_min < r < min(sc.range_max, 12.0)) or math.isnan(r):
            return None
        # the scan is in base_link (pointcloud_to_laserscan target_frame)
        h = p[2] + bearing
        return r, (p[0] + r * math.cos(h), p[1] + r * math.sin(h))

    def range_along(self, bearing, elevation):
        """LiDAR range along a camera ray. The 2-D scan is a height band through the
        dial height, so elevation is ignored; the camera sits on the body axis, so the
        camera bearing is used as the scan bearing (parallax < 2 deg beyond 1.5 m)."""
        return self._scan_hit(bearing)

    def face_normal(self, bearing, elevation):
        a, b = self._scan_hit(bearing + math.radians(1.5)), self._scan_hit(bearing - math.radians(1.5))
        p = self.pose()
        if not a or not b or p is None:
            return None
        (ax, ay), (bx, by) = a[1], b[1]
        L = math.hypot(ax - bx, ay - by)
        if L < 1e-3:
            return None
        nx, ny = -(ay - by) / L, (ax - bx) / L
        if (p[0] - (ax + bx) / 2) * nx + (p[1] - (ay + by) / 2) * ny < 0:
            nx, ny = -nx, -ny
        return nx, ny

    def pose(self):
        try:
            t = self.tf.lookup_transform(self.frame, "base_link", rclpy.time.Time(),
                                         timeout=rclpy.duration.Duration(seconds=2.0))
        except tf2_ros.TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, _yaw(t.transform.rotation)

    def cancel(self):
        gh = getattr(self, "_gh", None)
        if gh is not None:
            gh.cancel_goal_async()

    def _run(self, item):
        if not self.mission.wait_for_server(timeout_sec=5.0):
            return False, "mission server /m20/mission/run unavailable"
        goal = RunMission.Goal(); goal.items = [item]
        fut = self.mission.send_goal_async(goal)
        while not fut.done():
            time.sleep(0.05)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False, "mission rejected (robot not armed AUTONOMOUS/ASSISTED?)"
        self._gh = gh
        res = gh.get_result_async()
        t0 = time.monotonic()
        while not res.done():
            if time.monotonic() - t0 > self.timeout:
                gh.cancel_goal_async()
                return False, f"timeout after {self.timeout:.0f} s"
            time.sleep(0.2)
        self._gh = None
        r = res.result().result
        return bool(r.success), r.detail

    def navigate(self, x, y, yaw, label):
        if yaw is None:                                # any heading: face the way we go
            p = self.pose()
            yaw = math.atan2(y - p[1], x - p[0]) if p else 0.0
        it = MissionItem(); it.type = MissionItem.TYPE_GOTO; it.label = label
        it.goal.header.frame_id = self.frame
        it.goal.pose.position.x, it.goal.pose.position.y = float(x), float(y)
        it.goal.pose.orientation.z = math.sin(yaw / 2); it.goal.pose.orientation.w = math.cos(yaw / 2)
        return self._run(it)

    def wait(self, seconds):
        it = MissionItem(); it.type = MissionItem.TYPE_WAIT; it.wait_seconds = float(seconds)
        it.label = "wait"
        return self._run(it)

    def capture(self, settle_s=1.0, max_age_s=1.0):
        time.sleep(settle_s)                       # let the body stop rocking
        m = self._img
        if m is None or time.monotonic() - self._img_t > max_age_s:
            return None
        if m.encoding not in ("rgb8", "bgr8"):
            self.n.get_logger().warn(f"camera encoding {m.encoding} unsupported")
            return None
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step // 3, 3)[:, :m.width]
        if m.encoding == "bgr8":
            a = a[..., ::-1]
        from PIL import Image as PImage
        buf = io.BytesIO(); PImage.fromarray(a).save(buf, "PNG")
        return buf.getvalue()


class AgentNode(Node):
    def __init__(self):
        super().__init__("m20_agent")
        self.declare_parameter("backend", "claude")
        self.declare_parameter("landmarks", "")
        self.declare_parameter("report_dir", "")
        self.declare_parameter("frame", "map")
        self.declare_parameter("taught_places", "")   # overlay file for "this is the kitchen"
        path = self.get_parameter("landmarks").value
        if not path:
            raise ValueError("landmarks:=<yaml> is required")
        self.reg = LandmarkRegistry.load(path, self.get_parameter("taught_places").value or None)
        self.io = RosRobotIO(self, self.get_parameter("frame").value)
        log = lambda s: self.get_logger().info(s)  # noqa: E731
        self.exe = Executor(self.reg, make_backend(self.get_parameter("backend").value, self.reg),
                            self.io, self.get_parameter("report_dir").value or None, log=log)
        self.report_pub = self.create_publisher(String, "/m20/agent/report", 10)
        self.health_pub = self.create_publisher(HealthReport, "/m20/health", 10)
        self.create_timer(1.0, self._heartbeat)
        self.busy = threading.Lock()
        self.create_subscription(String, "/m20/agent/command", self.on_cmd, 10,
                                 callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f"agent up: backend={self.exe.vlm.name}, "
                               f"{len(self.reg.names())} landmarks; send /m20/agent/command")

    def _heartbeat(self):
        h = HealthReport()
        h.header.stamp = self.get_clock().now().to_msg()
        h.node_name = self.get_name()
        h.status = HealthReport.STATUS_OK
        h.message = f"backend={self.exe.vlm.name}; {'mission running' if self.busy.locked() else 'idle'}"
        self.health_pub.publish(h)

    STOP_WORDS = ("stop", "halt", "cancel", "abort", "freeze", "pause", "e-stop", "estop")

    def on_cmd(self, msg):
        words = msg.data.lower().strip().rstrip(".!").split()
        if words and words[0] in self.STOP_WORDS and len(words) <= 3:
            # STOP never goes through a model and never waits for the planner
            self.get_logger().warn(f"STOP from operator: {msg.data!r}")
            self.exe.stop()
            return
        if not self.busy.acquire(blocking=False):
            self.get_logger().warn("mission already running; command ignored")
            return
        def work():
            try:
                rep = self.exe.run(msg.data)
                out = String(); out.data = json.dumps(rep)
                self.report_pub.publish(out)
            finally:
                self.busy.release()
        threading.Thread(target=work, daemon=True).start()


def main():
    rclpy.init()
    node = AgentNode()
    ex = MultiThreadedExecutor(num_threads=4); ex.add_node(node)
    try:
        ex.spin()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
