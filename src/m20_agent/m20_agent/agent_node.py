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
from sensor_msgs.msg import Image
from std_msgs.msg import String
import tf2_ros

from m20_msgs.action import RunMission
from m20_msgs.msg import MissionItem

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

    def _on_img(self, msg):
        self._img, self._img_t = msg, time.monotonic()

    def pose(self):
        try:
            t = self.tf.lookup_transform(self.frame, "base_link", rclpy.time.Time(),
                                         timeout=rclpy.duration.Duration(seconds=2.0))
        except tf2_ros.TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, _yaw(t.transform.rotation)

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
        res = gh.get_result_async()
        t0 = time.monotonic()
        while not res.done():
            if time.monotonic() - t0 > self.timeout:
                gh.cancel_goal_async()
                return False, f"timeout after {self.timeout:.0f} s"
            time.sleep(0.2)
        r = res.result().result
        return bool(r.success), r.detail

    def navigate(self, x, y, yaw, label):
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
        path = self.get_parameter("landmarks").value
        if not path:
            raise ValueError("landmarks:=<yaml> is required")
        self.reg = LandmarkRegistry.load(path)
        self.io = RosRobotIO(self, self.get_parameter("frame").value)
        log = lambda s: self.get_logger().info(s)  # noqa: E731
        self.exe = Executor(self.reg, make_backend(self.get_parameter("backend").value, self.reg),
                            self.io, self.get_parameter("report_dir").value or None, log=log)
        self.report_pub = self.create_publisher(String, "/m20/agent/report", 10)
        self.busy = threading.Lock()
        self.create_subscription(String, "/m20/agent/command", self.on_cmd, 10,
                                 callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f"agent up: backend={self.exe.vlm.name}, "
                               f"{len(self.reg.names())} landmarks; send /m20/agent/command")

    def on_cmd(self, msg):
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
