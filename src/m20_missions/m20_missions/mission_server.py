"""m20_missions: the mission action server (Phase C opener, ADR-014's sole
fleet integration point).

Serves m20_msgs/action/RunMission: an ordered list of MissionItems executed one
at a time — TYPE_GOTO/TYPE_INSPECT via Nav2's NavigateToPose action (we are the
CLIENT of Nav2, the SERVER to operators/agents), TYPE_WAIT via a timer. The
Commander stays boss (ADR-005): if the robot leaves an armed AUTONOMOUS/ASSISTED
mode mid-mission — failsafe, e-stop, operator veto — the mission aborts and the
current Nav2 goal is cancelled. This node never touches /m20/set_mode.

Cancel semantics: a client cancel request cancels the in-flight Nav2 goal and
returns the per-item tally so the operator knows exactly where the robot stopped.
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy,
                       HistoryPolicy)

from nav2_msgs.action import NavigateToPose
from m20_msgs.action import RunMission
from m20_msgs.msg import MissionItem, RobotMode, HealthReport

CRUISE_MPS = 0.25          # crude ETA divisor, matches observed field speed


class MissionServer(Node):
    def __init__(self):
        super().__init__("m20_mission_server")
        self.cb = ReentrantCallbackGroup()
        self.mode_ok = False
        self.robot_xy = None

        # /m20/mode is latched + on-change (same lesson as the behavior engine,
        # 2026-07-20): subscribe transient_local or a late start never sees it.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(RobotMode, "/m20/mode", self.on_mode, latched,
                                 callback_group=self.cb)
        from nav_msgs.msg import Odometry
        self.create_subscription(Odometry, "/odom", self.on_odom, 10,
                                 callback_group=self.cb)

        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose",
                                callback_group=self.cb)
        self.server = ActionServer(
            self, RunMission, "/m20/mission/run",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=self.cb)

        self.health_pub = self.create_publisher(HealthReport, "/m20/health", 10)
        self.create_timer(1.0, self.heartbeat, callback_group=self.cb)
        self.get_logger().info("Mission server up: /m20/mission/run (RunMission).")

    # ---------- inputs ----------
    def on_mode(self, msg: RobotMode):
        self.mode_ok = msg.armed and msg.mode in (RobotMode.MODE_ASSISTED,
                                                  RobotMode.MODE_AUTONOMOUS)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.robot_xy = (p.x, p.y)

    def on_goal(self, goal_request):
        if not goal_request.items:
            return GoalResponse.REJECT
        if not self.mode_ok:
            self.get_logger().warn("mission rejected: not armed AUTONOMOUS/ASSISTED")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # ---------- execution ----------
    def execute(self, goal_handle):
        items = goal_handle.request.items
        result = RunMission.Result()
        for i, item in enumerate(items):
            label = item.label or f"item_{i}"
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.detail = f"canceled before {label}"
                return result
            if not self.mode_ok:
                goal_handle.abort()
                result.detail = f"aborted before {label}: commander veto/mode change"
                return result

            if item.type in (MissionItem.TYPE_GOTO, MissionItem.TYPE_INSPECT):
                ok, why = self._navigate(goal_handle, i, label, item)
            elif item.type == MissionItem.TYPE_WAIT:
                ok, why = self._wait(goal_handle, i, label, item.wait_seconds)
            else:
                ok, why = False, f"unsupported item type {item.type}"

            if not ok:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                result.completed = i
                result.detail = f"stopped at {label}: {why}"
                return result
            result.completed = i + 1

        result.success = True
        result.detail = "ok"
        goal_handle.succeed()
        return result

    def _feedback(self, gh, idx, label, status, eta=-1.0):
        fb = RunMission.Feedback()
        fb.current_index = idx
        fb.current_label = label
        fb.status = status
        fb.eta_seconds = float(eta)
        gh.publish_feedback(fb)

    def _navigate(self, gh, idx, label, item):
        if not self.nav.wait_for_server(timeout_sec=5.0):
            return False, "Nav2 action server unavailable"
        goal = NavigateToPose.Goal()
        goal.pose = item.goal
        goal.pose.header.frame_id = goal.pose.header.frame_id or "map"

        send = self.nav.send_goal_async(goal)
        while not send.done():
            time.sleep(0.05)
        nav_gh = send.result()
        if nav_gh is None or not nav_gh.accepted:
            return False, "Nav2 rejected the goal"

        res_future = nav_gh.get_result_async()
        gx = item.goal.pose.position.x
        gy = item.goal.pose.position.y
        while not res_future.done():
            if gh.is_cancel_requested or not self.mode_ok:
                nav_gh.cancel_goal_async()
                # let Nav2 acknowledge before returning so the robot halts cleanly
                t0 = time.time()
                while not res_future.done() and time.time() - t0 < 5.0:
                    time.sleep(0.1)
                return False, ("cancel requested" if gh.is_cancel_requested
                               else "commander veto/mode change")
            eta = -1.0
            if self.robot_xy is not None:
                d = math.hypot(gx - self.robot_xy[0], gy - self.robot_xy[1])
                eta = d / CRUISE_MPS
            self._feedback(gh, idx, label, "navigating", eta)
            time.sleep(0.5)

        status = res_future.result().status
        if status == 4:                        # GoalStatus.STATUS_SUCCEEDED
            self._feedback(gh, idx, label, "reached", 0.0)
            return True, ""
        return False, f"Nav2 terminal status {status}"

    def _wait(self, gh, idx, label, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            if gh.is_cancel_requested:
                return False, "cancel requested"
            if not self.mode_ok:
                return False, "commander veto/mode change"
            self._feedback(gh, idx, label, "waiting",
                           seconds - (time.time() - t0))
            time.sleep(0.5)
        return True, ""

    def heartbeat(self):
        h = HealthReport()
        h.header.stamp = self.get_clock().now().to_msg()
        h.node_name = self.get_name()
        h.status = HealthReport.STATUS_OK
        h.message = "serving /m20/mission/run"
        self.health_pub.publish(h)


def main():
    rclpy.init()
    node = MissionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()


if __name__ == "__main__":
    main()
