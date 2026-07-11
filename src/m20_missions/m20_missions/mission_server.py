"""m20_missions: mission/waypoint server + AI agent interface.

Phase-0 stub. Accepts MissionItem lists (Phase 7 will add an action server and the
LLM/VLM agent gateway). All mission execution requests go through Commander's
/m20/set_mode; this node never bypasses the mode machine (ADR-005).
"""
import rclpy
from rclpy.node import Node
from m20_msgs.msg import MissionItem, HealthReport


class MissionServer(Node):
    def __init__(self):
        super().__init__("m20_mission_server")
        self.queue = []
        self.create_subscription(MissionItem, "/m20/mission/enqueue", self.on_item, 10)
        self.health_pub = self.create_publisher(HealthReport, "/m20/health", 10)
        self.create_timer(1.0, self.heartbeat)
        self.get_logger().info("Mission server up (Phase-0 stub: queue only).")

    def on_item(self, item: MissionItem):
        self.queue.append(item)
        self.get_logger().info(f"queued [{item.label}] type={item.type} (n={len(self.queue)})")
        # TODO(Phase 7): execute via Nav2 NavigateToPose action + SetMode requests.

    def heartbeat(self):
        h = HealthReport()
        h.header.stamp = self.get_clock().now().to_msg()
        h.node_name = self.get_name()
        h.status = HealthReport.STATUS_OK
        self.health_pub.publish(h)


def main():
    rclpy.init()
    rclpy.spin(MissionServer())


if __name__ == "__main__":
    main()
