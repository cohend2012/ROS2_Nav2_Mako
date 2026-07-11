"""m20_commander: mode/arming/failsafe state machine.

PX4-Commander analog. The ONLY node allowed to publish RobotMode.
All mode changes go through the /m20/set_mode service; failsafes preempt everything.

Phase-0 stub: state machine + service + failsafe skeleton are real; the monitor
inputs (battery, comms, estimator covariance, e-stop) are wired to placeholder
subscriptions marked TODO until OQ-3/OQ-4 (see docs/SECOND_BRAIN.md) are resolved.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from m20_msgs.msg import RobotMode, FailsafeStatus, HealthReport, ControlAuthority
from m20_msgs.srv import SetMode, SetControlAuthority

# Legal transitions. ESTOP is reachable from everywhere; leaving ESTOP requires IDLE.
LEGAL = {
    RobotMode.MODE_IDLE: {RobotMode.MODE_TELEOP, RobotMode.MODE_ASSISTED,
                          RobotMode.MODE_AUTONOMOUS, RobotMode.MODE_ESTOP},
    RobotMode.MODE_TELEOP: {RobotMode.MODE_IDLE, RobotMode.MODE_ASSISTED,
                            RobotMode.MODE_ESTOP},
    RobotMode.MODE_ASSISTED: {RobotMode.MODE_IDLE, RobotMode.MODE_TELEOP,
                              RobotMode.MODE_AUTONOMOUS, RobotMode.MODE_ESTOP},
    RobotMode.MODE_AUTONOMOUS: {RobotMode.MODE_IDLE, RobotMode.MODE_ASSISTED,
                                RobotMode.MODE_ESTOP},
    RobotMode.MODE_ESTOP: {RobotMode.MODE_IDLE},
}


class Commander(Node):
    def __init__(self):
        super().__init__("m20_commander")
        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode = RobotMode.MODE_IDLE
        self.armed = False
        self.authority = "startup"
        self.active_failsafes = 0

        # Control authority (ADR-008): who owns the actuators. Default: vendor tier.
        self.authority_tier = ControlAuthority.TIER_VENDOR
        self.authority_holder = ""
        self.authority_lease_deadline = None

        self.mode_pub = self.create_publisher(RobotMode, "/m20/mode", latched)
        self.failsafe_pub = self.create_publisher(FailsafeStatus, "/m20/failsafe", 10)
        self.authority_pub = self.create_publisher(ControlAuthority, "/m20/control_authority", latched)
        self.set_mode_srv = self.create_service(SetMode, "/m20/set_mode", self.on_set_mode)
        self.set_authority_srv = self.create_service(
            SetControlAuthority, "/m20/set_control_authority", self.on_set_authority)
        self.health_sub = self.create_subscription(HealthReport, "/m20/health",
                                                   self.on_health, 20)
        # Commander publishes its own heartbeat too (HealthReport contract: every m20_* node @1 Hz).
        self.health_pub = self.create_publisher(HealthReport, "/m20/health", 10)

        # TODO(OQ-3): subscribe battery state from the bridge.
        # TODO(OQ-4): subscribe physical/software e-stop state.
        # TODO(Phase 2): subscribe estimator covariance for divergence detection.

        self.create_timer(0.5, self.tick)  # 2 Hz failsafe evaluation + publish
        self.create_timer(1.0, self.heartbeat)  # 1 Hz own health heartbeat
        self.publish_mode()
        self.get_logger().info("Commander up. Mode=IDLE, disarmed.")

    # ---------- mode machine ----------
    def on_set_mode(self, req: SetMode.Request, res: SetMode.Response):
        if self.active_failsafes and req.mode != RobotMode.MODE_ESTOP \
                and req.mode != RobotMode.MODE_IDLE:
            res.accepted = False
            res.reason = f"denied: active failsafes 0x{self.active_failsafes:x}"
            return res
        if req.mode not in LEGAL.get(self.mode, set()):
            res.accepted = False
            res.reason = f"illegal transition {self.mode} -> {req.mode}"
            return res
        self.mode = req.mode
        self.authority = req.requester or "unknown"
        self.armed = self.mode in (RobotMode.MODE_TELEOP, RobotMode.MODE_ASSISTED,
                                   RobotMode.MODE_AUTONOMOUS)
        self.publish_mode()
        self.get_logger().info(f"Mode -> {self.mode} (by {self.authority})")
        res.accepted = True
        res.reason = "ok"
        return res

    def publish_mode(self):
        m = RobotMode()
        m.header.stamp = self.get_clock().now().to_msg()
        m.mode = self.mode
        m.authority_source = self.authority
        m.armed = self.armed
        self.mode_pub.publish(m)

    # ---------- control authority (ADR-008, PX4 offboard analog) ----------
    def on_set_authority(self, req: SetControlAuthority.Request,
                         res: SetControlAuthority.Response):
        if req.tier == ControlAuthority.TIER_VENDOR:
            self._revoke_authority(f"released by {req.requester}")
            res.granted, res.reason = True, "reverted to vendor tier"
            return res
        if self.active_failsafes or not self.armed:
            res.granted, res.reason = False, "denied: disarmed or failsafe active"
            return res
        if self.authority_tier == ControlAuthority.TIER_POLICY:
            res.granted, res.reason = False, f"denied: held by {self.authority_holder}"
            return res
        self.authority_tier = ControlAuthority.TIER_POLICY
        self.authority_holder = req.requester
        lease_s = min(req.lease_ms, 600_000) * 1e-3
        self.authority_lease_deadline = self.get_clock().now().nanoseconds * 1e-9 + lease_s
        self.publish_authority(int(lease_s * 1000))
        self.get_logger().warn(f"POLICY authority -> {req.requester} ({lease_s:.1f}s lease)")
        res.granted, res.reason = True, "lease granted"
        return res

    def _revoke_authority(self, reason: str):
        if self.authority_tier != ControlAuthority.TIER_VENDOR:
            self.get_logger().warn(f"authority revoked: {reason}")
        self.authority_tier = ControlAuthority.TIER_VENDOR
        self.authority_holder = ""
        self.authority_lease_deadline = None
        self.publish_authority(0)

    def publish_authority(self, lease_ms: int):
        a = ControlAuthority()
        a.header.stamp = self.get_clock().now().to_msg()
        a.tier = self.authority_tier
        a.holder = self.authority_holder
        a.lease_ms = lease_ms
        self.authority_pub.publish(a)

    # ---------- failsafes ----------
    def on_health(self, msg: HealthReport):
        if msg.status >= HealthReport.STATUS_ERROR:
            self.raise_failsafe(FailsafeStatus.FS_MODULE_UNHEALTHY,
                                2, f"{msg.node_name}: {msg.message}")

    def raise_failsafe(self, flag: int, severity: int, detail: str):
        if not (self.active_failsafes & flag):
            self.get_logger().error(f"FAILSAFE 0x{flag:x}: {detail}")
        self.active_failsafes |= flag
        self._revoke_authority(f"failsafe 0x{flag:x}")  # ADR-008: failsafe kills any lease
        if severity >= 2 and self.mode != RobotMode.MODE_ESTOP:
            self.mode = RobotMode.MODE_ESTOP
            self.armed = False
            self.authority = "failsafe"
            self.publish_mode()

    def tick(self):
        if (self.authority_lease_deadline is not None and
                self.get_clock().now().nanoseconds * 1e-9 > self.authority_lease_deadline):
            self._revoke_authority("lease expired")
        fs = FailsafeStatus()
        fs.header.stamp = self.get_clock().now().to_msg()
        fs.active = self.active_failsafes
        fs.severity = 2 if self.mode == RobotMode.MODE_ESTOP else \
                      (1 if self.active_failsafes else 0)
        fs.detail = ""
        self.failsafe_pub.publish(fs)

    def heartbeat(self):
        h = HealthReport()
        h.header.stamp = self.get_clock().now().to_msg()
        h.node_name = self.get_name()
        h.status = HealthReport.STATUS_OK
        self.health_pub.publish(h)


def main():
    rclpy.init()
    rclpy.spin(Commander())


if __name__ == "__main__":
    main()
