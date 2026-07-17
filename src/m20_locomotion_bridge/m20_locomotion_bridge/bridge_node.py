"""m20_locomotion_bridge: the ONLY node that talks to the Deep Robotics SDK.

PX4 mixer/actuator analog. Everything above this node is vendor-agnostic.

Contract:
  subscribes  /cmd_vel            (geometry_msgs/Twist)   velocity commands
  subscribes  /m20/gait_request   (m20_msgs/GaitRequest)  gait/mode requests
  subscribes  /m20/mode           (m20_msgs/RobotMode)    only forwards cmds when armed
  publishes   /m20/odom_vendor    (nav_msgs/Odometry)     vendor odometry passthrough
  publishes   /m20/health         (m20_msgs/HealthReport) 1 Hz heartbeat
Watchdog: if no /cmd_vel arrives for `cmd_timeout_s`, command zero velocity.

Phase-0 stub: the VendorSDK class is a placeholder. Replace its methods with real
Deep Robotics SDK calls once OQ-3 (SDK surface: topics/UDP API, gait switching)
is resolved. Keep ALL vendor calls inside VendorSDK so the swap is one file.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from m20_msgs.msg import GaitRequest, RobotMode, HealthReport


class VendorSDK:
    """Placeholder for the Deep Robotics M20 SDK. See OQ-3 in SECOND_BRAIN.md."""

    def connect(self) -> bool:
        return True  # TODO: real connection (UDP / vendor ROS 2 topics)

    def send_velocity(self, vx: float, vy: float, wz: float) -> None:
        pass  # TODO

    def request_gait(self, gait: int) -> bool:
        return True  # TODO

    def stop(self) -> None:
        pass  # TODO: guaranteed-stop primitive if the SDK provides one


# M20 standing stance (found empirically; body ~0.46 m, level), per leg [hipx, hipy, knee, wheel].
# The vendor JOINT_INIT (knee≈limit) is a folded start pose, not a standing one.
_STANCE = [0.0, -0.7, 1.4, 0.0, 0.0, -0.7, 1.4, 0.0,
           0.0, 0.7, -1.4, 0.0, 0.0, 0.7, -1.4, 0.0]
_WHEELS = (3, 7, 11, 15)
_LEFT = (3, 11)          # FL, HL wheels
_RIGHT = (7, 15)         # FR, HR wheels

# TODO: Build a system that can connect to the real robot and the sim and switch between them.

# TODO: Build a way to make the robot walk in a gait. I think we need to add a gait request topic to the bridge node.


class SimVendorSDK:
    """Sim 'vendor' backend: drives the MuJoCo sim over the drdds joint interface
    (ADR-015). Holds the standing stance on the legs and skid-steers the wheels from
    the /cmd_vel the bridge forwards. This is the sim analog of the real VendorSDK."""

    # wheel_radius: CALIBRATED effective rolling radius (0.072 m, measured from test
    # drives — matches ODOM_R in mujoco_sim.py and R in estimator.py). The nominal
    # 0.10 m under-commands wheel speed ~28%: forward runs slow, and pivot falls below
    # the skid-scrub stiction threshold entirely (root cause of the 2026-07-17
    # rotate-to-heading deadlock: Nav2 flapping reset the yaw integral faster than it
    # could wind up the missing 39% of wheel speed).
    def __init__(self, node, wheel_radius=0.072, track=0.40):
        from drdds.msg import JointsDataCmd, ImuData   # lazy: only when sim backend chosen
        self._Cmd = JointsDataCmd
        self.node = node
        self.pub = node.create_publisher(JointsDataCmd, "/JOINTS_CMD", 10)
        node.create_subscription(ImuData, "/IMU_DATA", self._on_imu, 10)
        self.R, self.B = wheel_radius, track
        # Closed-loop yaw rate: open-loop skid-steer understeers (slip), so trim the
        # wheel differential by the measured yaw-rate error.
        # CASCADE RULE (learned 2026-07-17): this inner loop sits UNDER Nav2's 20 Hz
        # heading controller. A strong integrator here (Ki=0.8) added phase lag and
        # overshoot (body hit -0.59 rad/s on a -0.44 command) -> limit-cycle wiggle
        # with the outer loop: the robot oscillated instead of rotating (root cause
        # of the rotate-to-heading deadlock; measured via /cmd_vel_nav vs /cmd_vel vs
        # omega_z probe). Inner loop must be fast and overshoot-free: feedforward +
        # strong P, integrator reduced to a slow trim. Constant-setpoint teleop worked
        # with the old gains, which is why every direct drive test passed while Nav2
        # failed — test with a reactive outer loop, not just constant commands.
        self._omega = 0.0; self._yint = 0.0; self._last_t = None
        self.Kp, self.Ki = 3.0, 0.1

    def _on_imu(self, msg) -> None:
        self._omega = float(msg.data.omega_z)         # measured body yaw rate

    def connect(self) -> bool:
        return True

    def request_gait(self, gait: int) -> bool:
        return True

    def send_velocity(self, vx: float, vy: float, wz: float) -> None:
        # +vx = forward, +wz = CCW. Trim wz by the yaw-rate error so the body actually
        # turns at wz despite skid-steer slip. In this model +wheel-vel drives -x (vx negated).

        #TODO: Remove magic numbers and make the gains configurable.
        now = self.node.get_clock().now().nanoseconds * 1e-9
        dt = (now - self._last_t) if self._last_t else 0.05
        self._last_t = now
        yerr = wz - self._omega
        self._yint = max(-2.0, min(2.0, self._yint + yerr * dt))
        w_cmd = max(-5.0, min(5.0, wz + self.Kp * yerr + self.Ki * self._yint))
        # Skid-scrub dead-zone compensation: pivoting needs |w| >= ~0.5 rad/s of wheel
        # differential to break lateral scrub stiction — commands below that produce
        # ~no rotation (measured 2026-07-17: wz=0.44 held for 100 s yielded 24 deg).
        # Nav2's accel-limited ramps (which ramp FROM MEASURED rate) can never cross a
        # dead zone wider than accel*dt on their own, so the bridge linearizes the
        # plant: clear rotation intent (|wz|>=0.15) is boosted to the breakout rate.
        # The real M20's vendor controller does its own low-level compensation; this
        # stays in the sim SDK path only.
        # ONLY for pivot intent (|vx| ~ 0): during forward driving, wheel rolling
        # breaks the scrub stiction and small wz steering corrections work fine —
        # boosting those to 0.5 causes bang-bang swerving (broke straight-line nav
        # when first tried without the vx guard).
        if abs(vx) < 0.05 and abs(wz) >= 0.15 and abs(w_cmd) < 0.5:
            w_cmd = math.copysign(0.5, w_cmd if w_cmd != 0.0 else wz)
        vl = (-vx + w_cmd * self.B / 2.0) / self.R    # rad/s, left wheels
        vr = (-vx - w_cmd * self.B / 2.0) / self.R    # rad/s, right wheels
        self._publish(vl, vr)

    def stop(self) -> None:
        self._yint = 0.0                              # reset integrator on halt (anti-windup)
        self._publish(0.0, 0.0)

    def _publish(self, vl: float, vr: float) -> None:
        m = self._Cmd()
        m.header.stamp = self.node.get_clock().now().to_msg()
        for i in range(16):
            j = m.data.joints_data[i]
            if i in _WHEELS:
                j.kp, j.kd = 0.0, 5.0            # kd=5: enough wheel torque to skid-steer/pivot
                j.velocity = vl if i in _LEFT else vr
            else:
                j.kp, j.kd = 200.0, 4.0
                j.position = _STANCE[i]
        self.pub.publish(m)


class LocomotionBridge(Node):
    def __init__(self):
        super().__init__("m20_locomotion_bridge")
        # 1.0 s (was 0.5): Nav2's BT has natural /cmd_vel gaps of 0.5-0.8 s between
        # FollowPath attempts and recovery transitions; at 0.5 s the watchdog halted
        # (and reset the yaw integral) on every transition, starving in-place rotation
        # of authority. Still a real failsafe — a dead Nav2 halts the robot in 1 s.
        self.declare_parameter("cmd_timeout_s", 1.0)
        self.timeout = self.get_parameter("cmd_timeout_s").value
        # Backend: "stub" (default; real-robot placeholder + unit tests) or "sim"
        # (drives the MuJoCo sim over drdds). Keeps drdds out of the ROS-free tests.
        self.declare_parameter("sdk_backend", "stub")
        backend = self.get_parameter("sdk_backend").value

        self.sdk = SimVendorSDK(self) if backend == "sim" else VendorSDK()
        self.connected = self.sdk.connect()
        self.armed = False
        self.last_cmd_time = self.get_clock().now()
        self.stopped = True

        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(GaitRequest, "/m20/gait_request", self.on_gait, 10)
        # /m20/mode is latched by the Commander (TRANSIENT_LOCAL); match it so a
        # (re)started bridge receives the current mode immediately and arms.
        mode_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(RobotMode, "/m20/mode", self.on_mode, mode_qos)
        self.health_pub = self.create_publisher(HealthReport, "/m20/health", 10)

        self.create_timer(0.1, self.watchdog)   # 10 Hz
        self.create_timer(1.0, self.heartbeat)  # 1 Hz
        self.get_logger().info(f"Bridge up. SDK connected={self.connected}")

    def on_mode(self, msg: RobotMode):
        self.armed = msg.armed
        if not self.armed:
            self.halt("disarmed by commander")

    def on_cmd_vel(self, msg: Twist):
        if not (self.armed and self.connected):
            return
        self.last_cmd_time = self.get_clock().now()
        self.stopped = False
        self.sdk.send_velocity(msg.linear.x, msg.linear.y, msg.angular.z)

    def on_gait(self, msg: GaitRequest):
        if not (self.armed and self.connected):
            self.get_logger().warn(f"gait request from {msg.requester} ignored (disarmed)")
            return
        ok = self.sdk.request_gait(msg.gait)
        self.get_logger().info(f"gait {msg.gait} by {msg.requester}: {'ok' if ok else 'REJECTED'}")

    def watchdog(self):
        if self.stopped:
            return
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.timeout:
            self.halt(f"cmd_vel timeout ({age:.2f}s > {self.timeout}s)")

    def halt(self, reason: str):
        if not self.stopped:
            self.get_logger().warn(f"HALT: {reason}")
        self.sdk.stop()
        self.sdk.send_velocity(0.0, 0.0, 0.0)
        self.stopped = True

    def heartbeat(self):
        h = HealthReport()
        h.header.stamp = self.get_clock().now().to_msg()
        h.node_name = self.get_name()
        h.status = HealthReport.STATUS_OK if self.connected else HealthReport.STATUS_ERROR
        h.message = "" if self.connected else "vendor SDK not connected"
        self.health_pub.publish(h)


def main():
    rclpy.init()
    rclpy.spin(LocomotionBridge())


if __name__ == "__main__":
    main()
