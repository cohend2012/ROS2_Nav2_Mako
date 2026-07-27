"""m20_behaviors engine: registry + executor for behavior plugins.

Auto-discovers Behavior subclasses in the behaviors/ package, executes one at a
time, enforces the lifecycle, and — for TIER_POLICY behaviors — obtains and
renews a control-authority lease from Commander before any motion.

Safety invariants enforced HERE so individual behaviors can't skip them:
  * one behavior at a time; new requests rejected while one is RUNNING
  * hard timeout at SPEC.max_duration_s -> abort()
  * mode/failsafe change from Commander -> abort()
  * TIER_POLICY without a granted lease -> never starts
"""
import importlib
import json
import pkgutil

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy,
                       HistoryPolicy)

from m20_msgs.msg import (BehaviorRequest, BehaviorStatus, RobotMode,
                          FailsafeStatus, HealthReport, ControlAuthority)
from m20_msgs.srv import SetControlAuthority

from m20_behaviors.base import Behavior, Tier, Result
from m20_behaviors import behaviors as behaviors_pkg

TICK_HZ = 50.0


class BehaviorEngine(Node):
    def __init__(self):
        super().__init__("m20_behavior_engine")
        self.registry = self._discover()
        self.active: Behavior | None = None
        self.active_elapsed = 0.0
        self.mode_ok = False
        self.failsafe_clear = True

        self.create_subscription(BehaviorRequest, "/m20/behavior/request", self.on_request, 10)
        # /m20/mode is LATCHED (transient_local) and published on change — a volatile
        # subscriber that starts after the last mode change never learns the mode.
        # (Bit us 2026-07-20: standup denied with the commander armed in ASSISTED.)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(RobotMode, "/m20/mode", self.on_mode, latched)
        self.create_subscription(FailsafeStatus, "/m20/failsafe", self.on_failsafe, 10)
        self.status_pub = self.create_publisher(BehaviorStatus, "/m20/behavior/status", 10)
        self.health_pub = self.create_publisher(HealthReport, "/m20/health", 10)
        self.authority_cli = self.create_client(SetControlAuthority, "/m20/set_control_authority")

        self.create_timer(1.0 / TICK_HZ, self.tick)
        self.create_timer(1.0, self.heartbeat)
        self.get_logger().info(f"Behavior engine up. Registered: {sorted(self.registry)}")

    # ---------- discovery ----------
    def _discover(self) -> dict[str, type[Behavior]]:
        found = {}
        for m in pkgutil.iter_modules(behaviors_pkg.__path__):
            mod = importlib.import_module(f"{behaviors_pkg.__name__}.{m.name}")
            for obj in vars(mod).values():
                if (isinstance(obj, type) and issubclass(obj, Behavior)
                        and obj is not Behavior and obj.SPEC is not None):
                    found[obj.SPEC.name] = obj
        return found

    # ---------- inputs ----------
    def on_mode(self, msg: RobotMode):
        self.mode_ok = msg.armed and msg.mode in (RobotMode.MODE_ASSISTED,
                                                  RobotMode.MODE_AUTONOMOUS)
        if not self.mode_ok:
            self._abort("mode change / disarm")

    def on_failsafe(self, msg: FailsafeStatus):
        self.failsafe_clear = msg.active == 0
        if not self.failsafe_clear:
            self._abort(f"failsafe 0x{msg.active:x}")

    def on_request(self, req: BehaviorRequest):
        if self.active is not None:
            self._status(req.behavior_name, BehaviorStatus.STATE_FAILED,
                         "engine busy: one behavior at a time")
            return
        cls = self.registry.get(req.behavior_name)
        if cls is None:
            self._status(req.behavior_name, BehaviorStatus.STATE_FAILED,
                         f"unknown behavior (known: {sorted(self.registry)})")
            return
        if not (self.mode_ok and self.failsafe_clear):
            self._status(req.behavior_name, BehaviorStatus.STATE_FAILED,
                         "denied: mode not armed/assisted or failsafe active")
            return
        params = json.loads(req.params_json) if req.params_json else {}
        beh = cls(self)
        self._status(beh.SPEC.name, BehaviorStatus.STATE_PRECHECK, "")
        ok, reason = beh.precheck(params)
        if not ok:
            self._status(beh.SPEC.name, BehaviorStatus.STATE_FAILED, f"precheck: {reason}")
            return
        if beh.SPEC.tier == Tier.POLICY and not self._acquire_lease(beh):
            self._status(beh.SPEC.name, BehaviorStatus.STATE_FAILED,
                         "control authority lease denied by commander")
            return
        beh.start(params)
        self.active, self.active_elapsed = beh, 0.0
        self._status(beh.SPEC.name, BehaviorStatus.STATE_RUNNING, "")

    def _acquire_lease(self, beh: Behavior) -> bool:
        if not self.authority_cli.service_is_ready():
            return False
        req = SetControlAuthority.Request()
        req.tier = ControlAuthority.TIER_POLICY
        req.requester = f"{self.get_name()}/{beh.SPEC.name}"
        req.lease_ms = int(beh.SPEC.max_duration_s * 1000)
        future = self.authority_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        return bool(future.done() and future.result() and future.result().granted)

    # ---------- lifecycle ----------
    def tick(self):
        if self.active is None:
            return
        dt = 1.0 / TICK_HZ
        self.active_elapsed += dt
        if self.active_elapsed > self.active.SPEC.max_duration_s:
            self._abort("max duration exceeded")
            return
        result = self.active.step(dt)
        if result == Result.RUNNING:
            return
        state = (BehaviorStatus.STATE_SUCCEEDED if result == Result.SUCCEEDED
                 else BehaviorStatus.STATE_FAILED)
        self._finish(state, "")

    def _abort(self, reason: str):
        if self.active is None:
            return
        try:
            self.active.abort()
        finally:
            self._finish(BehaviorStatus.STATE_ABORTED, reason)

    def _finish(self, state: int, detail: str):
        name = self.active.SPEC.name
        self.active = None
        self._status(name, state, detail)
        # TODO(ADR-008): release authority lease via SetControlAuthority(TIER_VENDOR)

    # ---------- output ----------
    def _status(self, name: str, state: int, detail: str):
        s = BehaviorStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.behavior_name = name
        s.state = state
        s.progress = self.active.progress() if self.active else 0.0
        s.detail = detail
        self.status_pub.publish(s)

    def heartbeat(self):
        h = HealthReport()
        h.header.stamp = self.get_clock().now().to_msg()
        h.node_name = self.get_name()
        h.status = HealthReport.STATUS_OK
        h.message = f"active={self.active.SPEC.name if self.active else 'none'}"
        self.health_pub.publish(h)


def main():
    rclpy.init()
    rclpy.spin(BehaviorEngine())


if __name__ == "__main__":
    main()
