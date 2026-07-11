"""self_right — TIER_POLICY behavior: recover from a fall/rollover.

Requires a trained recovery policy (Deep Robotics rl_training / Isaac Lab,
exported to ONNX) streamed as joint targets through the bridge's low-level
channel under a Commander authority lease. Highest-value behavior: it converts
'a fall ends the mission' into 'a fall costs 10 seconds'.

Precheck notes: verify robot IS actually overturned (IMU gravity vector),
sufficient clearance, battery margin. Vendor may ship built-in recovery —
check OQ-9 before investing in a custom policy.
"""
from m20_behaviors.base import Behavior, BehaviorSpec, Tier, Result


class SelfRight(Behavior):
    SPEC = BehaviorSpec(
        name="self_right", tier=Tier.POLICY,
        description="Roll from overturned to standing via trained recovery policy.",
        max_duration_s=15.0, min_battery_pct=25.0, requires_estimator=False,
        tags=["recovery", "rl_policy"])

    def precheck(self, params):
        return False, "TODO: check IMU says overturned; policy model loaded (Phase 8)"

    def start(self, params):
        pass  # TODO: load ONNX policy, start 50 Hz joint-target streaming

    def step(self, dt):
        return Result.FAILED  # TODO: SUCCEEDED when upright + vendor controller re-engaged

    def abort(self):
        pass  # TODO: damp joints to safe compliance, hand back to vendor controller
