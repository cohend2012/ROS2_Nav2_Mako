"""three_wheel — TIER_POLICY behavior: locomotion on three wheels with one leg
raised (injured-wheel continuation, or raising a leg to point/carry).

Likely trainable as a variant in rl_training with one wheel masked. Also the
degraded-mode building block: if a wheel actuator fails in the field, Commander
can invoke this instead of declaring the mission dead (link to FS_MODULE_UNHEALTHY).

params_json: {"raised_leg": "FR", "max_speed": 0.8}
"""
from m20_behaviors.base import Behavior, BehaviorSpec, Tier, Result


class ThreeWheel(Behavior):
    SPEC = BehaviorSpec(
        name="three_wheel", tier=Tier.POLICY,
        description="Stable driving on three wheels with a designated leg raised.",
        max_duration_s=300.0, min_battery_pct=25.0,
        tags=["dynamic", "rl_policy", "degraded_mode"])

    def precheck(self, params):
        return False, "TODO Phase 8: valid leg id, policy loaded, flat-ish terrain"

    def start(self, params):
        pass  # TODO: transition sequence stand -> weight shift -> leg raise -> tripod drive

    def step(self, dt):
        return Result.FAILED  # TODO: RUNNING until requested distance/duration met

    def abort(self):
        pass  # TODO: lower leg, return to four-wheel stance, hand back to vendor
