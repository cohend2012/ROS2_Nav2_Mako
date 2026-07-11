"""jump_pipe — TIER_POLICY behavior: dynamic jump over a pipe-class obstacle.

The flagship novel behavior and the hardest: flight phase + landing under a
trained jump policy. Gate progression: sim success rate > 95% across randomized
pipe sizes BEFORE any hardware attempt (Phase 8 exit test), crash-rail rig for
first hardware trials.

params_json: {"pipe_height_m": 0.3, "approach_speed": 1.5}
"""
from m20_behaviors.base import Behavior, BehaviorSpec, Tier, Result


class JumpPipe(Behavior):
    SPEC = BehaviorSpec(
        name="jump_pipe", tier=Tier.POLICY,
        description="Jump over a detected pipe obstacle using a trained jump policy.",
        max_duration_s=10.0, min_battery_pct=40.0,
        tags=["dynamic", "rl_policy", "obstacle"])

    def precheck(self, params):
        return False, ("TODO Phase 8: verify pipe detected + measured by perception, "
                       "flat approach/landing zones, battery margin, policy loaded")

    def start(self, params):
        pass  # TODO: align approach, load policy, request authority already leased

    def step(self, dt):
        return Result.FAILED  # TODO: approach -> launch -> flight -> landing -> settle

    def abort(self):
        pass  # TODO: if pre-launch, brake; if airborne, execute landing branch (never freeze)
