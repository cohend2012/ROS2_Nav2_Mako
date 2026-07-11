"""camera_scan — TIER_VENDOR behavior: aim the camera at a target by moving the
robot body (yaw sweep + body-pitch commands), driven by detections from
m20_perception. The first behavior to build: no joint-level control needed, so
it exercises the whole engine pipeline safely in Phase 5.5.

params_json: {"target_class": "valve", "sector_deg": 180, "settle_s": 2.0}
"""
from m20_behaviors.base import Behavior, BehaviorSpec, Tier, Result


class CameraScan(Behavior):
    SPEC = BehaviorSpec(
        name="camera_scan", tier=Tier.VENDOR,
        description="Sweep body yaw to find and center a detected object class.",
        max_duration_s=60.0, min_battery_pct=15.0,
        tags=["perception", "inspection"])

    def precheck(self, params):
        if "target_class" not in params:
            return False, "missing target_class"
        return True, ""  # TODO(Phase 5.5): verify detector node healthy

    def start(self, params):
        self.target = params["target_class"]
        # TODO: subscribe /m20/detections; publish slow yaw cmd_vel sweep;
        # switch to centering controller when target detected; then body-pitch
        # via vendor body-pose command (OQ-8) to center vertically.

    def step(self, dt):
        return Result.RUNNING  # TODO: SUCCEEDED once target centered + settled

    def abort(self):
        pass  # TODO: publish zero cmd_vel, neutral body pose
