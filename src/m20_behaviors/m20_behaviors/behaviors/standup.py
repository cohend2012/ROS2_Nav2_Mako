"""standup: scripted rise from the folded pose to standing.

Mirrors the vendor `standup_state` — a smooth position-trajectory interpolation
(NOT the RL policy), so it's TIER_VENDOR (no control-authority lease needed).
Interpolates every leg joint folded -> stand over ~3 s with stiff PD; wheels locked.
abort() simply stops commanding (the robot holds its last commanded pose).
"""
from m20_behaviors.base import Behavior, BehaviorSpec, Tier, Result

# low resting crouch (legs tucked, body ~0.27 m) and standing target (~0.46 m),
# per leg [hipx, hipy, knee, wheel]. Validated in sim: crouch rests stably, rises level.
FOLDED = [0.0, -1.0, 2.3, 0.0,  0.0, -1.0, 2.3, 0.0,
          0.0, 1.0, -2.3, 0.0,  0.0, 1.0, -2.3, 0.0]
STAND = [0.0, -0.7, 1.4, 0.0,  0.0, -0.7, 1.4, 0.0,
         0.0, 0.7, -1.4, 0.0,  0.0, 0.7, -1.4, 0.0]
WHEELS = (3, 7, 11, 15)
DURATION_S = 3.0


def _smoothstep(t: float) -> float:
    t = 0.0 if t < 0 else 1.0 if t > 1 else t
    return t * t * (3.0 - 2.0 * t)


class Standup(Behavior):
    SPEC = BehaviorSpec(
        name="standup", tier=Tier.VENDOR,
        description="Scripted smooth rise from folded to standing (vendor standup_state analog).",
        max_duration_s=6.0, requires_estimator=False, min_battery_pct=10.0,
        tags=["locomotion", "recovery"])

    def precheck(self, params: dict):
        return True, ""

    def start(self, params: dict):
        from drdds.msg import JointsDataCmd          # lazy: keeps drdds out of ROS-free tests
        self._Cmd = JointsDataCmd
        self._pub = self.node.create_publisher(JointsDataCmd, "/JOINTS_CMD", 10)
        self._t = 0.0

    def step(self, dt: float) -> Result:
        self._t += dt
        s = _smoothstep(self._t / DURATION_S)
        m = self._Cmd()
        m.header.stamp = self.node.get_clock().now().to_msg()
        for i in range(16):
            j = m.data.joints_data[i]
            if i in WHEELS:
                j.kp, j.kd, j.velocity = 0.0, 1.0, 0.0      # wheels locked/damped
            else:
                j.kp, j.kd = 200.0, 4.0
                j.position = FOLDED[i] + s * (STAND[i] - FOLDED[i])
        self._pub.publish(m)
        return Result.SUCCEEDED if self._t >= DURATION_S else Result.RUNNING

    def abort(self) -> None:
        pass  # holds last commanded pose (near-standing → vendor-recoverable)

    def progress(self) -> float:
        return min(1.0, self._t / DURATION_S)
