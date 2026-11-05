"""Velocity -> wheel mixer used by the TRAINING env.

This is a ROS-free mirror of m20_locomotion_bridge.bridge_node.SimVendorSDK
.send_velocity (closed-loop yaw trim + skid-scrub pivot boost). The policy must be
trained against the SAME plant it will drive in the stack, so parity is enforced by
tests/test_ai_nav.py::test_mixer_matches_bridge — if you retune the bridge, that
test fails until this file follows.

Also mirrors the velocity_smoother accel limits (nav2_params.yaml), which sit
between any controller and the bridge.
"""
import math

R_WHEEL, TRACK = 0.072, 0.40     # calibrated effective radius / track (bridge defaults)
KP_YAW, KI_YAW = 3.0, 0.1
YAW_TRIM_MAX = 0.6     # bridge parity: a stalled turn must not wind the trim up (tipped the robot)


class SkidSteerMixer:
    def __init__(self, wheel_radius=R_WHEEL, track=TRACK):
        self.R, self.B = wheel_radius, track
        self.yint = 0.0

    def reset(self):
        self.yint = 0.0

    def wheels(self, vx, wz, omega_meas, dt):
        """Returns (left, right) wheel velocity setpoints in rad/s."""
        yerr = wz - omega_meas
        corr = KP_YAW * yerr + KI_YAW * self.yint       # bounded trim + anti-windup
        if abs(corr) < YAW_TRIM_MAX:
            self.yint = max(-2.0, min(2.0, self.yint + yerr * dt))
        w_cmd = wz + max(-YAW_TRIM_MAX, min(YAW_TRIM_MAX, corr))
        if abs(vx) < 0.05 and abs(wz) >= 0.15 and abs(w_cmd) < 0.5:
            w_cmd = math.copysign(0.5, w_cmd if w_cmd != 0.0 else wz)
        vl = (-vx + w_cmd * self.B / 2.0) / self.R
        vr = (-vx - w_cmd * self.B / 2.0) / self.R
        return vl, vr


class AccelLimiter:
    """velocity_smoother OPEN_LOOP equivalent (max_accel/max_decel per axis)."""

    def __init__(self, acc=(1.0, 4.0), dec=(1.5, 4.0)):
        self.acc, self.dec = acc, dec
        self.v = [0.0, 0.0]

    def reset(self):
        self.v = [0.0, 0.0]

    def step(self, target, dt):
        out = []
        for i, t in enumerate(target):
            cur = self.v[i]
            # accelerating = moving away from zero in the same direction
            lim = self.acc[i] if (t * cur >= 0 and abs(t) > abs(cur)) else self.dec[i]
            d = max(-lim * dt, min(lim * dt, t - cur))
            out.append(cur + d)
        self.v = out
        return tuple(out)
