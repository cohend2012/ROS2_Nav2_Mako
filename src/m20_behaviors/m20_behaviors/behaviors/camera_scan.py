"""camera_scan — TIER_VENDOR behavior: find and center a target with the camera
by rotating the body (yaw sweep via /cmd_vel; no joint-level control).

Phase 5.5: the first perception-driven behavior through the full engine
pipeline. Detector is built in (red-oxide color mask on /camera/image_raw —
the wellhead's paint in the sim); a separate m20_perception detector node can
replace it later without changing the lifecycle.

Exit test (roadmap): from an arbitrary start yaw the robot rotates, detects,
and holds the target centered (|offset| < 5% width for ~1 s) -> SUCCEEDED.
abort() stops rotation immediately (robot stationary, re-navigable).

params_json (all optional): {"target_class": "wellhead"}
"""
import numpy as np

from m20_behaviors.base import Behavior, BehaviorSpec, Tier, Result

SWEEP_WZ = 0.45          # rad/s search rotation
K_CENTER = 1.2           # P-gain: normalized pixel offset -> wz
CENTER_TOL = 0.05        # |offset| below this counts as centered
CENTER_HOLD_S = 1.0      # must stay centered this long to succeed
MIN_PIXELS = 25          # smaller red blobs are noise


class CameraScan(Behavior):
    SPEC = BehaviorSpec(
        name="camera_scan", tier=Tier.VENDOR,
        description="Sweep body yaw to find and center the target in camera.",
        max_duration_s=60.0, min_battery_pct=15.0,
        tags=["perception", "inspection"])

    def precheck(self, params):
        return True, ""

    def start(self, params):
        from geometry_msgs.msg import Twist          # lazy: ROS-free unit tests
        from sensor_msgs.msg import Image
        self.target = params.get("target_class", "wellhead")
        self._Twist = Twist
        self._pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self._sub = self.node.create_subscription(Image, "/camera/image_raw",
                                                  self._on_image, 5)
        self._offset = None       # normalized [-0.5..0.5] x-offset; None = not seen
        self._held = 0.0
        self._seen_once = False

    def _on_image(self, msg):
        img = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        img = img.reshape(msg.height, msg.width, -1)[:, :, :3]
        r = img[:, :, 0].astype(np.int16)
        g = img[:, :, 1].astype(np.int16)
        b = img[:, :, 2].astype(np.int16)
        mask = (r > 90) & (r - g > 45) & (r - b > 45)   # red-oxide paint
        if int(mask.sum()) < MIN_PIXELS:
            self._offset = None
            return
        xs = np.nonzero(mask)[1]
        self._offset = float(xs.mean() / msg.width - 0.5)
        self._seen_once = True

    def step(self, dt):
        t = self._Twist()
        if self._offset is None:
            t.angular.z = SWEEP_WZ                   # searching
            self._held = 0.0
        elif abs(self._offset) > CENTER_TOL:
            t.angular.z = -K_CENTER * self._offset   # centering (P on pixel offset)
            self._held = 0.0
        else:
            t.angular.z = 0.0                        # centered: hold to confirm
            self._held += dt
            if self._held >= CENTER_HOLD_S:
                self._pub.publish(t)
                return Result.SUCCEEDED
        self._pub.publish(t)
        return Result.RUNNING

    def abort(self):
        self._pub.publish(self._Twist())             # stop; robot stays put

    def progress(self):
        if not self._seen_once:
            return 0.1
        if self._offset is None:
            return 0.4
        return max(0.4, 1.0 - abs(self._offset) * 2)
