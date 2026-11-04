"""NavCore: the ROS-free decision loop shared by every integration mode.

    tick(pose, scan_bins, measured_vel) -> Command(vx, wz, state, detail)

States:  TRACKING -> ALIGNING (within xy tolerance, turn to goal yaw) -> SUCCEEDED
         any -> BLOCKED (safety envelope tripped; zero command, keep trying)
         any -> STUCK   (no progress for stuck_timeout_s -> caller aborts, so Nav2's
                         BT / the mission server can run its own recovery)

Safety envelope (not learnable, not configurable off): if any return inside the
forward cone is closer than stop_range, forward velocity is clamped to <= 0. The
policy may still turn or reverse out. This sits UNDER the policy and ABOVE the
bridge watchdog and commander veto — three independent layers.
"""
import math
from dataclasses import dataclass

import numpy as np

from m20_ai_nav import spec
from m20_ai_nav.geometry import CarrotTracker, to_body, wrap

TRACKING, ALIGNING, SUCCEEDED, BLOCKED, STUCK = (
    "TRACKING", "ALIGNING", "SUCCEEDED", "BLOCKED", "STUCK")


@dataclass
class Command:
    vx: float
    wz: float
    state: str
    detail: str = ""


class NavCore:
    def __init__(self, policy, lookahead=1.5, xy_tol=0.20, yaw_tol=0.10,
                 stop_range=0.60, front_half_angle=math.radians(35),
                 stuck_timeout_s=12.0, stuck_min_progress=0.25, align_kp=1.5):
        self.policy = policy
        self.tracker = CarrotTracker(lookahead)
        self.xy_tol, self.yaw_tol = xy_tol, yaw_tol
        self.stop_range, self.front = stop_range, front_half_angle
        self.stuck_timeout, self.stuck_prog = stuck_timeout_s, stuck_min_progress
        self.align_kp = align_kp
        self.goal_yaw = None
        self.reset()

    # ---------- goal management ----------
    def reset(self):
        self.mem = []              # [(t, world_xy[N,2])] short-term obstacle memory
        self.prev_bins = None
        self.last_action = np.zeros(2, np.float32)
        self._best = math.inf
        self._best_t = None
        self.aligning = False

    def set_path(self, xy, goal_yaw=None):
        """xy: list of (x, y) in the global frame; last point is the goal."""
        self.tracker.set_path(xy)
        self.goal_yaw = goal_yaw
        self.reset()

    def update_path(self, xy):
        """Replan without resetting progress bookkeeping (standalone 1 Hz replans)."""
        self.tracker.set_path(xy)

    # ---------- the loop ----------
    def tick(self, t, x, y, yaw, scan_bins, vx_meas=0.0, wz_meas=0.0):
        bins = self._with_memory(t, x, y, yaw, np.asarray(scan_bins, np.float32))
        if self.prev_bins is None:
            self.prev_bins = bins
        gx, gy = self.tracker.path[-1]
        dist_goal = math.hypot(gx - x, gy - y)
        # progress / stuck bookkeeping on remaining path length
        rem = self.tracker.remaining(x, y)
        if self._best_t is None or rem < self._best - self.stuck_prog:
            self._best, self._best_t = rem, t
        elif t - self._best_t > self.stuck_timeout and not self.aligning:
            return Command(0.0, 0.0, STUCK, f"no {self.stuck_prog} m progress in "
                                            f"{self.stuck_timeout:.0f} s ({rem:.2f} m left)")
        # final approach: xy reached -> rotate in place to the goal heading
        if dist_goal < self.xy_tol or self.aligning:
            self.aligning = True
            if self.goal_yaw is None:
                return Command(0.0, 0.0, SUCCEEDED, f"xy {dist_goal:.2f} m")
            e = wrap(self.goal_yaw - yaw)
            if abs(e) < self.yaw_tol:
                return Command(0.0, 0.0, SUCCEEDED, f"xy {dist_goal:.2f} m, yaw {e:+.2f}")
            wz = max(-spec.WZ_MAX, min(spec.WZ_MAX, self.align_kp * e))
            if abs(wz) < 0.2:
                wz = math.copysign(0.2, e)        # bridge dead-zone boost needs >= 0.15
            return Command(0.0, wz, ALIGNING, f"yaw err {e:+.2f}")
        cx, cy = self.tracker.carrot(x, y)
        bx, by = to_body(cx, cy, x, y, yaw)
        obs = spec.build_obs(bins, self.prev_bins, bx, by, self.last_action, vx_meas, wz_meas)
        a = np.asarray(self.policy(obs), np.float32).reshape(2)
        self.prev_bins = bins
        self.last_action = a
        vx, wz = spec.action_to_cmd(a)
        state, detail = TRACKING, f"carrot ({bx:+.2f},{by:+.2f}) goal {dist_goal:.2f} m"
        if vx > 0.0 and self._front_min(bins) < self.stop_range:
            vx, state, detail = 0.0, BLOCKED, f"front {self._front_min(bins):.2f} m < stop_range"
        return Command(vx, wz, state, detail)

    def _with_memory(self, t, x, y, yaw, bins):
        """Blind-zone memory. The LiDAR returns NOTHING inside SCAN_MIN (0.5 m) and
        the M20 body reaches 0.43-0.50 m, so an obstacle the robot creeps up to (or
        pivots beside) turns into "clear" exactly when it matters (collision in the
        sim mission, 2026-11-04). Near returns (< MEM_RANGE) are kept in the WORLD
        frame for MEM_S seconds, re-projected into the current body-frame bins, and
        min-pooled in, clamped to SCAN_MIN (= "as close as we can know")."""
        n = len(bins)
        w = 2 * math.pi / n
        near = bins < self.MEM_RANGE
        if near.any():
            k = np.nonzero(near)[0]
            ang = yaw + k * w
            r = bins[k].astype(float)
            self.mem.append((t, np.stack([x + r * np.cos(ang), y + r * np.sin(ang)], 1)))
        self.mem = [(tt, p) for tt, p in self.mem if t - tt <= self.MEM_S]
        if not self.mem:
            return bins
        pts = np.concatenate([p for _, p in self.mem])
        dx, dy = pts[:, 0] - x, pts[:, 1] - y
        d = np.hypot(dx, dy)
        keep = d < self.MEM_RANGE
        if not keep.any():
            return bins
        a = np.mod(np.arctan2(dy[keep], dx[keep]) - yaw + w / 2, 2 * math.pi)
        idx = (a // w).astype(int) % n
        out = bins.copy()
        np.minimum.at(out, idx, np.clip(d[keep], spec.SCAN_MIN, spec.SCAN_MAX).astype(np.float32))
        return out

    MEM_RANGE = 1.5     # m: only near returns matter for the blind zone
    MEM_S = 3.0         # s: long enough to pivot past an obstacle at 0.5 rad/s

    def _front_min(self, bins):
        n = len(bins)
        k = int(math.ceil(self.front / (2 * math.pi / n)))
        idx = [i % n for i in range(-k, k + 1)]
        return float(np.min(np.asarray(bins)[idx]))
