"""The policy contract: observation layout, action scaling, limits.

ONE source of truth shared by the training env (sim/env.py), the ONNX runtime
(policy.py) and the ROS node (controller_node.py). If you change anything here the
shipped model is invalid — bump OBS_VERSION and retrain (tests enforce the size).
"""
import math

import numpy as np

OBS_VERSION = 1

# --- action: normalized [-1, 1]^2 -> body velocity --------------------------------
# Limits = the existing velocity_smoother bounds (nav2_params.yaml), so the policy can
# never request anything Nav2 could not already request.
VX_MIN, VX_MAX = -0.2, 0.4      # m/s
WZ_MAX = 0.7                    # rad/s
CONTROL_HZ = 10.0

# --- observation --------------------------------------------------------------------
N_BINS = 72                     # 5 deg bins, bin 0 = straight ahead, CCW positive
SCAN_MAX = 5.0                  # m; ranges clipped here (local navigation horizon)
SCAN_MIN = 0.5                  # m; LiDAR blind radius (vendor config)
GOAL_MAX = 6.0                  # m; goal distance clip for normalization
N_EXTRA = 3 + 2 + 2             # goal(d, sin, cos) + last action(2) + measured vel(2)
OBS_DIM = 2 * N_BINS + N_EXTRA  # current + previous scan (motion cue) + extras


def action_to_cmd(a):
    """Normalized action in [-1,1]^2 -> (vx, wz). Asymmetric vx range."""
    a0 = float(np.clip(a[0], -1.0, 1.0))
    a1 = float(np.clip(a[1], -1.0, 1.0))
    vx = VX_MAX * a0 if a0 >= 0.0 else -VX_MIN * a0
    return vx, WZ_MAX * a1


def cmd_to_action(vx, wz):
    a0 = vx / VX_MAX if vx >= 0.0 else vx / -VX_MIN
    return np.array([np.clip(a0, -1, 1), np.clip(wz / WZ_MAX, -1, 1)], np.float32)


def bin_scan(ranges, angle_min, angle_increment, n_bins=N_BINS):
    """Min-pool an arbitrary LaserScan (any resolution, any angle_min) into N_BINS
    body-frame bins, bin k covering [k*w - w/2, k*w + w/2), w = 2pi/n_bins.
    inf/nan/<=SCAN_MIN are treated as 'no return' -> SCAN_MAX (like a clear ray).
    Returns float32 ranges in metres, clipped to [SCAN_MIN, SCAN_MAX]."""
    r = np.asarray(ranges, np.float64)
    if r.size == 0:
        raise ValueError("bin_scan: empty scan (refusing to fabricate a clear view)")
    ang = angle_min + angle_increment * np.arange(r.size)
    w = 2.0 * math.pi / n_bins
    idx = np.floor(np.mod(ang + w / 2.0, 2.0 * math.pi) / w).astype(int) % n_bins
    good = np.isfinite(r) & (r > SCAN_MIN)
    out = np.full(n_bins, SCAN_MAX, np.float64)
    np.minimum.at(out, idx[good], np.clip(r[good], SCAN_MIN, SCAN_MAX))
    return out.astype(np.float32)


def goal_features(gx_body, gy_body):
    d = math.hypot(gx_body, gy_body)
    th = math.atan2(gy_body, gx_body)
    return np.array([min(d, GOAL_MAX) / GOAL_MAX, math.sin(th), math.cos(th)], np.float32)


def build_obs(scan_bins, prev_scan_bins, gx_body, gy_body, last_action, vx_meas, wz_meas):
    """Assemble the flat observation. Scans are normalized to [0,1] (1 = far)."""
    s = np.asarray(scan_bins, np.float32) / SCAN_MAX
    p = np.asarray(prev_scan_bins, np.float32) / SCAN_MAX
    if s.shape != (N_BINS,) or p.shape != (N_BINS,):
        raise ValueError(f"scan bins must be ({N_BINS},), got {s.shape}/{p.shape}")
    extra = np.concatenate([
        goal_features(gx_body, gy_body),
        np.asarray(last_action, np.float32).reshape(2),
        np.array([vx_meas / VX_MAX, wz_meas / WZ_MAX], np.float32)])
    obs = np.concatenate([s, p, extra]).astype(np.float32)
    assert obs.shape == (OBS_DIM,)
    return obs
