#!/usr/bin/env python3
"""L1.7 — verify the sim LiDAR realism is real, not cosmetic (TEST_PLAN.md).

Run with the stack up (sim + bridge + commander armed), inside the container:
    python3 /cfg/lidar_realism_check.py

  L1.7a  XYZIRT contract: fields/offsets/point_step, ring range, intensity range,
         per-point timestamps progressive and spanning ~one 100 ms sweep.
  L1.7b  Motion skew is REAL: fit a line to the pipe_cross cluster (a straight
         feature); parked residual ~ sensor noise, spinning at 1 rad/s the sweep
         smear must grow the residual (>2x parked and >3 cm).
  L1.7c  Noise model: ground-ring range residual sigma in [0.5, 3] cm; ground-ring
         dropout fraction in [0.5, 12] %.

Scores against known scene geometry only (pipe_cross at x=2, flat ground); consumes
/LIDAR/POINTS exactly as a real consumer would.
"""
import math, time, sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist

DTYPE = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "timestamp"],
                  "formats": [np.float32, np.float32, np.float32, np.float32,
                              np.uint16, np.float64],
                  "offsets": [0, 4, 8, 12, 16, 24], "itemsize": 32})
RESULTS = []


def check(tag, ok, detail):
    RESULTS.append(ok)
    print(f"  [{'PASS' if ok else 'FAIL'}] {tag}: {detail}")


def parse(msg):
    return np.frombuffer(msg.data, dtype=DTYPE, count=msg.width)


def pipe_cluster(a):
    """Points on the pipe_cross segment (x=2 world; robot spawns ~origin, yaw~0):
    low metal returns in a box around it, excluding the y=-3 pipe runs."""
    m = ((np.abs(a["x"] - 2.0) < 0.6) & (np.abs(a["y"]) < 2.0) &
         (a["z"] < -0.15) & (a["intensity"] > 60))
    return a[m]


def line_rms(a):
    """Fit x = a*y + b to the cluster; return RMS residual (m)."""
    if a.shape[0] < 12:
        return None
    c = np.polyfit(a["y"].astype(float), a["x"].astype(float), 1)
    res = a["x"] - np.polyval(c, a["y"])
    return float(np.sqrt(np.mean(res ** 2)))


class Chk(Node):
    def __init__(self):
        super().__init__("lidar_realism_check")
        self.frames = []
        self.create_subscription(PointCloud2, "/LIDAR/POINTS", self.on_cloud, 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)

    def on_cloud(self, m):
        self.frames.append(m)
        if len(self.frames) > 6:
            self.frames.pop(0)

    def wait_frames(self, n, timeout=8.0):
        self.frames.clear()
        t0 = time.time()
        while len(self.frames) < n and time.time() - t0 < timeout and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        return list(self.frames)

    def drive(self, wz, secs):
        t0 = time.time()
        while time.time() - t0 < secs and rclpy.ok():
            t = Twist()
            t.angular.z = float(wz)
            self.cmd.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.04)


def main():
    rclpy.init()
    n = Chk()
    print("== L1.7a: XYZIRT contract ==")
    frames = n.wait_frames(3)
    if not frames:
        print("FAIL: no /LIDAR/POINTS frames — is the stack up with M20_LIDAR_REALISM=1?")
        sys.exit(1)
    msg = frames[-1]
    names = [(f.name, f.offset, f.datatype) for f in msg.fields]
    check("fields", names == [("x", 0, 7), ("y", 4, 7), ("z", 8, 7),
                              ("intensity", 12, 7), ("ring", 16, 4), ("timestamp", 24, 8)]
          and msg.point_step == 32, f"{names} step={msg.point_step}")
    a = parse(msg)
    check("ring range", a["ring"].min() >= 0 and a["ring"].max() == 15,
          f"rings {a['ring'].min()}..{a['ring'].max()}")
    check("intensity range", 0 < a["intensity"].min() and a["intensity"].max() <= 255
          and a["intensity"].max() > 100,
          f"{a['intensity'].min():.0f}..{a['intensity'].max():.0f} (max>100 = metal/tank seen)")
    ts = a["timestamp"]
    span = ts.max() - ts.min()
    mono = bool(np.all(np.diff(ts) >= -1e-9))
    check("timestamps", 0.085 <= span <= 0.115 and len(np.unique(ts)) >= 15 and mono,
          f"span={span*1000:.1f} ms, {len(np.unique(ts))} steps, monotone={mono}")

    print("== L1.7c: noise + dropout (parked, ground ring 15) ==")
    g = a[(a["ring"] == 15) & (a["intensity"] < 60)]
    rng_res = np.sqrt(g["x"].astype(float) ** 2 + g["y"] ** 2 + g["z"] ** 2)
    sigma = float(np.std(rng_res))
    check("range noise sigma", 0.005 <= sigma <= 0.03, f"{sigma*100:.2f} cm-class ({sigma:.4f} m)")
    drop = 1.0 - g.shape[0] / 120.0
    check("ground dropout", 0.005 <= drop <= 0.12, f"{drop*100:.1f}% of 120 azimuths missing")

    print("== L1.7b: motion skew (line-fit residual, parked vs spinning) ==")
    still = [line_rms(pipe_cluster(parse(f))) for f in frames]
    still = [r for r in still if r is not None]
    n.drive(1.0, 2.5)                       # spin up to steady 1 rad/s
    spin_frames = n.wait_frames(3)
    n.drive(0.0, 0.8)
    spin = [line_rms(pipe_cluster(parse(f))) for f in spin_frames]
    spin = [r for r in spin if r is not None]
    if not still or not spin:
        check("skew", False, f"cluster too small (still={len(still)}, spin={len(spin)} fits) "
              "— run from the default spawn pose facing the field")
    else:
        s0, s1 = float(np.median(still)), float(np.median(spin))
        check("skew", s1 > 2.0 * s0 and s1 > 0.03,
              f"parked RMS {s0*100:.1f} cm -> spinning RMS {s1*100:.1f} cm "
              f"(x{s1/max(s0,1e-6):.1f}; smear from 100 ms sweep is real)")

    ok = all(RESULTS)
    print(f"\nL1.7 {'PASS' if ok else 'FAIL'} ({sum(RESULTS)}/{len(RESULTS)})")
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
