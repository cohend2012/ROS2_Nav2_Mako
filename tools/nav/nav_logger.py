#!/usr/bin/env python3
"""Log the Nav2 run for honest verification (with drifting odometry):
  - TRUE trajectory        (/odom_true)         -> traj_true.csv   (ground truth)
  - RAW dead-reckoned odom (/odom)              -> traj_drift.csv  (drifts, uncorrected)
  - SLAM-corrected estimate (TF map->base_link) -> traj_est.csv    (what Nav2 uses)
  - GPS fixes              (/gps)               -> gps.csv         (noisy absolutes)
  - GPS-fused EKF estimate (/odom_filtered)     -> traj_ekf.csv    (if estimator runs)
  - Nav2 global plan       (/plan)              -> plan.csv
  - occupancy map          (/map)               -> map.npz
The gap TRUE vs RAW = accumulated drift; TRUE vs EST staying small = SLAM working;
TRUE vs EKF bounded = GPS doing its job (Phase A preview).
"""
import os, time, csv, math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from tf2_ros import Buffer, TransformListener

OUT = "/cfg/out"
RUN_SECS = float(os.environ.get("RUN_SECS", "120"))

class Logger(Node):
    def __init__(self):
        super().__init__("nav_logger")
        self.true = []; self.drift = []; self.est = []; self.gps = []; self.ekf = []
        self.plan = None; self.map = None
        self.create_subscription(Odometry, "/odom_true", self.on_true, 10)
        self.create_subscription(Odometry, "/odom", self.on_drift, 10)
        self.create_subscription(Odometry, "/gps",
            lambda m: self.gps.append((time.time(), m.pose.pose.position.x, m.pose.pose.position.y)), 10)
        self.create_subscription(Odometry, "/odom_filtered",
            lambda m: self.ekf.append((time.time(), m.pose.pose.position.x, m.pose.pose.position.y)), 10)
        self.create_subscription(Path, "/plan", self.on_plan, 10)
        mqos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, mqos)
        self.tfbuf = Buffer(); self.tfl = TransformListener(self.tfbuf, self)
    def on_true(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.true.append((time.time(), m.pose.pose.position.x, m.pose.pose.position.y, yaw))
    def on_drift(self, m):
        self.drift.append((time.time(), m.pose.pose.position.x, m.pose.pose.position.y))
    def on_plan(self, m):
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
    def on_map(self, m):
        self.map = (m.info.resolution, m.info.width, m.info.height,
                    m.info.origin.position.x, m.info.origin.position.y,
                    np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width))
    def sample_est(self):
        try:
            t = self.tfbuf.lookup_transform("map", "base_link", rclpy.time.Time())
            self.est.append((time.time(), t.transform.translation.x, t.transform.translation.y))
        except Exception:
            pass

def main():
    os.makedirs(OUT, exist_ok=True)
    rclpy.init(); n = Logger()
    t0 = time.time(); last = 0.0; last_dump = time.time()

    def dump(name, rows, hdr):
        with open(f"{OUT}/{name}", "w", newline="") as f:
            w = csv.writer(f); w.writerow(hdr); w.writerows(rows)

    def dump_all():
        dump("traj_true.csv", n.true, ["t", "x", "y", "yaw"])
        dump("traj_drift.csv", n.drift, ["t", "x", "y"])
        dump("traj_est.csv", n.est, ["t", "x", "y"])
        dump("gps.csv", n.gps, ["t", "x", "y"])
        dump("traj_ekf.csv", n.ekf, ["t", "x", "y"])

    while rclpy.ok() and time.time() - t0 < RUN_SECS:
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() - last > 0.1:
            n.sample_est(); last = time.time()
        # incremental dump: observers copy CSVs while we run — end-only writes made
        # every mid-run copy read the PREVIOUS run (the "blind logger" of session 4)
        if time.time() - last_dump > 10.0:
            dump_all(); last_dump = time.time()
    dump("traj_true.csv", n.true, ["t", "x", "y", "yaw"])
    dump("traj_drift.csv", n.drift, ["t", "x", "y"])
    dump("traj_est.csv", n.est, ["t", "x", "y"])
    dump("gps.csv", n.gps, ["t", "x", "y"])
    dump("traj_ekf.csv", n.ekf, ["t", "x", "y"])
    if n.plan:
        dump("plan.csv", n.plan, ["x", "y"])
    if n.map is not None:
        res, wdt, hgt, ox, oy, grid = n.map
        np.savez(f"{OUT}/map.npz", res=res, width=wdt, height=hgt, ox=ox, oy=oy, grid=grid)
    print(f"true={len(n.true)} drift={len(n.drift)} est={len(n.est)} "
          f"gps={len(n.gps)} ekf={len(n.ekf)} "
          f"plan={len(n.plan) if n.plan else 0} map={'y' if n.map is not None else 'n'}")
    n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
