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
        self.joints = []
        self.plan = None; self.map = None
        # joint positions (for replay-rendering the standup segment), decimated
        # 200 Hz -> 20 Hz; drdds is only in the container image, so keep the
        # logger runnable without it
        self._last_joints = 0.0
        try:
            from drdds.msg import JointsData
            self.create_subscription(JointsData, "/JOINTS_DATA", self.on_joints, 10)
        except ImportError:
            pass
        self.create_subscription(Odometry, "/odom_true", self.on_true, 10)
        self.create_subscription(Odometry, "/odom", self.on_drift, 10)
        self.create_subscription(Odometry, "/gps",
            lambda m: self.gps.append((time.time(), m.pose.pose.position.x, m.pose.pose.position.y)), 10)
        self.create_subscription(Odometry, "/odom_filtered",
            lambda m: self.ekf.append((time.time(), m.pose.pose.position.x, m.pose.pose.position.y)), 10)
        self.create_subscription(Path, "/plan", self.on_plan, 10)
        # planning-view capture (for the nav-view video panel): laser + local plan
        self.scans = []          # (t, x, y, yaw, r0..r359) at ~1 Hz, pose from TF
        self.local_plans = []    # (t, [x0,y0,x1,y1,...]) at ~1 Hz
        self._last_scan = 0.0
        self._last_lplan = 0.0
        try:
            from sensor_msgs.msg import LaserScan
            self.create_subscription(LaserScan, "/scan", self.on_scan, 5)
        except ImportError:
            pass
        self.create_subscription(Path, "/local_plan", self.on_local_plan, 5)
        mqos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, mqos)
        self.tfbuf = Buffer(); self.tfl = TransformListener(self.tfbuf, self)
    def on_true(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        # z + roll/pitch let the replay render place the body at the TRUE physics
        # height (the kinematic ground-settle guess made the standup look floaty)
        sinp = 2*(q.w*q.y - q.z*q.x)
        pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        self.true.append((time.time(), m.pose.pose.position.x, m.pose.pose.position.y,
                          yaw, m.pose.pose.position.z, roll, pitch))
    def on_drift(self, m):
        self.drift.append((time.time(), m.pose.pose.position.x, m.pose.pose.position.y))
    def on_joints(self, m):
        now = time.time()
        if now - self._last_joints >= 0.05:
            self._last_joints = now
            self.joints.append((now, *[m.data.joints_data[i].position for i in range(16)]))
    def on_plan(self, m):
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
    def on_scan(self, m):
        now = time.time()
        if now - self._last_scan < 1.0:
            return
        try:
            t = self.tfbuf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return
        self._last_scan = now
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.scans.append((now, t.transform.translation.x, t.transform.translation.y,
                           yaw, *[round(float(r), 3) for r in m.ranges]))
    def on_local_plan(self, m):
        now = time.time()
        if now - self._last_lplan < 1.0 or not m.poses:
            return
        self._last_lplan = now
        pts = []
        for p in m.poses:
            pts.extend((round(p.pose.position.x, 3), round(p.pose.position.y, 3)))
        self.local_plans.append((now, pts))
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

    JHDR = ["t"] + [f"j{i}" for i in range(16)]

    def dump_all():
        dump("traj_true.csv", n.true, ["t", "x", "y", "yaw", "z", "roll", "pitch"])
        dump("traj_drift.csv", n.drift, ["t", "x", "y"])
        dump("traj_est.csv", n.est, ["t", "x", "y"])
        dump("gps.csv", n.gps, ["t", "x", "y"])
        dump("traj_ekf.csv", n.ekf, ["t", "x", "y"])
        dump("joints.csv", n.joints, JHDR)
        with open(f"{OUT}/scans.csv", "w", newline="") as f:
            w = csv.writer(f)
            for row in n.scans:
                w.writerow(row)
        import json
        with open(f"{OUT}/local_plans.jsonl", "w") as f:
            for t, pts in n.local_plans:
                f.write(json.dumps({"t": t, "pts": pts}) + "\n")

    while rclpy.ok() and time.time() - t0 < RUN_SECS:
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() - last > 0.1:
            n.sample_est(); last = time.time()
        # incremental dump: observers copy CSVs while we run — end-only writes made
        # every mid-run copy read the PREVIOUS run (the "blind logger" of session 4)
        if time.time() - last_dump > 10.0:
            dump_all(); last_dump = time.time()
    dump_all()
    if n.plan:
        dump("plan.csv", n.plan, ["x", "y"])
    if n.map is not None:
        res, wdt, hgt, ox, oy, grid = n.map
        np.savez(f"{OUT}/map.npz", res=res, width=wdt, height=hgt, ox=ox, oy=oy, grid=grid)
    print(f"true={len(n.true)} drift={len(n.drift)} est={len(n.est)} "
          f"gps={len(n.gps)} ekf={len(n.ekf)} joints={len(n.joints)} "
          f"plan={len(n.plan) if n.plan else 0} map={'y' if n.map is not None else 'n'}")
    n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
