#!/usr/bin/env python3
"""Log the Nav2 run for honest verification:
  - ground-truth trajectory (/odom)  -> traj.csv
  - Nav2 global plan (/plan)         -> plan.csv (last received)
  - occupancy map (/map)             -> map.npz (last received)
Runs for RUN_SECS or until Ctrl-C.
"""
import os, time, csv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, Path, OccupancyGrid

OUT = "/cfg/out"
RUN_SECS = float(os.environ.get("RUN_SECS", "90"))

class Logger(Node):
    def __init__(self):
        super().__init__("nav_logger")
        self.traj = []
        self.plan = None
        self.map = None
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Path, "/plan", self.on_plan, 10)
        mqos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, mqos)
    def on_odom(self, m):
        self.traj.append((time.time(), m.pose.pose.position.x, m.pose.pose.position.y))
    def on_plan(self, m):
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
    def on_map(self, m):
        self.map = (m.info.resolution, m.info.width, m.info.height,
                    m.info.origin.position.x, m.info.origin.position.y,
                    np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width))

def main():
    os.makedirs(OUT, exist_ok=True)
    rclpy.init(); n = Logger()
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < RUN_SECS:
        rclpy.spin_once(n, timeout_sec=0.1)
    # write outputs
    with open(f"{OUT}/traj.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t", "x", "y"]); w.writerows(n.traj)
    if n.plan:
        with open(f"{OUT}/plan.csv", "w", newline="") as f:
            w = csv.writer(f); w.writerow(["x", "y"]); w.writerows(n.plan)
    if n.map is not None:
        res, wdt, hgt, ox, oy, grid = n.map
        np.savez(f"{OUT}/map.npz", res=res, width=wdt, height=hgt, ox=ox, oy=oy, grid=grid)
    print(f"logged traj={len(n.traj)} plan={len(n.plan) if n.plan else 0} map={'yes' if n.map is not None else 'no'}")
    n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
