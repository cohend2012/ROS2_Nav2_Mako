#!/usr/bin/env python3
"""m20_estimator: lightweight EKF for drift-free localization.

Fuses wheel odometry (/JOINTS_DATA) + IMU yaw-rate (/IMU_DATA) + GPS (/gps) into
/odom_filtered. Predict from wheel speed + gyro (high rate); correct from GPS (bounds
drift). State [x, y, yaw]. Validated in tools/ekf_benchmark.py (drift 2.4 m -> ~0.3 m).

Production option is robot_localization (+navsat_transform); this is a minimal,
dependency-light estimator that runs wherever the drdds msgs exist. GPS arrives here as
nav_msgs/Odometry (x,y) — a local-frame proxy for NavSatFix+navsat_transform.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from drdds.msg import JointsData, ImuData
from nav_msgs.msg import Odometry

WHEELS = (3, 7, 11, 15)
R = 0.072   # CALIBRATED effective rolling radius (measured from test drives) — the
            # nominal 0.10 over-reads distance ~38%; must match ODOM_R in mujoco_sim.py
GPS_SIGMA = 0.8


def wrap(a): return math.atan2(math.sin(a), math.cos(a))


class Estimator(Node):
    def __init__(self):
        super().__init__("m20_estimator")
        self.x = np.zeros(3)
        self.P = np.eye(3) * 0.5
        # Process noise: middle ground. Too small (0.02) over-trusts odometry -> drifts
        # ~2.8 m; too large (0.6) chases noisy GPS -> jittery. ~0.2 trusts the smooth
        # wheel+IMU prediction enough to FILTER GPS noise while still bounding drift.
        self.Q = np.diag([0.2, 0.2, 0.12]) ** 2
        self.Rgps = np.eye(2) * 0.8 ** 2       # GPS measurement noise (matches sim ~0.8 m)
        self.vx = 0.0
        self.gyro = 0.0
        self.last = None
        self.create_subscription(JointsData, "/JOINTS_DATA", self.on_joints, 10)
        self.create_subscription(ImuData, "/IMU_DATA", self.on_imu, 10)
        self.create_subscription(Odometry, "/gps", self.on_gps, 10)
        self.pub = self.create_publisher(Odometry, "/odom_filtered", 10)
        self.create_timer(1 / 100.0, self.step)      # 100 Hz predict + publish
        self.get_logger().info("estimator up: wheel+IMU+GPS EKF -> /odom_filtered")

    def on_joints(self, m: JointsData):
        self.vx = -float(np.mean([m.data.joints_data[i].velocity for i in WHEELS])) * R

    def on_imu(self, m: ImuData):
        self.gyro = float(m.data.omega_z)

    def on_gps(self, m: Odometry):                   # GPS correction (bounds drift)
        z = np.array([m.pose.pose.position.x, m.pose.pose.position.y])
        H = np.array([[1, 0, 0], [0, 1, 0]])
        yk = z - H @ self.x
        S = H @ self.P @ H.T + self.Rgps
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ yk
        self.P = (np.eye(3) - K @ H) @ self.P

    def step(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last is None:
            self.last = now
            return
        dt = now - self.last
        self.last = now
        th = self.x[2]
        self.x[0] += self.vx * math.cos(th) * dt
        self.x[1] += self.vx * math.sin(th) * dt
        self.x[2] = wrap(th + self.gyro * dt)
        F = np.array([[1, 0, -self.vx * math.sin(th) * dt],
                      [0, 1,  self.vx * math.cos(th) * dt],
                      [0, 0, 1]])
        self.P = F @ self.P @ F.T + self.Q * dt
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = "map"
        o.child_frame_id = "base_link"
        o.pose.pose.position.x = float(self.x[0])
        o.pose.pose.position.y = float(self.x[1])
        o.pose.pose.orientation.z = math.sin(self.x[2] / 2)
        o.pose.pose.orientation.w = math.cos(self.x[2] / 2)
        self.pub.publish(o)


def main():
    rclpy.init()
    try:
        rclpy.spin(Estimator())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
