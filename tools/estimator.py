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
import os
import numpy as np
import rclpy
from rclpy.node import Node
from drdds.msg import JointsData, ImuData
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

WHEELS = (3, 7, 11, 15)
# M20_EKF_TF=1: broadcast odom->base_link from the fused estimate (the sim must run
# with M20_NO_ODOM_TF=1 — one publisher per TF edge). Used for GPS-ANCHORED MAPPING:
# slam_toolbox's odom prior stays GPS-bounded instead of accumulating dead-reckoning
# drift, so the built map doesn't freeze that drift in (measured warp up to 0.65 m).
EKF_TF = os.environ.get("M20_EKF_TF", "0") == "1"
CORR_RATE = 0.05      # m/s   max rate the GPS correction may bend the TF
CORR_RATE_YAW = 0.02  # rad/s (smooth prior for the scan matcher, bounded drift)
R = 0.072   # CALIBRATED effective rolling radius (measured from test drives) — the
            # nominal 0.10 over-reads distance ~38%; must match ODOM_R in mujoco_sim.py
GPS_SIGMA = 0.8


def wrap(a): return math.atan2(math.sin(a), math.cos(a))


class Estimator(Node):
    def __init__(self):
        super().__init__("m20_estimator")
        self.x = np.zeros(3)
        self.tf_state = np.zeros(3)   # smoothed pose for the TF (see step())
        self.gps_anchor = None        # last course-aid anchor fix (see on_gps())
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
        self.tf_bc = TransformBroadcaster(self) if EKF_TF else None
        self.create_timer(1 / 100.0, self.step)      # 100 Hz predict + publish
        self.get_logger().info(
            f"estimator up: wheel+IMU+GPS EKF -> /odom_filtered (tf={'ON' if EKF_TF else 'off'})")

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
        # COURSE-OVER-GROUND yaw aid (2026-07-25): GPS position can't observe yaw,
        # so gyro-bias heading drift slowly ROTATES the odom frame — iteration-3
        # map came out rotated ~3° (landmark y-offset linear in x). For a robot
        # driving forward, the direction of GPS displacement IS the heading:
        # once we've moved >0.8 m from the anchor fix while rolling forward,
        # apply a yaw measurement (sigma 0.35 rad — noisy, but it BOUNDS drift).
        if self.gps_anchor is None:
            self.gps_anchor = z
            return
        d = z - self.gps_anchor
        if float(np.hypot(*d)) > 0.8 and self.vx > 0.15:
            course = math.atan2(d[1], d[0])
            r_yaw = 0.35 ** 2
            innov = wrap(course - self.x[2])
            s = self.P[2, 2] + r_yaw
            k = self.P[:, 2] / s
            self.x = self.x + k * innov
            self.x[2] = wrap(self.x[2])
            self.P = self.P - np.outer(k, self.P[2, :])
            self.gps_anchor = z
        elif float(np.hypot(*d)) > 0.8:
            self.gps_anchor = z                      # pivoting/reversing: re-anchor only

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
        if self.tf_bc is not None:
            # SMOOTHED TF (2026-07-25 lesson): publishing the raw EKF as the odom
            # prior injected 5 Hz GPS jitter into every scan registration and made
            # the map WORSE (tank1 0.27→0.95 m). Scan matchers need a smooth,
            # locally-consistent prior. So the TF integrates the same wheel+IMU
            # prediction, and the GPS-derived correction LEAKS in rate-limited
            # (≤ CORR_RATE m/s) — drift stays bounded, no jumps ever.
            th_s = self.tf_state[2]
            self.tf_state[0] += self.vx * math.cos(th_s) * dt
            self.tf_state[1] += self.vx * math.sin(th_s) * dt
            self.tf_state[2] = wrap(th_s + self.gyro * dt)
            err = self.x - self.tf_state
            err[2] = wrap(err[2])
            lim = np.array([CORR_RATE * dt, CORR_RATE * dt, CORR_RATE_YAW * dt])
            self.tf_state += np.clip(err * 0.5, -lim, lim)
            self.tf_state[2] = wrap(self.tf_state[2])
            tf = TransformStamped()
            tf.header.stamp = o.header.stamp
            tf.header.frame_id = "odom"
            tf.child_frame_id = "base_link"
            tf.transform.translation.x = float(self.tf_state[0])
            tf.transform.translation.y = float(self.tf_state[1])
            tf.transform.rotation.z = math.sin(self.tf_state[2] / 2)
            tf.transform.rotation.w = math.cos(self.tf_state[2] / 2)
            self.tf_bc.sendTransform(tf)


def main():
    rclpy.init()
    try:
        rclpy.spin(Estimator())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
