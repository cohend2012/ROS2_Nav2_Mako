#!/usr/bin/env python3
"""Relay the running sim's robot state to a NATIVE MuJoCo viewer (tools/splat/mujoco_mirror.py).

WSLg can't paint MuJoCo windows on some machines, so the sim runs headless in WSL and a
Windows-side MuJoCo viewer mirrors it. This node subscribes to the sim's ground truth
(/odom_true: base pose) and /JOINTS_DATA (16 joints, MuJoCo qpos[7:23] order) and streams
`qpos` (23 floats) as text lines over TCP. WSL2 forwards the port to Windows localhost.

Protocol (one line each, newline-terminated):
  first line : "scene <mjcf file name>"        e.g. "scene indoor_splat.xml"
  then       : 23 comma-separated floats       x y z qw qx qy qz q0..q15   (~60 Hz)
Env: MIRROR_PORT (8770), M20_SCENE (scene file name the sim is running).
"""
import os, socket, threading, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from drdds.msg import JointsData

PORT = int(os.environ.get("MIRROR_PORT", 8770))
SCENE = os.environ.get("M20_SCENE", "indoor_splat.xml")
HZ = 60.0

state = {"base": None, "q": [0.0] * 16}


def on_odom(m):
    p, o = m.pose.pose.position, m.pose.pose.orientation
    state["base"] = (p.x, p.y, p.z, o.w, o.x, o.y, o.z)


def on_joints(m):
    state["q"] = [m.data.joints_data[i].position for i in range(16)]


def serve(conn, addr):
    print(f"[mirror_relay] viewer connected from {addr}", flush=True)
    try:
        conn.sendall(f"scene {SCENE}\n".encode())
        while True:
            if state["base"] is not None:
                conn.sendall((",".join(f"{v:.5f}" for v in (*state["base"], *state["q"])) + "\n").encode())
            time.sleep(1.0 / HZ)
    except OSError:
        print(f"[mirror_relay] viewer {addr} disconnected", flush=True)
    finally:
        conn.close()


def main():
    rclpy.init()
    node = rclpy.create_node("m20_mujoco_mirror_relay")
    node.create_subscription(Odometry, "/odom_true", on_odom, qos_profile_sensor_data)
    node.create_subscription(JointsData, "/JOINTS_DATA", on_joints, 10)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", PORT))
    srv.listen(4)
    print(f"[mirror_relay] streaming {SCENE} robot state on :{PORT}", flush=True)
    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        threading.Thread(target=serve, args=(conn, addr), daemon=True).start()


if __name__ == "__main__":
    main()
