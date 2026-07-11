"""Robot-side station bridge (ADR-013).

Runs ON the robot; the operator connects from anywhere on the network:
  * foxglove_bridge (ws://<robot>:8765) -> Foxglove Studio: 3D view of lidar
    point clouds, TF, costmaps, camera topics, plots of /m20/health,
    /m20/failsafe, /m20/mode, /m20/behavior/status. Day-1 monitoring with
    zero custom UI code.
  * rosbridge (ws://<robot>:9090) -> custom web operator UI later (teleop,
    behavior triggers, mission editing). Same JSON/WebSocket surface the AI
    agent uses -> one gateway, two clients (ADR-014).

robot_id namespaces everything: fleet rule ADR-014. Default empty = single robot.
TODO(OQ-14): add WebRTC/RTSP camera relay for low-latency drive-camera video;
Foxglove image topics are fine for monitoring, not for teleop at speed.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="",
                              description="Namespace for fleet operation (ADR-014)"),
        Node(package="foxglove_bridge", executable="foxglove_bridge",
             namespace=robot_id, parameters=[{"port": 8765}]),
        Node(package="rosbridge_server", executable="rosbridge_websocket",
             namespace=robot_id, parameters=[{"port": 9090}]),
    ])
