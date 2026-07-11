"""Phase-1 placeholder: launch vendor sim + bridge + commander.
Fill in once OQ-5 (which vendor sim to standardize on) is resolved."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="m20_commander", executable="commander_node"),
        Node(package="m20_locomotion_bridge", executable="bridge_node"),
        Node(package="m20_missions", executable="mission_server"),
    ])
