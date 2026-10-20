from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("drone_node")) / "config"
    return LaunchDescription([
        Node(
            package="drone_node",
            executable="drone_node_exec",
            output="screen",
            parameters=[
                str(config / "inter.yaml"),
                str(config / "flight_controller.yaml"),
            ],
        ),
    ])