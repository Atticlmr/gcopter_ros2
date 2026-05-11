from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("gcopter_ros2")
    config_file = os.path.join(pkg_dir, "config", "gcopter_ros2.yaml")

    return LaunchDescription([
        Node(
            package="gcopter_ros2",
            executable="gcopter_ros2_planner",
            name="gcopter_ros2_planner",
            output="screen",
            parameters=[config_file],
        ),
    ])
