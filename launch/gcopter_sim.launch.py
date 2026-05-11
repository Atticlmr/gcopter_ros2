from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("gcopter_ros2")
    config_file = os.path.join(pkg_dir, "config", "gcopter_ros2.yaml")
    rviz_config = os.path.join(pkg_dir, "config", "gcopter_rviz.rviz")

    mockamap_node = Node(
        package="mockamap",
        executable="mockamap_node",
        name="mockamap_node",
        output="screen",
        parameters=[
            {"seed": 1024},
            {"update_freq": 1.0},
            {"resolution": 0.25},
            {"x_length": 50},
            {"y_length": 50},
            {"z_length": 5},
            {"type": 1},
            {"complexity": 0.025},
            {"fill": 0.3},
            {"fractal": 1},
            {"attenuation": 0.1},
        ],
    )

    planner_node = Node(
        package="gcopter_ros2",
        executable="gcopter_ros2_planner",
        name="gcopter_ros2_planner",
        output="screen",
        parameters=[config_file],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    plot_node = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "rqt_plot",
            "rqt_plot",
            "/visualizer/speed",
            "/visualizer/total_thrust",
            "/visualizer/tilt_angle",
            "/visualizer/body_rate",
        ],
        output="screen",
    )

    return LaunchDescription([
        mockamap_node,
        planner_node,
        rviz_node,
        plot_node,
    ])
