import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("control_pkg"),
        "config",
        "control_params.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to the parameter YAML file for control nodes",
        ),
        Node(
            package="control_pkg",
            executable="target_follower_node",
            name="target_follower_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
        Node(
            package="control_pkg",
            executable="uno_serial_bridge_node",
            name="uno_serial_bridge_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])