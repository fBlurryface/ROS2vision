import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("recognition_pkg"),
        "config",
        "target_detector.face.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to the parameter YAML file for target_detector_node",
        ),
        Node(
            package="recognition_pkg",
            executable="target_detector_node",
            name="target_detector_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])