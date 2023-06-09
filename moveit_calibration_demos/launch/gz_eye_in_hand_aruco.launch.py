#!/usr/bin/env -S ros2 launch
"""MoveIt2 hand-eye calibration example inside Gazebo simulation (eye-to-hand variant)"""

from os import path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    world = LaunchConfiguration("world")
    rviz_config = LaunchConfiguration("rviz_config")

    # List of included launch descriptions
    launch_descriptions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("moveit_calibration_demos"),
                        "launch",
                        "_gz_moveit_calibration.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ["world", world],
                ("rviz_config", rviz_config),
            ],
        ),
    ]

    return LaunchDescription(declared_arguments + launch_descriptions)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # World and model for Gazebo
        DeclareLaunchArgument(
            "world",
            default_value=path.join(
                get_package_share_directory("moveit_calibration_demos"),
                "worlds",
                "eye_in_hand_aruco.sdf",
            ),
            description="Name or filepath of world to load.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("moveit_calibration_demos"),
                "rviz",
                "eye_in_hand_aruco.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
    ]
