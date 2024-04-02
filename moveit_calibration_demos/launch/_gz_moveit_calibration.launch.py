#!/usr/bin/env -S ros2 launch
"""MoveIt2 hand-eye calibration example inside Gazebo simulation (eye-in-hand variant)"""

from math import pi
from os import path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    world = LaunchConfiguration("world")
    robot = LaunchConfiguration("robot")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    log_level = LaunchConfiguration("log_level")

    # List of included launch descriptions
    launch_descriptions = [
        # TODO: Use official launch script from moveit2_tutorials once it supports this use case
        # Launch move_group of MoveIt 2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare(robot),
                        "launch",
                        "gz.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ["world", world],
                ("rviz_config", rviz_config),
                ("use_sim_time", use_sim_time),
                ("gz_verbosity", gz_verbosity),
                ("log_level", log_level),
            ],
        ),
    ]

    # List of nodes to be launched
    nodes = [
        # static_transform_publisher (identity; world_moveit_calibration)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                ["world"],
                "--child-frame-id",
                ["world_moveit_calibration"],
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # static_transform_publisher (identity; rgbd_camera_base_link)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "rgbd_camera_base_link",
                "--child-frame-id",
                "rgbd_camera/rgbd_camera_link/rgbd_camera",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # static_transform_publisher (rotation; rgbd_camera_optical_frame)
        # Note: Necessary because Gazebo camera sensors use different coordinates (X forward)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "rgbd_camera_optical_frame",
                "--child-frame-id",
                "rgbd_camera_base_link",
                "--roll",
                str(-pi / 2),
                "--pitch",
                "0.0",
                "--yaw",
                str(-pi / 2),
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # ros_gz_bridge (camera pose ground truth -> ROS 2)
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="log",
            arguments=[
                "/model/rgbd_camera/pose"
                + "@"
                + "geometry_msgs/msg/PoseStamped[ignition.msgs.Pose",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[("/model/rgbd_camera/pose", "/camera_pose_ground_truth")],
        ),
        # ros_gz_bridge (image -> ROS 2; camera info -> ROS 2)
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="log",
            arguments=[
                ["/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image"],
                ["/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image"],
                [
                    "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                ],
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    return LaunchDescription(declared_arguments + launch_descriptions + nodes)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # World and model for Gazebo
        DeclareLaunchArgument(
            "world",
            default_value="ERROR_empty_launch_argument",
            description="Name or filepath of world to load.",
        ),
        DeclareLaunchArgument(
            "robot",
            default_value="panda",
            description="Name or filepath of model to load.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "rviz_config",
            default_value="ERROR_empty_launch_argument",
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "gz_verbosity",
            default_value="3",
            description="Verbosity level for Gazebo (0~4).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]
