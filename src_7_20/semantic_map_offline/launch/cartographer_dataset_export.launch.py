#!/usr/bin/env python3
"""Load a frozen pbstream and export a Replica-compatible RGB-D dataset."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = Path(
        get_package_share_directory("semantic_map_offline")
    )
    default_config_dir = package_share / "config" / "cartographer"

    defaults = {
        "configuration_directory": str(default_config_dir),
        "configuration_basename": "offline_ros_bag_with_odom_imu.lua",
        "trajectory_id": "0",
        "jsonl_path": "",
        "color_topic": "/camera/color/image_raw",
        "depth_topic": "/camera/depth/image_raw",
        "camera_info_topic": "/camera/color/camera_info",
        "static_tf_topic": "/tf_static",
        "tracking_frame": "camera_gyro_frame",
        "camera_frame": "camera_color_optical_frame",
        "max_rgb_depth_time_diff_s": "0.05",
        "max_detection_time_diff_s": "0.15",
        "max_trajectory_interval_s": "10.0",
        "input_depth_scale": "0.001",
        "start_frame": "0",
        "max_frames": "0",
        "jpeg_quality": "95",
        "overwrite": "false",
        "allow_resolution_mismatch": "false",
        "service_timeout_s": "30.0",
        "startup_delay_s": "2.0",
    }
    arguments = [
        DeclareLaunchArgument("pbstream_path"),
        DeclareLaunchArgument("bag_uri"),
        DeclareLaunchArgument("output_directory"),
        *[
            DeclareLaunchArgument(name, default_value=value)
            for name, value in defaults.items()
        ],
    ]

    cartographer = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_final_state",
        output="screen",
        arguments=[
            "-configuration_directory",
            LaunchConfiguration("configuration_directory"),
            "-configuration_basename",
            LaunchConfiguration("configuration_basename"),
            "-load_state_filename",
            LaunchConfiguration("pbstream_path"),
            "-load_frozen_state=true",
            "-start_trajectory_with_default_topics=false",
        ],
        parameters=[{"use_sim_time": False}],
    )

    exporter = Node(
        package="semantic_map_offline",
        executable="cartographer_dataset_exporter",
        name="cartographer_dataset_exporter",
        output="screen",
        parameters=[{
            "bag_uri": LaunchConfiguration("bag_uri"),
            "output_directory": LaunchConfiguration("output_directory"),
            "jsonl_path": LaunchConfiguration("jsonl_path"),
            "trajectory_id": ParameterValue(
                LaunchConfiguration("trajectory_id"), value_type=int
            ),
            "trajectory_service": "/trajectory_query",
            "service_timeout_s": ParameterValue(
                LaunchConfiguration("service_timeout_s"), value_type=float
            ),
            "color_topic": LaunchConfiguration("color_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "static_tf_topic": LaunchConfiguration("static_tf_topic"),
            "tracking_frame": LaunchConfiguration("tracking_frame"),
            "camera_frame": LaunchConfiguration("camera_frame"),
            "max_rgb_depth_time_diff_s": ParameterValue(
                LaunchConfiguration("max_rgb_depth_time_diff_s"),
                value_type=float,
            ),
            "max_detection_time_diff_s": ParameterValue(
                LaunchConfiguration("max_detection_time_diff_s"),
                value_type=float,
            ),
            "max_trajectory_interval_s": ParameterValue(
                LaunchConfiguration("max_trajectory_interval_s"),
                value_type=float,
            ),
            "input_depth_scale": ParameterValue(
                LaunchConfiguration("input_depth_scale"), value_type=float
            ),
            "start_frame": ParameterValue(
                LaunchConfiguration("start_frame"), value_type=int
            ),
            "max_frames": ParameterValue(
                LaunchConfiguration("max_frames"), value_type=int
            ),
            "jpeg_quality": ParameterValue(
                LaunchConfiguration("jpeg_quality"), value_type=int
            ),
            "overwrite": ParameterValue(
                LaunchConfiguration("overwrite"), value_type=bool
            ),
            "allow_resolution_mismatch": ParameterValue(
                LaunchConfiguration("allow_resolution_mismatch"),
                value_type=bool,
            ),
        }],
    )

    delayed_exporter = TimerAction(
        period=LaunchConfiguration("startup_delay_s"),
        actions=[exporter],
    )
    stop_after_export = RegisterEventHandler(
        OnProcessExit(
            target_action=exporter,
            on_exit=[Shutdown(reason="Cartographer dataset export finished")],
        )
    )
    return LaunchDescription([
        *arguments,
        cartographer,
        delayed_exporter,
        stop_after_export,
    ])
