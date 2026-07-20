#!/usr/bin/env python3
"""Run scan SLAM, OPI YOLO and bbox semantic projection on semantic_01."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    mapping_share = Path(get_package_share_directory("semantic_map_pkg"))
    offline_share = Path(get_package_share_directory("semantic_map_offline"))
    mapping_launch = mapping_share / "launch" / "offline_cartographer_bag.launch.py"
    projection_launch = offline_share / "launch" / "offline_projection.launch.py"

    defaults = {
        "bag_uri": "/home/weiyu/vscode_workspace/ros2_wp/ros_bag/semantic_01",
        "run_yolo": "true",
        "jsonl_path": "",
        "detect_model": "/home/weiyu/vscode_workspace/models/yolo11n.pt",
        "yolo_output_dir": "/tmp/semantic_01_yolo",
        "snapshot_path": "/tmp/semantic_01_output/semantic_objects.json",
        "output_directory": "/tmp/semantic_01_output",
        "projected_frames_dir": "",
        "rate": "0.5",
        "start_offset": "0.0",
        "bag_delay_s": "5.0",
        "enable_rviz": "true",
        "enable_http": "false",
        "http_port": "8088",
        "frame_skip": "0",
        "pixel_stride": "4",
        "min_confidence": "0.35",
        "classes_filter": ",".join(str(class_id) for class_id in range(1, 80)),
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(mapping_launch)),
        launch_arguments={
            "bag_uri": LaunchConfiguration("bag_uri"),
            "configuration_basename": "offline_ros_bag_no_odom.lua",
            "publish_robot_state": "false",
            "play_color": "true",
            "play_depth": "true",
            "rate": LaunchConfiguration("rate"),
            "start_offset": LaunchConfiguration("start_offset"),
            "bag_delay_s": LaunchConfiguration("bag_delay_s"),
            "enable_rviz": LaunchConfiguration("enable_rviz"),
        }.items(),
    )

    yolo = Node(
        package="opi_yolo_rknn_recorder",
        executable="ai_recorder_node",
        name="semantic_01_yolo",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_yolo")),
        parameters=[{
            "use_sim_time": True,
            "image_topic": "/camera/color/image_raw",
            "result_topic": "/yolo/results_json",
            "debug_image_topic": "/yolo/debug_image",
            "output_dir": LaunchConfiguration("yolo_output_dir"),
            "detect_model": LaunchConfiguration("detect_model"),
            "enable_detect": True,
            "enable_pose": False,
            "imgsz": 640,
            "conf": LaunchConfiguration("min_confidence"),
            "classes_filter": LaunchConfiguration("classes_filter"),
            "iou": 0.45,
            "frame_skip": LaunchConfiguration("frame_skip"),
            "save_every_n": 30,
            "enable_http": LaunchConfiguration("enable_http"),
            "http_port": LaunchConfiguration("http_port"),
            "publish_debug_image": True,
        }],
    )

    projection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(projection_launch)),
        launch_arguments={
            "use_sim_time": "true",
            "input_topic": "/yolo/results_json",
            "jsonl_path": LaunchConfiguration("jsonl_path"),
            "depth_topic": "/camera/depth/image_raw",
            "color_topic": "/camera/color/image_raw",
            "camera_frame": "camera_color_optical_frame",
            "target_frame": "map",
            "camera_fx": "365.1741638183594",
            "camera_fy": "365.42144775390625",
            "camera_cx": "318.27630615234375",
            "camera_cy": "243.80377197265625",
            "depth_scale": "0.001",
            "pixel_stride": LaunchConfiguration("pixel_stride"),
            "max_time_diff_s": "0.20",
            "processing_delay_frames": "5",
            "tf_timeout_s": "0.0",
            "min_confidence": LaunchConfiguration("min_confidence"),
            "excluded_labels": "person",
            "excluded_class_ids": "0",
            "save_directory": LaunchConfiguration("projected_frames_dir"),
            "snapshot_path": LaunchConfiguration("snapshot_path"),
            "output_directory": LaunchConfiguration("output_directory"),
        }.items(),
    )

    return LaunchDescription([*arguments, mapping, yolo, projection])
