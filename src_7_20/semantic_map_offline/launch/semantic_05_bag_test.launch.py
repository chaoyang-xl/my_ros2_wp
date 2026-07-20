#!/usr/bin/env python3
"""Run semantic_05 with scan, wheel odometry, corrected IMU and RGB-D semantics."""

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
    defaults = {
        "bag_uri": "/home/weiyu/vscode_workspace/ros2_wp/ros_bag/semantic_05",
        "run_yolo": "false",
        "jsonl_path": "",
        "detect_model": "/home/weiyu/vscode_workspace/models/yolov8s-world.pt",
        "clip_model": "/home/weiyu/vscode_workspace/models/clip/ViT-B-32.pt",
        "classes_path": str(offline_share / "config" / "class_list" / "gpt_indoor_general.txt"),
        "yolo_device": "0",
        "yolo_excluded_labels": "person,wall,floor,ceiling,unknown",
        "yolo_output_dir": "/tmp/semantic_05_yolo",
        "save_yolo_images": "true",
        "publish_yolo_debug_image": "true",
        "yolo_save_every_n": "30",
        "yolo_image_jpeg_quality": "90",
        "snapshot_path": "/tmp/semantic_05_output/semantic_objects.json",
        "output_directory": "/tmp/semantic_05_output",
        "projected_frames_dir": "",
        "rate": "0.5",
        "start_offset": "0.0",
        "bag_delay_s": "5.0",
        "enable_rviz": "true",
        "frame_skip": "0",
        "pixel_stride": "2",
        "voxel_size": "0.015",
        "overlap_radius": "0.04",
        "min_confidence": "0.35",
        "excluded_class_ids": "",
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(mapping_share / "launch" / "offline_cartographer_bag.launch.py")
        ),
        launch_arguments={
            "bag_uri": LaunchConfiguration("bag_uri"),
            "configuration_basename": "offline_ros_bag_with_odom_imu.lua",
            "publish_robot_state": "false",
            "play_color": "true",
            "play_color_info": "true",
            "play_depth": "true",
            "play_imu": "true",
            "enable_imu_sync": "true",
            "rate": LaunchConfiguration("rate"),
            "start_offset": LaunchConfiguration("start_offset"),
            "bag_delay_s": LaunchConfiguration("bag_delay_s"),
            "enable_rviz": LaunchConfiguration("enable_rviz"),
        }.items(),
    )

    yolo = Node(
        package="semantic_map_offline",
        executable="yolo_world_recorder_node",
        name="semantic_05_yolo_world",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_yolo")),
        parameters=[{
            "use_sim_time": True,
            "image_topic": "/camera/color/image_raw",
            "result_topic": "/yolo/results_json",
            "output_dir": LaunchConfiguration("yolo_output_dir"),
            "detect_model": LaunchConfiguration("detect_model"),
            "clip_model": LaunchConfiguration("clip_model"),
            "classes_path": LaunchConfiguration("classes_path"),
            "excluded_labels": LaunchConfiguration("yolo_excluded_labels"),
            "device": LaunchConfiguration("yolo_device"),
            "imgsz": 640,
            "conf": LaunchConfiguration("min_confidence"),
            "iou": 0.45,
            "frame_skip": LaunchConfiguration("frame_skip"),
            "save_annotated_images": LaunchConfiguration("save_yolo_images"),
            "save_every_n": LaunchConfiguration("yolo_save_every_n"),
            "image_jpeg_quality": LaunchConfiguration("yolo_image_jpeg_quality"),
            "publish_debug_image": LaunchConfiguration("publish_yolo_debug_image"),
        }],
    )

    # semantic_05 already records registered depth in the RGB optical frame.
    projection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(offline_share / "launch" / "offline_projection.launch.py")
        ),
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
            "max_time_diff_s": "0.15",
            "processing_delay_frames": "5",
            "tf_timeout_s": "0.0",
            "min_confidence": LaunchConfiguration("min_confidence"),
            "excluded_labels": "person",
            "excluded_class_ids": LaunchConfiguration("excluded_class_ids"),
            "image_qos_reliable": "true",
            "save_directory": LaunchConfiguration("projected_frames_dir"),
            "snapshot_path": LaunchConfiguration("snapshot_path"),
            "output_directory": LaunchConfiguration("output_directory"),
            "voxel_size": LaunchConfiguration("voxel_size"),
            "overlap_radius": LaunchConfiguration("overlap_radius"),
            "snapshot_interval_s": "2.0",
        }.items(),
    )

    return LaunchDescription([*arguments, mapping, yolo, projection])
