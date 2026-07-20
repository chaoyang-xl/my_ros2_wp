#!/usr/bin/env python3
"""Replay the converted Meeting_Room bag through YOLO, MobileSAM and fusion."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("semantic_map_offline"))
    defaults = {
        "bag_uri": (
            "/home/weiyu/vscode_workspace/ros2_wp/ros_bag/Meeting_Room/"
            "rosbag2/car_meetingroom_mapping"
        ),
        "detect_model": "/home/weiyu/vscode_workspace/models/yolo11n.pt",
        "sam_checkpoint": "/home/weiyu/vscode_workspace/models/mobile_sam.pt",
        "sam_source": (
            "/home/weiyu/vscode_workspace/ros2_wp/src/"
            "semantic_map_offline/MobileSAM"
        ),
        "sam_device": "cuda",
        "yolo_output_dir": "/tmp/meeting_room_yolo",
        "output_directory": "/tmp/meeting_room_sam_output",
        "snapshot_path": "/tmp/meeting_room_sam_output/semantic_objects.json",
        "rate": "0.15",
        "start_offset": "0.0",
        "bag_delay_s": "12.0",
        "enable_rviz": "true",
        "rviz_config": str(package_share / "config" / "meeting_room_debug.rviz"),
        "frame_skip": "0",
        "min_confidence": "0.35",
        "classes_filter": ",".join(str(class_id) for class_id in range(1, 80)),
        "pixel_stride": "1",
        "voxel_size": "0.01",
        "projection_cluster_eps": "0.03",
        "projection_cluster_min_points": "10",
        "overlap_radius": "0.04",
        "mask_erode_px": "1",
        # The bag does not contain lidar->camera calibration. These defaults
        # preserve the standard ROS optical-axis convention with zero offset.
        "camera_x": "0.0",
        "camera_y": "0.0",
        "camera_z": "0.0",
        "camera_qx": "-0.5",
        "camera_qy": "0.5",
        "camera_qz": "-0.5",
        "camera_qw": "0.5",
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]

    decoder = Node(
        package="semantic_map_offline",
        executable="compressed_rgbd_decoder_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    odom_tf = Node(
        package="semantic_map_offline",
        executable="odom_camera_tf_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "camera_x": LaunchConfiguration("camera_x"),
            "camera_y": LaunchConfiguration("camera_y"),
            "camera_z": LaunchConfiguration("camera_z"),
            "camera_qx": LaunchConfiguration("camera_qx"),
            "camera_qy": LaunchConfiguration("camera_qy"),
            "camera_qz": LaunchConfiguration("camera_qz"),
            "camera_qw": LaunchConfiguration("camera_qw"),
        }],
    )
    yolo = Node(
        package="opi_yolo_rknn_recorder",
        executable="ai_recorder_node",
        name="meeting_room_yolo",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "image_topic": "/camera/color/image_raw",
            "result_topic": "/yolo/results_json",
            "output_dir": LaunchConfiguration("yolo_output_dir"),
            "detect_model": LaunchConfiguration("detect_model"),
            "enable_detect": True,
            "enable_pose": False,
            "imgsz": 640,
            "conf": LaunchConfiguration("min_confidence"),
            "iou": 0.45,
            "classes_filter": LaunchConfiguration("classes_filter"),
            "frame_skip": LaunchConfiguration("frame_skip"),
            "save_every_n": 30,
            "enable_http": False,
            "publish_debug_image": True,
        }],
    )
    projection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / "launch" / "sam_offline_projection.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "input_topic": "/yolo/results_json",
            "jsonl_path": "",
            "depth_topic": "/camera/depth/image_raw",
            "color_topic": "/camera/color/image_raw",
            "camera_frame": "orbbec_camera_color_optical_frame",
            "target_frame": "odom",
            "camera_fx": "610.5088500976562",
            "camera_fy": "610.8049926757812",
            "camera_cx": "638.3673095703125",
            "camera_cy": "365.3813171386719",
            "depth_scale": "0.001",
            "min_depth_m": "0.3",
            "max_depth_m": "10.0",
            "pixel_stride": LaunchConfiguration("pixel_stride"),
            "projection_voxel_size": LaunchConfiguration("voxel_size"),
            "projection_cluster_eps": LaunchConfiguration("projection_cluster_eps"),
            "projection_cluster_min_points": LaunchConfiguration(
                "projection_cluster_min_points"
            ),
            "max_time_diff_s": "0.10",
            "processing_delay_frames": "5",
            "tf_timeout_s": "0.0",
            "min_confidence": LaunchConfiguration("min_confidence"),
            "excluded_labels": "person",
            "excluded_class_ids": "0",
            "sam_checkpoint": LaunchConfiguration("sam_checkpoint"),
            "sam_source": LaunchConfiguration("sam_source"),
            "sam_device": LaunchConfiguration("sam_device"),
            "mask_erode_px": LaunchConfiguration("mask_erode_px"),
            "publish_debug_image": "true",
            "publish_markers": "true",
            "snapshot_path": LaunchConfiguration("snapshot_path"),
            "output_directory": LaunchConfiguration("output_directory"),
            "voxel_size": LaunchConfiguration("voxel_size"),
            "overlap_radius": LaunchConfiguration("overlap_radius"),
            "snapshot_interval_s": "2.0",
        }.items(),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
    )
    bag_play = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", LaunchConfiguration("bag_uri"),
            "--clock", "--rate", LaunchConfiguration("rate"),
            "--start-offset", LaunchConfiguration("start_offset"),
            "--topics", "/fastlio_odom",
            "/orbbec_camera/color/camera_info",
            "/orbbec_camera/color/image_raw/compressed",
            "/orbbec_camera/depth/image_raw/compressedDepth",
        ],
        output="screen",
    )
    delayed_bag = TimerAction(
        period=LaunchConfiguration("bag_delay_s"), actions=[bag_play]
    )
    return LaunchDescription([
        *arguments, decoder, odom_tf, yolo, projection, rviz, delayed_bag
    ])
