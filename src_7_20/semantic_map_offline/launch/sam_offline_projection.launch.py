#!/usr/bin/env python3
"""Launch MobileSAM mask projection and semantic object fusion."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    defaults = {
        "use_sim_time": "true",
        "color_topic": "/camera/color/image_raw",
        "sam_checkpoint": "",
        "sam_source": "",
        "sam_device": "",
        "mask_erode_px": "2",
        "publish_debug_image": "true",
        "debug_image_topic": "/semantic_offline/sam_debug_image",
        "debug_mask_alpha": "0.45",
        "color_buffer_size": "30",
        "input_topic": "/yolo/results_json",
        "depth_topic": "/camera/depth/image_raw",
        "cloud_topic": "/semantic_offline/points",
        "metadata_topic": "/semantic_offline/detections",
        "jsonl_path": "",
        "save_directory": "",
        "camera_frame": "",
        "target_frame": "map",
        "camera_fx": "311.3878784",
        "camera_fy": "311.3878784",
        "camera_cx": "317.5",
        "camera_cy": "198.5",
        "depth_scale": "0.001",
        "min_depth_m": "0.3",
        "max_depth_m": "5.0",
        "pixel_stride": "2",
        "projection_voxel_size": "0.0",
        "projection_cluster_eps": "0.0",
        "projection_cluster_min_points": "10",
        "max_time_diff_s": "0.15",
        "detection_buffer_size": "100",
        "processing_delay_frames": "0",
        "tf_timeout_s": "0.3",
        "min_confidence": "0.0",
        "excluded_labels": "person",
        "excluded_class_ids": "",
        "image_qos_reliable": "false",
        "fused_cloud_topic": "/semantic_offline/fused_points",
        "objects_topic": "/semantic_offline/objects",
        "marker_topic": "/semantic_offline/object_markers",
        "publish_markers": "true",
        "snapshot_path": "",
        "output_directory": "",
        "voxel_size": "0.02",
        "overlap_radius": "0.04",
        "max_centroid_distance_m": "1.0",
        "min_geometric_overlap": "0.05",
        "min_bbox_overlap": "0.0",
        "association_threshold": "0.45",
        "geometry_weight": "0.7",
        "semantic_weight": "0.3",
        "observation_cluster_eps": "0.10",
        "observation_cluster_min_points": "10",
        "max_extent_growth": "2.0",
        "denoise_interval": "20",
        "map_merge_interval": "20",
        "map_merge_overlap": "0.80",
        "min_confirmed_observations": "3",
        "candidate_max_missed_frames": "30",
        "stale_after_s": "0.0",
        "non_fusing_labels": "person",
        "sync_buffer_size": "50",
        "snapshot_interval_s": "2.0",
    }
    arguments = [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
    fusion_names = (
        "use_sim_time", "cloud_topic", "metadata_topic", "fused_cloud_topic",
        "objects_topic", "marker_topic", "publish_markers", "snapshot_path",
        "output_directory", "voxel_size", "overlap_radius",
        "max_centroid_distance_m", "min_geometric_overlap", "min_bbox_overlap",
        "association_threshold", "geometry_weight", "semantic_weight",
        "stale_after_s", "non_fusing_labels", "sync_buffer_size",
        "observation_cluster_eps", "observation_cluster_min_points",
        "max_extent_growth", "denoise_interval", "map_merge_interval",
        "map_merge_overlap", "min_confirmed_observations",
        "candidate_max_missed_frames",
        "snapshot_interval_s",
    )
    projector_names = tuple(name for name in defaults if name not in fusion_names[3:])
    projector_parameters = {
        name: (
            ParameterValue(LaunchConfiguration(name), value_type=str)
            if name == "excluded_class_ids"
            else LaunchConfiguration(name)
        )
        for name in projector_names
    }
    projector = Node(
        package="semantic_map_offline",
        executable="sam_offline_projector_node",
        output="screen",
        parameters=[projector_parameters],
    )
    fusion = Node(
        package="semantic_map_offline",
        executable="object_fusion_node",
        output="screen",
        parameters=[{name: LaunchConfiguration(name) for name in fusion_names}],
    )
    return LaunchDescription([*arguments, projector, fusion])
