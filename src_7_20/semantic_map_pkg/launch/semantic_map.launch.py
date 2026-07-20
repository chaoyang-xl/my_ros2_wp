#!/usr/bin/env python3
"""Start the core RGB-D semantic mapping pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _argument(name, default, description):
    return DeclareLaunchArgument(
        name,
        default_value=default,
        description=description,
    )


def generate_launch_description() -> LaunchDescription:
    """Create the frontend bridge and one semantic projection backend."""
    arguments = [
        _argument('use_loop_closure', 'false',
                  'Use loop_closure_guard_node instead of the basic projector'),
        _argument('use_sim_time', 'false', 'Use the rosbag or simulator clock'),
        _argument('enable_debug_viz', 'false',
                  'Display raw semantic seeds before occupancy snapping'),
        _argument('raw_marker_lifetime_s', '5.0',
                  'Raw seed marker lifetime'),
        _argument('raw_point_scale', '0.10', 'Raw seed marker diameter'),
        _argument('raw_publish_hz', '10.0', 'Raw seed marker publish rate'),
        _argument('input_topic', '/yolo/results_json',
                  'Detection and pose JSON topic'),
        _argument('depth_topic', '/camera/depth_image', 'Aligned depth image'),
        _argument('camera_frame', '',
                  'Camera frame override; empty uses the depth image frame'),
        _argument('camera_fx', '320.0', 'Camera focal length fx'),
        _argument('camera_fy', '320.0', 'Camera focal length fy'),
        _argument('camera_cx', '320.0', 'Camera principal point cx'),
        _argument('camera_cy', '240.0', 'Camera principal point cy'),
        _argument('depth_scale', '1.0',
                  'Depth-to-metre scale: 0.001 for millimetres, 1.0 for metres'),
        _argument('target_frame', 'map', 'Semantic seed target frame'),
        _argument('tf_timeout_s', '0.1', 'TF lookup timeout'),
        _argument('slop', '0.1', 'Detection/depth synchronization tolerance'),
        _argument('depth_buffer_size', '30',
                  'Number of recent depth images kept for JSON timestamp matching'),
        _argument('max_depth_time_diff_s', '0.15',
                  'Maximum JSON/depth timestamp difference'),
        _argument('fallback_latest_depth_without_stamp', 'true',
                  'Use latest depth image when detection JSON has no stamp'),
        _argument('use_image_stamp_for_tf', 'false',
                  'Use matched depth image stamp for TF instead of latest transform'),
        _argument('pose_iou_threshold', '0.3', 'Detection-pose IoU threshold'),
        _argument('enable_pose', 'true', 'Attach pose semantics to person seeds'),
        _argument('search_radius_m', '0.2', 'Occupancy snap search radius'),
        _argument('occupied_threshold', '50', 'Occupied-cell threshold'),
        _argument('min_island_pixels', '2', 'Minimum occupancy island size'),
        _argument('max_island_size_m', '2.0',
                  'Maximum island extent before treating it as a wall'),
        _argument('max_total_pixels', '0',
                  'Maximum island cells; zero disables this limit'),
        _argument('match_distance', '1.0', 'Object association distance'),
        _argument('smoothing_alpha', '0.3', 'Position EMA coefficient'),
        _argument('memory_timeout', '5.0', 'Candidate expiration time'),
        _argument('min_confirmed_seen', '50', 'Observations before confirmation'),
        _argument('show_candidates', 'false', 'Publish candidate markers'),
        _argument('label_policy_json', '', 'Per-label policy JSON override'),
        _argument('semantic_objects_topic', '/semantic_objects',
                  'Confirmed-object JSON topic'),
        _argument('semantic_objects_path', '/tmp/semantic_objects.json',
                  'Confirmed-object snapshot path; empty disables file output'),
        _argument('lc_check_period', '1.0', 'Loop-closure TF check period'),
        _argument('lc_translation_threshold', '0.20',
                  'Loop-closure translation threshold'),
        _argument('lc_rotation_threshold', '0.08',
                  'Loop-closure rotation threshold'),
        _argument('lc_source_frame', 'odom', 'Loop-closure source frame'),
        _argument('lc_resnap_enabled', 'true',
                  'Re-snap static objects after loop closure'),
        _argument('lc_resnap_radius_m', '0.3',
                  'Post-loop-closure re-snap radius'),
        _argument('lc_resnap_max_attempts', '4',
                  'Maximum re-snap attempts on successive map generations'),
        _argument('lc_resnap_radius_step_m', '0.15',
                  'Search-radius increase after each failed attempt'),
        _argument('lc_resnap_max_radius_m', '0.6',
                  'Maximum re-snap search radius'),
        _argument('lc_stable_checks', '2',
                  'Consecutive stable TF checks required before re-snap'),
        _argument('lc_stable_translation_threshold', '0.02',
                  'Maximum translation change during a stable TF check'),
        _argument('lc_stable_rotation_threshold', '0.01',
                  'Maximum rotation change during a stable TF check'),
    ]

    frontend = Node(
        package='semantic_map_pkg',
        executable='frontend_bridge_node',
        name='frontend_bridge_node',
        output='screen',
        parameters=[{
            name: LaunchConfiguration(name)
            for name in (
                'input_topic', 'depth_topic', 'camera_frame',
                'camera_fx', 'camera_fy', 'camera_cx', 'camera_cy',
                'depth_scale', 'target_frame', 'tf_timeout_s', 'slop',
                'depth_buffer_size', 'max_depth_time_diff_s',
                'fallback_latest_depth_without_stamp', 'use_image_stamp_for_tf',
                'pose_iou_threshold', 'enable_pose',
            )
        } | {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    projection_parameters = {
        name: LaunchConfiguration(name)
        for name in (
            'search_radius_m', 'occupied_threshold', 'min_island_pixels',
            'max_island_size_m', 'max_total_pixels', 'match_distance',
            'smoothing_alpha', 'memory_timeout', 'min_confirmed_seen',
            'show_candidates', 'label_policy_json',
            'semantic_objects_topic', 'semantic_objects_path',
        )
    }
    projection_parameters['use_sim_time'] = LaunchConfiguration('use_sim_time')

    projection = Node(
        package='semantic_map_pkg',
        executable='semantic_projection_node',
        name='semantic_projection_node',
        output='screen',
        parameters=[projection_parameters],
        condition=UnlessCondition(LaunchConfiguration('use_loop_closure')),
    )

    loop_parameters = dict(projection_parameters)
    loop_parameters.update({
        name: LaunchConfiguration(name)
        for name in (
            'lc_check_period', 'lc_translation_threshold',
            'lc_rotation_threshold', 'lc_source_frame',
            'lc_resnap_enabled', 'lc_resnap_radius_m', 'lc_stable_checks',
            'lc_resnap_max_attempts', 'lc_resnap_radius_step_m',
            'lc_resnap_max_radius_m',
            'lc_stable_translation_threshold',
            'lc_stable_rotation_threshold',
        )
    })
    loop_closure = Node(
        package='semantic_map_pkg',
        executable='loop_closure_guard_node',
        name='loop_closure_guard_node',
        output='screen',
        parameters=[loop_parameters],
        condition=IfCondition(LaunchConfiguration('use_loop_closure')),
    )

    raw_seed_visualizer = Node(
        package='semantic_map_pkg',
        executable='raw_seed_visualizer_node',
        name='raw_seed_visualizer_node',
        output='screen',
        parameters=[{
            'input_topic': '/semantic_seed',
            'marker_lifetime_s': LaunchConfiguration('raw_marker_lifetime_s'),
            'point_scale': LaunchConfiguration('raw_point_scale'),
            'publish_hz': LaunchConfiguration('raw_publish_hz'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_debug_viz')),
    )

    return LaunchDescription([
        *arguments,
        frontend,
        projection,
        loop_closure,
        raw_seed_visualizer,
    ])
