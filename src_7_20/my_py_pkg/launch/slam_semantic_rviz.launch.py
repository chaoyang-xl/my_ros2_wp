#!/usr/bin/env python3
"""Launch RViz with a saved SLAM map and semantic object markers."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WORKSPACE_SRC = Path('/home/weiyu/vscode_workspace/ros2_wp/src')
DEFAULT_OUTPUT_ROOT = WORKSPACE_SRC / 'semantic_map_offline' / 'outputs' / 'semantic_map_05_processed_tf'
DEFAULT_MAP_YAML = DEFAULT_OUTPUT_ROOT / 'slam_map' / 'semantic_05.yaml'
DEFAULT_OBJECTS_JSON = DEFAULT_OUTPUT_ROOT / 'tracking_sam_final' / 'semantic_objects.json'


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('my_py_pkg'))
    default_rviz = package_share / 'config' / 'slam_semantic_map.rviz'

    return LaunchDescription([
        DeclareLaunchArgument('map_yaml', default_value=str(DEFAULT_MAP_YAML)),
        DeclareLaunchArgument('objects_json', default_value=str(DEFAULT_OBJECTS_JSON)),
        DeclareLaunchArgument('rviz_config', default_value=str(default_rviz)),
        DeclareLaunchArgument('min_observations', default_value='5'),
        DeclareLaunchArgument('min_confidence', default_value='0.0'),
        DeclareLaunchArgument('classes', default_value=''),
        Node(
            package='my_py_pkg',
            executable='slam_semantic_rviz_node',
            name='slam_semantic_rviz_node',
            output='screen',
            parameters=[{
                'map_yaml': LaunchConfiguration('map_yaml'),
                'objects_json': LaunchConfiguration('objects_json'),
                'min_observations': LaunchConfiguration('min_observations'),
                'min_confidence': LaunchConfiguration('min_confidence'),
                'classes': LaunchConfiguration('classes'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
