#!/usr/bin/env python3
"""Start package-local Gazebo, Nav2, and semantic navigation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Build the standalone semantic-navigation simulation."""
    package_share = get_package_share_directory('semantic_navigation_pkg')
    launch_dir = Path(package_share) / 'launch'
    default_world = str(Path(package_share) / 'worlds' / 'semantic_house.world')
    default_params = str(Path(package_share) / 'config' / 'nav2_params.yaml')
    default_rviz = str(Path(package_share) / 'config' / 'semantic_navigation.rviz')

    arguments = [
        DeclareLaunchArgument(
            'map',
            description='Absolute path to occupancy map YAML',
        ),
        DeclareLaunchArgument(
            'semantic_map',
            description='Absolute path to semantic_objects.json',
        ),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('nav2_params', default_value=default_params),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('spawn_x', default_value='-2.0'),
        DeclareLaunchArgument('spawn_y', default_value='1.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.08'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('standoff_m', default_value='0.7'),
        DeclareLaunchArgument('robot_clearance_m', default_value='0.20'),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'fishbot_gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            'use_sim_time': 'true',
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'nav2_localization.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('nav2_params'),
            'use_sim_time': 'true',
            'enable_rviz': LaunchConfiguration('enable_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
    )

    semantic_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'semantic_navigation.launch.py')),
        launch_arguments={
            'semantic_map_path': LaunchConfiguration('semantic_map'),
            'use_sim_time': 'true',
            'standoff_m': LaunchConfiguration('standoff_m'),
            'robot_clearance_m': LaunchConfiguration('robot_clearance_m'),
        }.items(),
    )

    delayed_navigation = TimerAction(
        period=4.0,
        actions=[nav2, semantic_navigation],
    )

    return LaunchDescription([*arguments, gazebo, delayed_navigation])

