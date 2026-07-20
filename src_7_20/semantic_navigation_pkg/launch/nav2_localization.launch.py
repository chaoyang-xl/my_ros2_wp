#!/usr/bin/env python3
"""Start Nav2 and AMCL using package-local Fishbot parameters."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the standalone Nav2 localization launch description."""
    package_share = get_package_share_directory('semantic_navigation_pkg')
    nav2_share = get_package_share_directory('nav2_bringup')
    default_params = str(Path(package_share) / 'config' / 'nav2_params.yaml')
    default_rviz = str(Path(package_share) / 'config' / 'semantic_navigation.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    arguments = [
        DeclareLaunchArgument(
            'map',
            description='Absolute path to occupancy map YAML',
        ),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
    ]

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(nav2_share) / 'launch' / 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'False',
            'use_localization': 'True',
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': use_sim_time,
            'autostart': LaunchConfiguration('autostart'),
            'use_composition': 'False',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )

    return LaunchDescription([*arguments, nav2, rviz])

