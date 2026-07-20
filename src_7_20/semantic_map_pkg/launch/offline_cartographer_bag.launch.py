#!/usr/bin/env python3
"""Rebuild a Cartographer map from the recorded robot-calibration bag."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start TF, Cartographer, occupancy-grid generation and bag playback."""
    package_share = Path(get_package_share_directory('semantic_map_pkg'))
    config_dir = package_share / 'config' / 'cartographer'
    default_urdf = package_share / 'urdf' / 'rosbag_robot.urdf'
    default_rviz = package_share / 'config' / 'offline_mapping.rviz'

    arguments = [
        DeclareLaunchArgument(
            'bag_uri',
            description='Absolute path to a rosbag2 directory',
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='offline_ros_bag_with_odom.lua',
        ),
        DeclareLaunchArgument('urdf_path', default_value=str(default_urdf)),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('start_offset', default_value='0.0'),
        DeclareLaunchArgument('bag_delay_s', default_value='3.0'),
        DeclareLaunchArgument('play_color', default_value='true'),
        DeclareLaunchArgument(
            'play_color_info',
            default_value='true',
            description='Replay RGB camera_info independently of the RGB image',
        ),
        DeclareLaunchArgument('play_depth', default_value='true'),
        DeclareLaunchArgument('play_imu', default_value='false'),
        DeclareLaunchArgument(
            'enable_imu_sync',
            default_value='false',
            description='Merge split accel/gyro streams and align device stamps',
        ),
        DeclareLaunchArgument(
            'publish_robot_state',
            default_value='true',
            description='Publish static TF from urdf_path; disable when the bag contains robot TF',
        ),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=str(default_rviz)),
    ]

    def launch_setup(context):
        urdf_path = Path(LaunchConfiguration('urdf_path').perform(context))
        robot_description = urdf_path.read_text(encoding='utf-8')

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 30.0,
                'use_sim_time': True,
            }],
            condition=IfCondition(LaunchConfiguration('publish_robot_state')),
        )

        cartographer = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', str(config_dir),
                '-configuration_basename',
                LaunchConfiguration('configuration_basename'),
            ],
            parameters=[{'use_sim_time': True}],
        )

        occupancy_grid = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            arguments=[
                '-resolution', LaunchConfiguration('resolution'),
                '-publish_period_sec', '1.0',
            ],
            parameters=[{'use_sim_time': True}],
        )

        imu_sync = Node(
            package='semantic_map_pkg',
            executable='imu_sync_node',
            name='semantic_bag_imu_sync',
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('enable_imu_sync')),
        )

        rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('enable_rviz')),
        )

        # Deliberately omit the recorded /map and dynamic /tf.  Cartographer
        # must generate both from /scan and /odom during this replay.
        bag_topics = ['/scan', '/odom', '/tf_static']
        if LaunchConfiguration('play_color').perform(context).lower() == 'true':
            bag_topics.append('/camera/color/image_raw')
        if LaunchConfiguration('play_color_info').perform(context).lower() == 'true':
            bag_topics.append('/camera/color/camera_info')
        if LaunchConfiguration('play_depth').perform(context).lower() == 'true':
            bag_topics.extend([
                '/camera/depth/image_raw',
                '/camera/depth/camera_info',
            ])
        if LaunchConfiguration('play_imu').perform(context).lower() == 'true':
            bag_topics.extend([
                '/camera/accel/sample',
                '/camera/gyro/sample',
            ])
        bag_play = ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play', LaunchConfiguration('bag_uri'),
                '--clock',
                '--rate', LaunchConfiguration('rate'),
                '--start-offset', LaunchConfiguration('start_offset'),
                '--topics',
                *bag_topics,
            ],
            output='screen',
        )

        delayed_bag = TimerAction(
            period=LaunchConfiguration('bag_delay_s'),
            actions=[bag_play],
        )
        return [
            robot_state_publisher,
            cartographer,
            occupancy_grid,
            imu_sync,
            rviz,
            delayed_bag,
        ]

    return LaunchDescription([*arguments, OpaqueFunction(function=launch_setup)])
