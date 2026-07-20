#!/usr/bin/env python3
"""Start the package-local Fishbot model in Gazebo Sim."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the standalone Fishbot Gazebo launch description."""
    package_share = get_package_share_directory('semantic_navigation_pkg')

    resource_path = str(Path(package_share) / 'models')
    default_world = str(Path(package_share) / 'worlds' / 'semantic_house.world')
    urdf_path = Path(package_share) / 'urdf' / 'fishbot_base.urdf'
    robot_description = urdf_path.read_text(encoding='utf-8')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')

    arguments = [
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('spawn_x', default_value='-2.0'),
        DeclareLaunchArgument('spawn_y', default_value='1.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.08'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
    ]

    # 1. Gazebo Server
    # 只负责仿真计算、物理、传感器、插件。
    gazebo_server = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '-r',
            '-s',
            world,
        ],
        output='screen',
    )

    # 2. Gazebo GUI Client
    # 单独启动 GUI，连接已经运行的 Gazebo server。
    # headless=true 时不启动 GUI。
    gazebo_client = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz',
                    'sim',
                    '-g',
                ],
                output='screen',
                condition=UnlessCondition(headless),
            )
        ],
    )

    # 3. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # 4. ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='fishbot_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/fishbot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/model/fishbot/tf', '/tf'),
        ],
    )

    # 5. Spawn Fishbot
    # 关键修复：不要再用 -topic robot_description。
    # 直接从 URDF 文件生成 Gazebo 模型，避免模型进入 server 但 visual 不进入 scene。
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_fishbot',
                output='screen',
                arguments=[
                    '-name', 'fishbot',
                    '-file', str(urdf_path),
                    '-x', LaunchConfiguration('spawn_x'),
                    '-y', LaunchConfiguration('spawn_y'),
                    '-z', LaunchConfiguration('spawn_z'),
                    '-Y', LaunchConfiguration('spawn_yaw'),
                ],
            )
        ],
    )

    return LaunchDescription([
        *arguments,
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        bridge,
        spawn_robot,
    ])