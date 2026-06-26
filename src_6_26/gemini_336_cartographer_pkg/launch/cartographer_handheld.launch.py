import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ==== 请修改为你的 .lua 文件实际所在的绝对路径 ====
    config_dir = '/home/weiyu/vscode_workspace/ros2_wp/src/gemini_336_cartographer_pkg/config'
    config_basename = 'gemini_handheld.lua'
    
    return LaunchDescription([
        # 1. 启动 Cartographer 核心节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}], # 真实相机测绘不用 sim_time
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', config_basename
            ],
            remappings=[
                ('/scan', '/scan'),          # pointcloud_to_laserscan 默认发布 /scan
                ('/imu', '/camera/imu')      # 将算法所需的 /imu 映射到我们脚本发布的 /camera/imu
            ]
        ),
        
        # 2. 启动占据栅格地图节点 (用于在 RViz 里显示 2D 地图)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'resolution': 0.05          # 地图分辨率 5cm
            }]
        ),
        # 3. 启动RViz2节点 (可选，如果你想在 RViz 中可视化地图和轨迹)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(config_dir, 'gemini_330_series.rviz')]  # RViz 配置文件路径
        ), 
        # 4. 启动 IMU 合并节点
        Node(
            package='gemini_336_cartographer_pkg',
            executable='imu_merger_node',
            name='imu_merger_node',
            output='screen'
        ),
        # 5. 启动系统起搏器节点，保持相机和雷达持续苏醒
        Node(
            package='gemini_336_cartographer_pkg',
            executable='pacemaker_node',
            name='pacemaker_node',
            output='screen'
        ),

    ])