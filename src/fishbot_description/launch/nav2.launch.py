import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 定义变量
    fishbot_pkg = FindPackageShare(package='fishbot_description').find('fishbot_description')
    nav2_pkg = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    
    # 2. 地图文件路径 (请修改为你实际保存的地图路径！！！)
    map_file = os.path.join('/home/weiyu', 'my_slam_map.yaml')
    
    # 3. 参数文件路径
    params_file = os.path.join('/home/weiyu/vscode_workspace/ros2_wp/src/fishbot_description/config', 'nav2_params.yaml')
    
    # 4. 是否使用仿真时间 (必须为 True)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # =======================================================
    # 5. 引用 Nav2 的标准启动流程
    # 它会自动启动 AMCL(定位), Planner(规划), Controller(控制), BT(行为树)
    # =======================================================
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true' # 自动进入活动状态
        }.items()
    )

    # 6. 启动 RViz (使用 Nav2 提供的配置，这很重要，能看到代价地图)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_pkg, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        nav2_bringup_launch,
        rviz_node
    ])