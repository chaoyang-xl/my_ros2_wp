import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('smart_car_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'smart_car.urdf.xacro')
    
    # 假设你把 world 文件放在了 smart_car_description/worlds/ 目录下
    # 如果你是直接用系统的路径，请替换这里的 world_path
    #world_path = os.path.join(pkg_share, 'worlds', 'turtlebot3_house.world')
    world_path = os.path.join(pkg_share, 'worlds', 'test_empty.world')
    # 1. 启动 robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
    )

    # 2. 启动 Gazebo Sim 并加载特定的 world 文件
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )
    # 3. 将 URDF 模型生成到 Gazebo 世界中
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'smart_car',
            '-x', '-2.0', '-y', '1.0', '-z', '0.1',
        ],
        output='screen'
    )

    # 4. 关键：启动 ros_gz_bridge，打通两个世界的话题
    # 这里桥接了 cmd_vel (控制指令), odom (里程计), camera (图像)
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        bridge_node
    ])