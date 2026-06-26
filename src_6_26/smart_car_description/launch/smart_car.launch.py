import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('smart_car_description')
    
    # 文件路径
    urdf_file = os.path.join(pkg_share, 'urdf', 'smart_car.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'smart_car.rviz')
    world_path = os.path.join(pkg_share, 'worlds', 'test_empty.world')
    #world_path = 'empty.sdf'
    # Launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    use_vision = LaunchConfiguration('use_vision', default='true')
    use_controller = LaunchConfiguration('use_controller', default='true')
    
    # 1. 启动 robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time
        }]
    )
    
    # 2. 启动 Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}',
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 3. 在 Gazebo 中生成机器人
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
    
    # 4. ROS-Gazebo 桥接
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        output='screen'
    )
    
    # 5. RVIZ2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    # 6. 视觉处理节点
    vision_processor_node = Node(
        package='smart_car_description',
        executable='vision_processor_node',
        name='vision_processor_node',
        condition=IfCondition(LaunchConfiguration('use_vision')),
        output='screen'
    )
    
    # 7. 运动控制节点
    motion_controller_node = Node(
        package='smart_car_description',
        executable='motion_controller_node',
        name='motion_controller_node',
        condition=IfCondition(LaunchConfiguration('use_controller')),
        output='screen'
    )
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RVIZ2'
        ),
        DeclareLaunchArgument(
            'use_vision',
            default_value='true',
            description='Start vision processor node'
        ),
        DeclareLaunchArgument(
            'use_controller',
            default_value='true',
            description='Start motion controller node'
        ),
        
        # 启动节点
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        bridge_node,
        #rviz_node,
        vision_processor_node,
        motion_controller_node
    ])
