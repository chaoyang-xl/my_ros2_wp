import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'fishbot_description'
    urdf_name = "fishbot_base.urdf"

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    with open(urdf_model_path, 'r') as inf:
        robot_desc = inf.read()

    # 1. State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True  # 【必须设置为 True】
        }],
        output='screen'
    )
    # 2. 启动 Gazebo Server (后台计算 + 传感器插件)
    # 加上 -s 参数后，Gazebo 会在无界面模式下运行，但会加载传感器
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[
            'gz', 'sim', 
            '-r',        # 自动运行
            '-s',        # 服务器模式(无界面)
            
            # 【这里是唯一的修改点】
            # 使用绝对路径加载刚才创建的文件
            '/home/weiyu/turtlebot3_house.world' 
        ],
        output='screen'
    )

    # 3. 启动 Gazebo Client (图形界面)
    # 这个进程专门负责显示画面
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen'
    )

    # 4. 生成机器人
    spawn_entity_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'fishbot',
            '-topic', 'robot_description',
            '-x', '-2.0', '-y', '1.0', '-z', '0.1'
        ],
        output='screen'
    )

    # 5. 桥接
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 1. 雷达
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            
            # 2. 里程计
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            
            # 3. 控制指令
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            
            # 4. 时钟 (已验证通过)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            
            # 5. 【核心修复】TF 变换
            # Gazebo 里叫 /model/fishbot/tf，我们要把它桥接过来
            '/model/fishbot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            
            # joint state
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',

            # imu
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        output='screen',
        # 【关键一步】重映射！
        # 把 Gazebo 的长名字，改成 ROS 标准的 /tf
        remappings=[
            ('/model/fishbot/tf', '/tf')
        ]
    )

    # 6. RViz
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'fishbot.rviz')], # 如果你有配置文件的话
        parameters=[{'use_sim_time': True}], # 【新增】使用仿真时间
        output='screen',
    )
    # 7. Cartographer Node
    cartographer_node = Node(
    package='cartographer_ros',
    executable='cartographer_node',
    name='cartographer_node',
    parameters=[{'use_sim_time': True}],
    arguments=[
        '-configuration_directory',
        os.path.join(pkg_share, 'config'),
        '-configuration_basename',
        'fishbot_2d.lua'
    ],
    output='screen'
    )
    # 8. Cartographer Occupancy Grid Node
    occupancy_grid_node = Node(
    package='cartographer_ros',
    executable='cartographer_occupancy_grid_node',
    parameters=[{'use_sim_time': True}],
    arguments=['-resolution', '0.05'],
    output='screen'
    )


    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(start_gazebo_server_cmd)  # 启动服务器
    ld.add_action(start_gazebo_client_cmd)  # 启动画面
    ld.add_action(spawn_entity_cmd)
    ld.add_action(bridge_node)
    ld.add_action(rviz2_node)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)

    
    return ld