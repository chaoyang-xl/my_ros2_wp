# 1  slam建图

## 键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard

## gazebo环境:
ros2 launch fishbot_description display_rviz2.launch.py

## cartographer建图
 ros2 launch cartographer_test cartographer.launch.py 


# 2  使用NAV2导航时先打开gazebo环境获取数据再打开rviz进行acml定位和导航
	ros2 launch fishbot_description display_rviz2.launch.py 
	ros2 launch fishbot_description nav2.launch.py 
