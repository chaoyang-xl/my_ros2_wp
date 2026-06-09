# 测试流程

1、运行cartographer建图流程
ros2 launch fishbot_description cartographer.launch.py 
ros2 launch fishbot_description display_rviz2.launch.py 
ros2 run teleop_twist_keyboard teleop_twist_keyboard


2、运行建图流程

ros2 run my_work_pkg semantic_projection_node
测试数据：
ros2 topic pub /semantic_seed std_msgs/msg/String "data: '{\"label\": \"bin\", \"confidence\": 0.9, \"gx\": -2.6, \"gy\": 3.8}'" -1


ros2 launch orbbec_camera gemini_330_series.launch.py enable_sync:=true enable_accel:=true enable_gyro:=true depth_fps:=15 depth_registration:=true