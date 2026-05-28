终端 1：启动相机

Bash
ros2 launch orbbec_camera gemini_330_series.launch.py enable_sync:=true enable_imu:=true enable_accel:=true enable_gyro:=true depth_fps:=30

终端 2：运行 IMU 合并脚本

Bash
cd ~/vscode_workspace/ros2_wp
python3 imu_merger.py

终端 3：启动点云转激光（压扁点云）
(如果你还没有安装，先 sudo apt install ros-jazzy-pointcloud-to-laserscan)

Bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
--ros-args -r cloud_in:=/camera/depth/points \
-p target_frame:=camera_depth_optical_frame \
-p min_height:=-1.0 -p max_height:=1.0 \
-p range_min:=0.3 -p range_max:=6.0 \
-p qos_overrides./scan.reliability:=reliable


ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
--ros-args -r cloud_in:=/camera/depth/points \
-p target_frame:=camera_link \
-p min_height:=-0.3 \
-p max_height:=0.3 \
-p range_min:=0.3 \
-p range_max:=6.0
终端 4：启动 Cartographer

Bash
cd ~/vscode_workspace/ros2_wp/src/launch
ros2 launch cartographer_handheld.launch.py



按照以下步骤启动 V2版本 雷达话题/scan需要订阅持续激活 
ros2 launch orbbec_camera gemini_330_series.launch.py enable_sync:=true enable_accel:=true enable_gyro:=true depth_fps:=15

ros2 topic echo --qos-reliability best_effort /camera/depth/points | grep frame_id


ros2 topic echo --qos-reliability best_effort /scan

ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
--ros-args -r cloud_in:=/camera/depth/points \
-p target_frame:=camera_link \
-p min_height:=-0.3 -p max_height:=0.3 \
-p range_min:=0.3 -p range_max:=6.0

ros2 run gemini_336_cartographer_pkg imu_merger_node 

ros2 launch gemini_336_cartographer_pkg cartographer_handheld.launch.py

ros2 daemon stop
killall -9 ros2 python3 rviz2


#V3启动顺序
##1、启动相机驱动节点
ros2 launch orbbec_camera gemini_330_series.launch.py enable_sync:=true enable_accel:=true enable_gyro:=true depth_fps:=15

##2、启动点云转雷达帧节点
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
--ros-args -r cloud_in:=/camera/depth/points \
-p target_frame:=camera_link \
-p min_height:=-0.3 -p max_height:=0.3 \
-p range_min:=0.3 -p range_max:=6.0

##3、启动cartographer节点（包含RVIZ，/scan激活节点）
ros2 launch gemini_336_cartographer_pkg cartographer_handheld.launch.py