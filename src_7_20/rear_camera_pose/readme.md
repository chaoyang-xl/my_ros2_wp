## 模式 1：V4L2 直连 (本地 USB 测试，默认)
### 直接用 /dev/video2
ros2 launch rear_camera_pose rear_pose.launch.py

### 指定其他摄像头
ros2 launch rear_camera_pose rear_pose.launch.py camera_device:=/dev/video0

### 或使用脚本
cd src/rear_camera_pose && ./scripts/run_rear_pose.sh /dev/video2


## 模式 2：ROS topic 订阅 (板端，配合任何 camera driver)

ros2 launch rear_camera_pose rear_pose.launch.py \
  camera_device:='' \
  image_topic:=/usb_cam/image_raw
