# Meeting_Room ROS1 bag test

## 数据概况

原始文件：

```text
/home/weiyu/vscode_workspace/ros2_wp/ros_bag/Meeting_Room/rosbag/car_meetingroom_mapping.bag
```

它是 ROS1 bag V2，约 885 MiB、74.895 秒，共 6317 条消息：

| Topic | 类型 | 数量 | 说明 |
| --- | --- | ---: | --- |
| `/fastlio_odom` | `nav_msgs/Odometry` | 749 | `odom -> lidar_link` 位姿 |
| `/orbbec_camera/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | 1829 | 1280x720 RGB |
| `/orbbec_camera/depth/image_raw/compressedDepth` | `sensor_msgs/CompressedImage` | 1909 | 1280x720、16UC1、毫米深度 |
| `/orbbec_camera/color/camera_info` | `sensor_msgs/CameraInfo` | 1830 | RGB-D 共用光学帧标定 |

bag 不包含 `/tf`、`/tf_static`、激光、IMU 或二维地图，因此本测试不启动
Cartographer。对象地图固定坐标系为 `odom`。

相机内参来自 bag：

```text
fx=610.5088500976562  fy=610.8049926757812
cx=638.3673095703125  cy=365.3813171386719
```

## ROS2 转换

当前已转换到：

```text
/home/weiyu/vscode_workspace/ros2_wp/ros_bag/Meeting_Room/rosbag2/car_meetingroom_mapping
```

重新转换时安装 `rosbags`，原始 bag 不会被修改：

```bash
python3 -m pip install --user --break-system-packages rosbags

rosbags-convert \
  --src /home/weiyu/vscode_workspace/ros2_wp/ros_bag/Meeting_Room/rosbag/car_meetingroom_mapping.bag \
  --dst /home/weiyu/vscode_workspace/ros2_wp/ros_bag/Meeting_Room/rosbag2/car_meetingroom_mapping \
  --dst-storage mcap \
  --dst-typestore ros2_jazzy
```

## 完整测试

该入口使用原 `opi_yolo_rknn_recorder/ai_recorder_node`，不是旧 JSONL：

```bash
cd /home/weiyu/vscode_workspace/ros2_wp
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch semantic_map_offline meeting_room_sam_test.launch.py \
  rate:=0.15 \
  output_directory:=/tmp/meeting_room_sam_output \
  snapshot_path:=/tmp/meeting_room_sam_output/semantic_objects.json \
  enable_rviz:=true
```

默认仍以 `pixel_stride=1` 对 SAM mask 逐像素投影，但在发布 ROS PointCloud2 前按
`voxel_size=0.01` 选择每个体素的一个代表点。融合本身也是 1 cm 体素，因此该步骤
不会降低最终对象地图的有效分辨率，同时可避免单帧 8-14 万点、3-5 MB 的 DDS 消息
导致 `/semantic_offline/points` 丢失或融合节点长期积压。性能不足时先将 `rate` 降到
`0.10`。

Meeting Room 入口还默认设置 `projection_cluster_eps=0.03`、
`projection_cluster_min_points=10`：每个 SAM 检测在发布前独立做一次空间聚类，只保留
最大的连续点簇。该步骤用于剔除 mask 边缘混入的墙面、地面和错误深度，不参与跨帧
关联。元数据中的 `raw_point_count`、`voxel_point_count`、`point_count` 和
`cluster_removed_count` 可用于判断每个框经过各阶段后保留了多少点。

查看原 YOLO 和 SAM mask：

```bash
ros2 run rqt_image_view rqt_image_view /yolo/debug_image
ros2 run rqt_image_view rqt_image_view /semantic_offline/sam_debug_image
```

RViz 默认固定帧为 `odom`，显示当前帧点云、融合点云、对象属性 marker 和
`/meeting_room/trajectory`。

## 外参限制

bag 没有记录 `lidar_link -> orbbec_camera_color_optical_frame` 外参。默认暂时使用
标准 ROS 光学轴旋转和零平移：

```text
translation = [0, 0, 0]
quaternion xyzw = [-0.5, 0.5, -0.5, 0.5]
```

拿到真实标定后可直接覆盖：

```bash
ros2 launch semantic_map_offline meeting_room_sam_test.launch.py \
  camera_x:=... camera_y:=... camera_z:=... \
  camera_qx:=... camera_qy:=... camera_qz:=... camera_qw:=...
```

在真实外参补齐之前，单帧对象形状和 SAM mask 可以用于验证；跨帧融合位置、全局
朝向以及最终导航 JSON 不能作为定量结果。输出 JSON 的 `frame_id` 是 `odom`，用于
导航前还需要转换到导航系统使用的 `map`。
