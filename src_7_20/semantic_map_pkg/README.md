# semantic_map_pkg

独立 ROS 2 语义建图包，从检测 JSON 和深度图生成 `map` 坐标系语义对象。

## 离线bag测试流程 orangepi5plus环境
### 启动yolo
ros2 launch opi_yolo_rknn_recorder ai_recorder.launch.py \
  image_topic:=/camera/color/image_raw \
  output_dir:=/tmp/yolo_semantic_check \
  detect_model:=/home/weiyu/vscode_workspace/models/yolo11n.pt \
  pose_model:=/home/weiyu/vscode_workspace/models/yolo11n-pose.pt \
  enable_detect:=true \
  enable_pose:=true \
  frame_skip:=0 \
  save_every_n:=30

### 启动语义投影节点

ros2 launch semantic_map_pkg semantic_map.launch.py \
  use_loop_closure:=true \
  use_sim_time:=true \
  input_topic:=/yolo/results_json \
  depth_topic:=/camera/depth/image_raw \
  camera_frame:=camera_depth_optical_frame \
  camera_fx:=311.3878784 \
  camera_fy:=311.3878784 \
  camera_cx:=317.5 \
  camera_cy:=198.5 \
  depth_scale:=0.001 \
  min_confirmed_seen:=3 \
  show_candidates:=true \
  max_depth_time_diff_s:=0.3 \
  semantic_objects_path:=/tmp/semantic_01_objects.json
未吸附的原始节点投影：
  ros2 run semantic_map_pkg raw_seed_visualizer_node 

### 播放bag
```bash
ros2 launch semantic_map_pkg offline_cartographer_bag.launch.py \
  bag_uri:=/home/weiyu/vscode_workspace/ros2_wp/ros_bag/semantic_bags/semantic_01\
  rate:=1.0 \
  enable_rviz:=true
```

ros2 launch opi_yolo_rknn_recorder ai_recorder.launch.py image_topic:=/camera/color/image_raw output_dir:=/home/orangepi/yolo_results detect_model:=/home/orangepi/models/yolo11n_rknn_model pose_model:=/home/orangepi/models/yolo11n-pose_rknn_model enable_detect:=true enable_pose:=true imgsz:=640 conf:=0.15 http_port:=8088

## 核心链路

```text
/yolo/results_json + depth + TF
  -> frontend_bridge_node
  -> /semantic_seed
  -> semantic_projection_node 或 loop_closure_guard_node
  -> Marker + /semantic_objects + semantic_objects.json
```

本包从 `my_work_pkg` 提取核心闭环，原包保持不变。调试可视化、诊断节点、
轻量 tracker 和标准 `Detection2DArray` 备用入口未迁移。

## 构建

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select semantic_map_pkg
source install/setup.bash
```

## 启动

```bash
ros2 launch semantic_map_pkg semantic_map.launch.py \
  input_topic:=/yolo/results_json \
  depth_topic:=/camera/depth_image \
  camera_frame:=camera_optical_link \
  camera_fx:=609.818 camera_fy:=609.572 \
  camera_cx:=639.986 camera_cy:=362.435 \
  depth_scale:=0.001
```

启用 SLAM 回环修正：

```bash
ros2 launch semantic_map_pkg semantic_map.launch.py \
  use_loop_closure:=true \
  use_sim_time:=false \
  camera_frame:=camera_optical_link \
  depth_scale:=0.001 \
  lc_stable_checks:=2
```

回环发生后，对象先做 TF 几何修正；静态 `snapped` 对象会等待新 `/map`，并在
`map<-odom` 连续稳定后才 re-snap，避免使用回环前缓存地图。

启动文件内是仿真相机默认值。实机必须使用标定内参。毫米深度通常设置
`depth_scale:=0.001`，米制深度设置 `depth_scale:=1.0`。

## 输入输出

| 方向 | 名称 | 类型 |
| --- | --- | --- |
| 输入 | `/yolo/results_json` | `std_msgs/msg/String` |
| 输入 | `/camera/depth_image` | `sensor_msgs/msg/Image` |
| 输入 | `/map` | `nav_msgs/msg/OccupancyGrid` |
| 输入 | `/tf`、`/tf_static` | TF |
| 中间 | `/semantic_seed` | `std_msgs/msg/String` |
| 输出 | `/semantic_objects` | `std_msgs/msg/String` |
| 输出 | `~/markers` | `visualization_msgs/msg/MarkerArray` |
| 输出 | `/tmp/semantic_objects.json` | confirmed 对象快照 |

详细接口、类别策略、对象生命周期和回环规则见
[semantic_objects_interface.md](semantic_objects_interface.md)。

使用真实 rosbag 重建 Cartographer 地图时，参见
[ROSBAG_OFFLINE_TEST.md](ROSBAG_OFFLINE_TEST.md)。该文档记录了已验证的 bag 话题、
frame、相机内参、离线 Launch、地图保存步骤以及 RGB-D 未注册限制。

包含轮式里程计、分离式加速度/陀螺仪和已注册 RGB-D 的 `semantic_05` 完整流程见
`semantic_map_offline/SEMANTIC_05_BAG_TEST.md`。本包提供
`imu_sync_node` 和 `offline_ros_bag_with_odom_imu.lua` 作为该流程的 SLAM 输入层。

## 测试

吸附前原始点可视化：

```bash
ros2 launch semantic_map_pkg semantic_map.launch.py \
  use_loop_closure:=true \
  use_sim_time:=true \
  enable_debug_viz:=true
```

RViz MarkerArray 话题：

```text
/raw_seed_visualizer_node/markers
```

`show_candidates` 显示吸附后的 candidate；`enable_debug_viz` 显示吸附前的
`/semantic_seed`，二者含义不同。

```bash
colcon test --packages-select semantic_map_pkg
colcon test-result --verbose
```

本包不依赖 `my_work_pkg`，可以直接作为独立 Git 仓库维护。

## License

Apache-2.0，见 [LICENSE](LICENSE)。
