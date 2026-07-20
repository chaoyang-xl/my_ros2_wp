# Rosbag 离线 Cartographer 测试

## 1. 已检查的数据

测试 bag：

```text
/home/weiyu/vscode_workspace/ros2_wp/ros_bag/semantic_bags/semantic_01
```

| 项目 | 结果 |
| --- | --- |
| 存储格式 | sqlite3，单个约 2.4 GiB 的 db3 |
| 时长 | 336.18 秒 |
| 消息数 | 110635 |
| LaserScan | `/scan`，frame `laser`，约 12 Hz |
| Odometry | `/odom`，`odom -> base_link` |
| RGB | 640×480，`camera_color_optical_frame` |
| Depth | 640×400，`16UC1`，`camera_depth_optical_frame` |
| 已录制地图 | `/map`，离线重建时明确不播放 |
| 已录制动态 TF | `/tf`，离线重建时明确不播放 |

深度相机内参：

```text
fx = 311.3878784
fy = 311.3878784
cx = 317.5
cy = 198.5
depth_scale = 0.001
```

RGB 相机内参：

```text
fx = 365.1741638
fy = 365.4214478
cx = 318.2763062
cy = 243.8037720
```

## 2. 离线建图资源

本包已经包含：

- `launch/offline_cartographer_bag.launch.py`
- `config/cartographer/offline_ros_bag_with_odom.lua`
- Cartographer Lua include 依赖
- `urdf/rosbag_robot.urdf`
- `config/offline_mapping.rviz`

Launch 只播放：

```text
/scan
/odom
/tf_static
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_raw
/camera/depth/camera_info
```

`/map` 和动态 `/tf` 由新的 Cartographer 运行生成，避免混入 bag 中的旧建图结果。

## 3. 构建

```bash
cd /home/weiyu/vscode_workspace/ros2_wp
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select semantic_map_pkg
source install/setup.bash
```

## 4. 启动离线 Cartographer

```bash
ros2 launch semantic_map_pkg offline_cartographer_bag.launch.py \
  bag_uri:=/home/weiyu/vscode_workspace/ros2_wp/ros_bag/semantic_bags/semantic_01
```

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `rate` | `1.0` | rosbag 播放倍率 |
| `start_offset` | `0.0` | 从 bag 起点后多少秒开始 |
| `bag_delay_s` | `3.0` | 等待 TF 和 Cartographer 启动的时间 |
| `resolution` | `0.05` | OccupancyGrid 分辨率 |
| `enable_rviz` | `true` | 是否加载包内 RViz 配置 |
| `configuration_basename` | `offline_ros_bag_with_odom.lua` | Cartographer 入口配置 |

建议首次完整测试保持 `rate:=1.0`。高倍率播放会增加处理压力，不适合作为地图质量基准。

## 5. 观察与保存

确认新地图正在发布：

```bash
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
```

bag 播放完成后，保持 Launch 运行，在另一个终端保存 OccupancyGrid：

```bash
mkdir -p /home/weiyu/maps/semantic_01
ros2 run nav2_map_server map_saver_cli \
  -f /home/weiyu/maps/semantic_01/semantic_01
```

需要保留 Cartographer 状态用于继续优化或定位时：

```bash
ros2 service call /finish_trajectory \
  cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

ros2 service call /write_state \
  cartographer_ros_msgs/srv/WriteState \
  "{filename: '/home/weiyu/maps/semantic_01/semantic_01.pbstream', include_unfinished_submaps: true}"
```

## 6. 语义建图前的重要限制

这个 bag 的 RGB 与深度图没有处于同一像素模型：

```text
RGB:   640×480，独立 RGB 内参
Depth: 640×400，独立 depth 内参
```

因此不能把 RGB 检测框坐标直接用于索引 `/camera/depth/image_raw`。这样虽然程序可能
运行，但目标深度和最终 map 坐标存在系统性误差。

正式测试完整语义链之前，应先使用 `depth_image_proc` 或等价算法，将深度注册到
RGB optical frame，再让 YOLO 和 `frontend_bridge_node` 使用同一分辨率、同一内参。
当前机器的软件源中存在 `ros-jazzy-depth-image-proc`，但尚未安装，本次没有自动修改
系统软件。

## 7. 已完成的短时验证

隔离 ROS domain 中已经验证：

- Cartographer Lua 及全部 include 文件可解析。
- `robot_state_publisher`、Cartographer 和 bag player 均正常启动。
- 新 `/map` 成功生成，测试时分辨率 0.05 m、尺寸 130×194。
- `map -> odom` TF 正常发布。
- bag 中旧 `/map` 和动态 `/tf` 未参与重建。

Cartographer 日志会出现少量 `Dropped earlier points`。bag 中 LaserScan 的
`scan_time` 约为 0.08359 秒，点时间跨度与相邻 scan 时间存在轻微重叠。当前配置使用
`num_subdivisions_per_laser_scan = 1`，短时测试仍能稳定生成地图；首次完整测试应重点
观察最终地图质量，再决定是否增加 scan 时间清洗节点。

## 8. 不依赖 RGB-D 注册的核心算法测试

这项测试用于验证：

```text
栅格吸附 -> candidate/confirmed -> TF jump 几何修正
-> 等新 /map 和 TF 稳定 -> snapped 对象 re-snap
```

它直接发布 `/semantic_seed`，因此可以使用当前 bag 验证回环机制，不受 RGB 与深度
尚未注册的影响。

启动第 4 节的离线 Cartographer 后，在第二个终端启动回环节点：

```bash
source /home/weiyu/vscode_workspace/ros2_wp/install/setup.bash

ros2 launch semantic_map_pkg semantic_map.launch.py \
  use_loop_closure:=true \
  use_sim_time:=true \
  min_confirmed_seen:=3 \
  show_candidates:=true \
  semantic_objects_path:=/tmp/semantic_01_objects.json
```

在 RViz 使用 `Publish Point` 点击椅子或其他独立占据结构附近，然后读取坐标：

```bash
ros2 topic echo /clicked_point --once
```

把下面的 `gx`、`gy` 替换为点击坐标，有限次数发布一个静态对象：

```bash
ros2 topic pub --rate 5 --times 5 /semantic_seed std_msgs/msg/String \
  "{data: '{\"label\":\"chair\",\"confidence\":0.95,\"gx\":1.0,\"gy\":2.0}'}"
```

检查对象和日志：

```bash
ros2 topic echo /semantic_objects
cat /tmp/semantic_01_objects.json
```

预期现象：

- 第 3 次有效观测后对象变为 `confirmed`。
- Marker 文本包含 `source=snapped`。
- Cartographer 出现足够大的 `map<-odom` 修正时，日志先输出 `TF jump detected` 和
  `pending_resnap=1`。
- 收到 jump 后的新 `/map`，并且 TF 连续稳定两次后，日志输出
  `Deferred re-snap round`；失败对象会在后续新地图帧继续尝试。
- re-snap 前后 `times_seen`、`confidence`、`state` 和姿态属性保持不变。

人工 seed 只发布有限次数，避免回环后新观测立即覆盖修正结果。
