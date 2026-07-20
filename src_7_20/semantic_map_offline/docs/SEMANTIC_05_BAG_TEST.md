# semantic_05 rosbag test

## 数据检查结果

Bag 目录：

```text
/home/weiyu/vscode_workspace/ros2_wp/ros_bag/semantic_05
```

| 数据 | 话题与结果 |
| --- | --- |
| 时长 | 约 160.29 秒，sqlite3，约 2.4 GiB |
| 激光 | `/scan`，frame `laser`，约 12 Hz |
| 轮式里程计 | `/odom`，`odom -> base_link`，约 50 Hz |
| 加速度计 | `/camera/accel/sample`，约 202.7 Hz |
| 陀螺仪 | `/camera/gyro/sample`，约 202.7 Hz |
| RGB | 640x480，`camera_color_optical_frame`，约 10 Hz |
| 深度 | 640x480、`16UC1`，已注册到 RGB optical frame，约 10 Hz |
| 静态 TF | 包含 `base_link -> laser`、`base_link -> camera_link` 和相机内部 TF |

RGB 与深度共用的相机模型：

```text
fx = 365.1741638183594
fy = 365.42144775390625
cx = 318.27630615234375
cy = 243.80377197265625
depth_scale = 0.001
```

该 bag 的深度已经和 RGB 对齐，不需要启动 `depth_image_proc`。专用 Launch 直接将
RGB 检测框用于 `/camera/depth/image_raw`。

专用 Launch 始终回放 RGB，包括 `run_yolo:=false` 的第二阶段。投影节点按原图像素为
每个三维点提取相机 RGB，最终 PLY、NPZ 和 ROS PointCloud2 都是真彩色。包内
YOLO-World 使用室内开放词汇列表，并在推理端按名称排除
`person,wall,floor,ceiling,unknown`。类别 0 是 `chair`，不能按 COCO 规则过滤 ID 0。

## SLAM 数据处理

相机把角速度和线加速度分别录在两个 `sensor_msgs/msg/Imu` 话题中，而且 header
使用设备时钟。`semantic_map_pkg/imu_sync_node.py` 会：

1. 按最近时间配对加速度与角速度；
2. 依据 rosbag `/clock` 将设备时间平移到 ROS 时间；
3. 发布完整、单调递增的 `/imu`；
4. 由 Cartographer 以约 101 Hz 采样使用。

Cartographer 配置为 `scan + wheel odometry + IMU`，入口文件是
`offline_ros_bag_with_odom_imu.lua`。建图时不播放 bag 中的动态 `/tf`，避免录制的
`odom -> base_link` 与 Cartographer 输出冲突；静态 `/tf_static` 继续播放。该 bag 的
静态 TF 比包内通用 URDF 更准确，因此专用 Launch 默认关闭 `robot_state_publisher`。

## 构建

工作空间内存在 Cartographer 源码但当前使用系统安装版本时，应忽略源码包：

```bash
cd /home/weiyu/vscode_workspace/ros2_wp
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-ignore cartographer cartographer_ros cartographer_ros_msgs \
  --packages-select semantic_map_pkg semantic_map_offline \
  --symlink-install

source install/setup.bash
```

## 推荐的两阶段流程

两阶段运行可以避免 YOLO 推理、SLAM 和点云融合同时争用计算资源，也能保证离线处理
可重复。每次测试应使用新的 YOLO 和对象输出目录，因为 JSONL 会追加、对象文件也可能
保留旧结果。

### 第一阶段：生成 YOLO JSONL

终端 1：

```bash
source /opt/ros/jazzy/setup.bash
source /home/weiyu/vscode_workspace/ros2_wp/install/setup.bash

ros2 run semantic_map_offline yolo_world_recorder_node --ros-args \
  -p image_topic:=/camera/color/image_raw \
  -p output_dir:=/tmp/semantic_05_world_yolo \
  -p detect_model:=/home/weiyu/vscode_workspace/models/yolov8s-world.pt \
  -p clip_model:=/home/weiyu/vscode_workspace/models/clip/ViT-B-32.pt \
  -p classes_path:=$(ros2 pkg prefix semantic_map_offline)/share/semantic_map_offline/config/class_list/gpt_indoor_general.txt \
  -p device:=0 \
  -p conf:=0.35 \
  -p frame_skip:=0
```

终端 2：

```bash
source /opt/ros/jazzy/setup.bash
source /home/weiyu/vscode_workspace/ros2_wp/install/setup.bash

ros2 bag play /home/weiyu/vscode_workspace/ros2_wp/ros_bag/semantic_05 \
  --clock \
  --topics /camera/color/image_raw
```

检测结果位于 `/tmp/semantic_05_world_yolo/results.jsonl`。记录中的时间来自 RGB 图像
`header.stamp`，可以和第二阶段的深度帧匹配。

### 第二阶段：SLAM、投影与对象融合

```bash
source /opt/ros/jazzy/setup.bash
source /home/weiyu/vscode_workspace/ros2_wp/install/setup.bash

ros2 launch semantic_map_offline semantic_05_bag_test.launch.py \
  run_yolo:=false \
  jsonl_path:=/tmp/semantic_05_world_yolo/results.jsonl \
  output_directory:=/tmp/semantic_05_output \
  snapshot_path:=/tmp/semantic_05_output/semantic_objects.json \
  rate:=0.5 \
  enable_rviz:=true
```

专用 Launch 使用 bbox 内点云，不使用 SAM。深度处理延迟五帧，等待对应时间的
Cartographer TF 可用，但变换仍使用深度图自身的原始时间戳。

bbox 流程当前默认 `pixel_stride=2`、`voxel_size=0.015`，比原来的每 4 像素取一点和
3 cm 体素保留更多细节。

## MobileSAM 高密度流程

SAM 使用独立 Launch，不会改写 bbox 流程：

```bash
source /opt/ros/jazzy/setup.bash
source /home/weiyu/vscode_workspace/ros2_wp/install/setup.bash

ros2 launch semantic_map_offline semantic_05_sam_bag_test.launch.py \
  run_yolo:=false \
  jsonl_path:=/tmp/semantic_05_world_yolo/results.jsonl \
  output_directory:=/tmp/semantic_05_sam_output \
  snapshot_path:=/tmp/semantic_05_sam_output/semantic_objects.json \
  rate:=0.25 \
  enable_rviz:=true
```

需要在一次回放中同时运行 YOLO、SAM 和融合，并把结果保存在持久目录时：

```bash
OUTPUT_ROOT=$HOME/semantic_map_outputs/semantic_05_sam_full
mkdir -p "$OUTPUT_ROOT"

ros2 launch semantic_map_offline semantic_05_sam_bag_test.launch.py \
  run_yolo:=true \
  yolo_output_dir:="$OUTPUT_ROOT/yolo" \
  save_yolo_images:=true \
  yolo_save_every_n:=1 \
  yolo_image_jpeg_quality:=90 \
  publish_yolo_debug_image:=true \
  output_directory:="$OUTPUT_ROOT/map" \
  snapshot_path:="$OUTPUT_ROOT/map/semantic_objects.json" \
  rate:=0.25 enable_rviz:=true
```

检测图保存在 `$OUTPUT_ROOT/yolo/frames/`，文件名包含原始 RGB 消息时间戳。设置
`save_yolo_images:=false` 可完全停止写图；设置 `publish_yolo_debug_image:=false` 可停止
发布 `/yolo/debug_image`，两者互不影响。

默认资源和参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `save_yolo_images` | `true` | 是否保存YOLO标注图 |
| `yolo_save_every_n` | `30` | 每N次推理归档一张；全量图片设为1 |
| `yolo_image_jpeg_quality` | `90` | 标注图JPEG质量 |
| `publish_yolo_debug_image` | `true` | 是否发布`/yolo/debug_image` |
| `sam_checkpoint` | `/home/weiyu/vscode_workspace/models/mobile_sam.pt` | MobileSAM 权重 |
| `sam_source` | 包内 `MobileSAM` 源码目录 | Python 源码入口 |
| `sam_device` | `cuda` | 当前机器已验证 GPU 可用 |
| `pixel_stride` | `1` | mask 内逐像素生成深度点 |
| `voxel_size` | `0.01` | 融合对象使用 1 cm 体素 |
| `mask_erode_px` | `1` | 轻微收缩 mask，减少边缘背景 |
| `rate` | `0.25` | 给 SAM 和高密度融合留出处理时间 |
| `publish_debug_image` | `true` | 发布SAM分割叠加图 |
| `publish_markers` | `true` | 发布融合对象3D框和track属性 |

当前系统 Python 需要安装 `timm`；Torch 和 Torchvision 已存在时：

```bash
python3 -m pip install --user --break-system-packages timm
```

短测使用同一份 109 帧 YOLO JSONL。旧 bbox 配置得到 5752 个融合点；SAM 高密度配置
得到 38622 个融合点，约为 6.7 倍，并保留 7 个对象的相机 RGB。输出位于
`/tmp/semantic_05_sam_smoke_20260714`，可直接查看：

```bash
conda run -n semantic_map python \
  scripts/view_tracked_objects_3d.py \
  --objects-dir /tmp/semantic_05_sam_smoke_20260714/objects \
  --min-observations 1 \
  --show-boxes \
  --show-origin
```

若完整 bag 处理速度不足，先将 `rate` 降到 `0.15`；只有显存或文件规模压力明显时，
再把 `pixel_stride` 调为 `2` 或 `voxel_size` 调为 `0.015`。

## 快速单阶段预览

```bash
ros2 launch semantic_map_offline semantic_05_bag_test.launch.py \
  run_yolo:=true \
  detect_model:=/home/weiyu/vscode_workspace/models/yolov8s-world.pt \
  clip_model:=/home/weiyu/vscode_workspace/models/clip/ViT-B-32.pt \
  output_directory:=/tmp/semantic_05_preview \
  snapshot_path:=/tmp/semantic_05_preview/semantic_objects.json \
  rate:=0.5
```

该模式适合快速观察，不建议把结果作为最终离线产物。

## 查看结果

专用 SAM launch 默认打开 `config/semantic_mapping_debug.rviz`。Fixed Frame 为
`map`，四层信息会同时更新：

```text
/map
/semantic_offline/points          当前帧投影
/semantic_offline/fused_points    累计对象地图
/semantic_offline/object_markers  track边界和属性
```

为两个 `PointCloud2` 显示选择 `RGB8` Color Transformer。

另开终端查看 MobileSAM 的 mask 是否准确包住物体：

```bash
source /opt/ros/jazzy/setup.bash
source /home/weiyu/vscode_workspace/ros2_wp/install/setup.bash
ros2 run rqt_image_view rqt_image_view /semantic_offline/sam_debug_image
```

调试顺序建议先看 mask，再看当前帧点云，最后看融合点云。前两项正确而融合结果
混乱时，问题集中在对象关联；当前帧已经错位时，应优先检查深度注册、内参和 TF。

检查运行状态：

```bash
ros2 topic hz /map
ros2 topic hz /imu
ros2 topic hz /semantic_offline/points
ros2 run tf2_ros tf2_echo map camera_color_optical_frame
```

查看保存的对象点云：

```bash
conda run -n semantic_map python \
  scripts/view_tracked_objects_3d.py \
  --objects-dir /tmp/semantic_05_output/objects \
  --min-observations 5 \
  --show-boxes \
  --show-origin
```

查看器默认显示相机 RGB；需要突出不同对象时，在命令末尾添加
`--color-mode track`。

输出目录包含：

```text
/tmp/semantic_05_output/
|-- semantic_objects.json
|-- fused_objects.npz
`-- objects/
    |-- object_0001_<class>.ply
    `-- object_0001_<class>.npz
```

`semantic_objects.json` 是交给导航模块的对象快照；PLY 用于通用点云查看；NPZ 还保留
点、颜色和对象属性等结构化数组。

## 保存 SLAM 地图并叠加语义对象

完整回放结束后不要立刻关闭 launch。另开终端保存 `/map` 和 Cartographer 状态：

```bash
mkdir -p /tmp/semantic_05_sam_output/slam_map

ros2 run nav2_map_server map_saver_cli \
  -f /tmp/semantic_05_sam_output/slam_map/semantic_05

ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/tmp/semantic_05_sam_output/slam_map/semantic_05.pbstream', include_unfinished_submaps: true}"
```

生成语义 SLAM 地图：

```bash
python3 src/semantic_map_offline/scripts/overlay_objects_on_slam_map.py \
  --map-yaml /tmp/semantic_05_sam_output/slam_map/semantic_05.yaml \
  --objects-dir /tmp/semantic_05_sam_output/objects \
  --output /tmp/semantic_05_sam_output/semantic_slam_map.png \
  --min-observations 5
```

本次完整测试输出位于 `/tmp/semantic_05_world_sam_full_20260714`。SLAM 地图为
322x313、0.05 m/pixel；全部 confirmed 版本包含 60 个对象，稳定版本使用
`min_observations=10`，包含 24 个对象。两份叠加结果中的对象点都位于地图有效尺寸内。

## 已知日志

Cartographer 可能输出 `Dropped N earlier points`。该 bag 每帧激光约 1667 个点，
点时间跨度约 83.7 ms，相邻 scan 间隔偶尔比它短 0.05 至 0.9 ms，因此时间重叠部分的
少量旧点会被丢弃。短测中每帧通常仅丢个位数到十几个点，并已正常创建 submap；这不是
IMU 或 TF 错误，不建议为了隐藏警告而清零 `LaserScan.time_increment`。

最终对象质量仍取决于 SLAM 轨迹。若地图出现明显重影或闭环错位，应先调整
Cartographer，再评价 map 坐标系中的对象融合结果。
