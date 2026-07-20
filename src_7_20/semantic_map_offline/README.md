# semantic_map_offline

面向 RGB-D 数据集和 ROS 2 rosbag 的离线语义对象建图包。项目将二维目标检测结果投影为
`map/world` 坐标系下的彩色三维点云，并提供跨帧对象关联、点云融合、对象 JSON 导出以及
二维/三维可视化。

项目默认使用检测框内深度点，不依赖 SAM；MobileSAM 是独立的可选前端，不通过参数开关
与默认流程混用。

## 功能

- 投影检测框内全部有效深度点，而不是只投影中心点。
- 支持 ROS 2 检测 JSON topic、JSONL 文件和 Replica 格式 RGB-D 数据。
- 支持检测框投影和 MobileSAM mask 投影两套独立入口。
- 在 `map/world` 中完成对象关联、状态管理和 XYZRGB 点云融合。
- 每个 confirmed 对象保存独立的 PLY、NPZ 和导航用 JSON 记录。
- 提供 RViz 实时调试、Open3D 三维查看和 XY 语义俯视图。
- 单帧正投影/反投影闭环测试可计算检测框 IoU；追踪流程不计算 IoU。

## 数据链路

```text
RGB image -> detector JSON (class, confidence, xyxy, stamp)
                    |
depth + intrinsics -+-> bbox points or MobileSAM mask points
                         -> camera-frame XYZRGB
TF / camera pose -------> map/world XYZRGB
                         -> observation cleanup
                         -> object association and fusion
                         -> PLY + NPZ + semantic_objects.json
                         -> RViz / 3D viewer / 2D top-down image
```

ROS 主话题：

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/yolo/results_json` | `std_msgs/msg/String` | 检测类别、置信度、框和图像时间戳 |
| `/semantic_offline/points` | `sensor_msgs/msg/PointCloud2` | 当前帧对象点云 |
| `/semantic_offline/detections` | `std_msgs/msg/String` | 当前帧检测与点云元数据 |
| `/semantic_offline/fused_points` | `sensor_msgs/msg/PointCloud2` | 跨帧融合对象点云 |
| `/semantic_offline/objects` | `std_msgs/msg/String` | 持久对象属性 |
| `/semantic_offline/object_markers` | `visualization_msgs/msg/MarkerArray` | 对象边界和状态 |
| `/semantic_offline/sam_debug_image` | `sensor_msgs/msg/Image` | SAM mask 调试图，仅 SAM 前端 |

## 仓库结构

```text
semantic_map_offline/
├── semantic_map_offline/       # 投影、追踪、融合、保存和 ROS 节点
├── launch/                     # 通用入口和已验证 rosbag 入口
├── scripts/                    # Replica 评估与 2D/3D 查看工具
├── config/                     # RViz 配置和 YOLO-World 类别词表
├── test/                       # 核心算法与 ROS 输出测试
├── evaluation/                 # 小型实验报告和汇总，不提交批量点云
├── MobileSAM/                  # 官方 MobileSAM Git submodule
├── TRACKING_AND_FUSION.md      # 算法、参数和实验说明
└── *_BAG_TEST.md               # 特定 bag 的复现实验记录
```

## 安装

### 1. 获取源码

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules <repository-url> semantic_map_offline
cd semantic_map_offline
```

已经 clone 但缺少 `MobileSAM/` 内容时：

```bash
git submodule update --init --recursive
```

### 2. 安装 ROS 依赖并构建

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src/semantic_map_offline --ignore-src -r -y
colcon build --packages-select semantic_map_offline --symlink-install
source install/setup.bash
```

每个新终端都需要重新 source ROS 和工作空间：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 3. 可选 Python 依赖

```bash
# Replica 评估和查看工具
python3 -m pip install -r requirements-eval.txt

# MobileSAM
python3 -m pip install -r requirements-sam.txt

# 包内 YOLO-World
python3 -m pip install -r requirements-yolo-world.txt
```

PyTorch/CUDA 应根据目标主机单独安装。模型权重不进入仓库，可统一放在工作空间外的
`models/` 目录。

## 快速开始：ROS 检测框投影

该入口启动 `offline_projector_node` 和 `object_fusion_node`，不加载 SAM：

```bash
ros2 launch semantic_map_offline offline_projection.launch.py \
  use_sim_time:=true \
  input_topic:=/yolo/results_json \
  color_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_frame:=camera_depth_optical_frame \
  target_frame:=map \
  camera_fx:=<fx> camera_fy:=<fy> \
  camera_cx:=<cx> camera_cy:=<cy> \
  depth_scale:=0.001 \
  output_directory:=/tmp/semantic_output \
  snapshot_path:=/tmp/semantic_output/semantic_objects.json
```

随后播放包含 `/clock`、RGB、depth 和 TF 的 bag：

```bash
ros2 bag play /absolute/path/to/bag --clock
```

如果 bag 没有检测 topic，但已经有 YOLO `results.jsonl`，将
`input_topic` 替换为：

```bash
jsonl_path:=/absolute/path/to/results.jsonl
```

相机内参必须与深度图对应。检测框和深度图应来自同一成像模型，或深度已经注册到
彩色相机；仅按分辨率缩放不能补偿 RGB-depth 外参和视场差异。

## 快速开始：ROS + MobileSAM

SAM 入口需要同步 RGB、深度和检测结果：

```bash
ros2 launch semantic_map_offline sam_offline_projection.launch.py \
  use_sim_time:=true \
  input_topic:=/yolo/results_json \
  color_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_frame:=camera_depth_optical_frame \
  target_frame:=map \
  camera_fx:=<fx> camera_fy:=<fy> \
  camera_cx:=<cx> camera_cy:=<cy> \
  sam_checkpoint:=/absolute/path/to/mobile_sam.pt \
  sam_source:=/absolute/path/to/semantic_map_offline/MobileSAM \
  sam_device:=cuda \
  output_directory:=/tmp/semantic_sam_output \
  snapshot_path:=/tmp/semantic_sam_output/semantic_objects.json
```

查看二维 mask：

```bash
ros2 run rqt_image_view rqt_image_view /semantic_offline/sam_debug_image
```

## 包内 YOLO-World

`yolo_world_recorder_node` 的 JSON topic 和 JSONL 格式与投影节点直接兼容。类别 ID 按
类别词表中的顺序生成，不能假设 `class_id=0` 一定是 `person`。

```bash
ros2 run semantic_map_offline yolo_world_recorder_node --ros-args \
  -p image_topic:=/camera/color/image_raw \
  -p output_dir:=/tmp/yolo_world_results \
  -p detect_model:=/absolute/path/to/yolov8s-world.pt \
  -p clip_model:=/absolute/path/to/ViT-B-32.pt \
  -p classes_path:=$(ros2 pkg prefix semantic_map_offline)/share/semantic_map_offline/config/class_list/gpt_indoor_general.txt \
  -p excluded_labels:=person,wall,floor,ceiling,unknown
```

节点发布 `/yolo/results_json` 和 `/yolo/debug_image`，并保存 `latest.json`、
`latest.jpg` 和 `results.jsonl`。检测时间戳来自输入 RGB 图像的 `header.stamp`。

检测图保存可以独立控制：`save_annotated_images` 决定是否写图，`save_every_n` 决定
每隔多少次推理写入 `frames/`（设为 `1` 保存每帧，设为 `0` 只更新 `latest.jpg`），
`image_jpeg_quality` 控制 JPEG 质量。归档文件使用图像消息时间戳命名，便于与 bag 对齐。
`semantic_05` 专用 launch 对应参数为 `save_yolo_images`、`yolo_save_every_n`、
`yolo_image_jpeg_quality` 和 `publish_yolo_debug_image`。

## Replica 数据集

数据目录应至少包含：

```text
DATA_ROOT/
├── cam_params.json
├── traj.txt
└── results/
    ├── frame000000.jpg
    ├── depth000000.png
    └── ...
```

### 单帧投影精度验证

这是唯一计算反投影 IoU 的入口，不进行追踪或融合：

```bash
python3 scripts/evaluate_single_frame_roundtrip.py \
  --data-root /absolute/path/to/data \
  --model /absolute/path/to/yolo.pt \
  --output /tmp/projection_roundtrip \
  --start 800 --frames 20 --pixel-stride 2
```

### 多帧检测框追踪

项目默认流程，不使用 SAM，也不计算 IoU：

当 `DATA_ROOT/detections` 覆盖请求范围时会自动复用导出检测；没有该目录时才使用下面的 YOLO-World 参数现场推理。两个入口均可用 `--progress-every 10` 显示进度和 ETA。

```bash
python3 scripts/evaluate_projection_roundtrip.py \
  --data-root /absolute/path/to/data \
  --model /absolute/path/to/yolov8s-world.pt \
  --clip-model /absolute/path/to/clip/ViT-B-32.pt \
  --classes-path config/class_list/gpt_indoor_general.txt \
  --output /tmp/replica_bbox \
  --start 0 --frames 2000 --device 0 \
  --pose-convention replica --pixel-stride 2 \
  --voxel-size 0.02 --overlap-radius 0.04 \
  --min-confirmed-observations 3
```

### 多帧 MobileSAM 追踪

```bash
python3 scripts/evaluate_sam_projection_tracking.py \
  --data-root /absolute/path/to/data \
  --model /absolute/path/to/yolov8s-world.pt \
  --clip-model /absolute/path/to/clip/ViT-B-32.pt \
  --classes-path config/class_list/gpt_indoor_general.txt \
  --output /tmp/replica_sam \
  --start 0 --frames 2000 --device 0 \
  --pose-convention replica --pixel-stride 2 \
  --voxel-size 0.02 --overlap-radius 0.04 \
  --min-confirmed-observations 3 \
  --sam-checkpoint /absolute/path/to/mobile_sam.pt \
  --sam-source MobileSAM --sam-device cuda --mask-erode-px 2
```

标准 Replica `traj.txt` 使用 `--pose-convention replica`。完整算法和调参说明见
[TRACKING_AND_FUSION.md](TRACKING_AND_FUSION.md)。

## 输出

```text
OUTPUT/
├── objects/
│   ├── object_0001_chair.ply
│   ├── object_0001_chair.npz
│   └── ...
├── semantic_objects.json
├── associations.json
└── summary.json
```

| 输出 | 主要内容 |
| --- | --- |
| PLY | 对象在 `map/world` 下的 XYZRGB 点云，适合 CloudCompare/Open3D |
| NPZ | `points_map`、`rgb`、对象 ID、类别、置信度、状态和观测统计 |
| `semantic_objects.json` | confirmed 对象的类别、位姿、3D 边界、统计和点云路径 |
| `associations.json` | 每帧观测与对象 ID 的关联诊断 |
| `summary.json` | 本次处理帧数、检测数和对象数量汇总 |

`track_id` 是对象关联 ID，不是机器人轨迹。candidate 可通过实时 Marker 调试，但导航
JSON 只保存 confirmed 对象。

## 查看结果

三维彩色点云：

```bash
python3 scripts/view_tracked_objects_3d.py \
  --objects-dir /tmp/replica_sam/objects \
  --min-observations 5 --show-boxes --show-origin
```

二维 XY 语义俯视图：

```bash
python3 scripts/view_objects_2d.py \
  --objects-dir /tmp/replica_sam/objects \
  --output /tmp/replica_sam/semantic_objects_xy.png \
  --json-output /tmp/replica_sam/semantic_objects_xy.json \
  --min-observations 5 --color-mode object --point-radius 1
```

PNG 使用对象伪彩色、米制等比例坐标、矩形边界和类别标签。配套 JSON 保存对象的完整
二维边界、中心、尺寸、高度和观测属性。

ROS 调试推荐加载 [config/semantic_mapping_debug.rviz](config/semantic_mapping_debug.rviz)，
同时观察当前帧点云、融合点云、对象 Marker、TF 和 `/map`。

## 常用参数
[具体参数说明](docs/parameter_explanation.md) 

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `pixel_stride` | `2` | 投影像素步长；越小点越密、计算越慢 |
| `voxel_size` | `0.02 m` | 融合点云体素尺寸 |
| `overlap_radius` | `0.04 m` | 点云最近邻重叠半径 |
| `max_centroid_distance_m` | `1.0 m` | 对象关联质心距离门限 |
| `min_geometric_overlap` | `0.05` | 最低几何重叠率 |
| `association_threshold` | `0.45` | 几何与语义组合分数门限 |
| `min_confirmed_observations` | `3` | candidate 转 confirmed 的观测次数 |
| `candidate_max_missed_frames` | `30` | candidate 最大连续丢失帧数 |
| `excluded_labels` | `person` | 不投影的类别名，逗号分隔 |

大范围降低关联门限可能错误合并相邻物体。真实 rosbag 中应先检查时间同步、深度注册和
TF/SLAM 质量，再调整对象关联参数。

## 已验证数据与报告
- [Cartographer Dataset Exporter 使用手册](docs/CARTOGRAPHER_DATASET_EXPORTER.md)：最终 pbstream 加原 bag 的一键导出命令。
- [Cartographer 最终轨迹离线语义建图指南](docs/CARTOGRAPHER_FINAL_TRAJECTORY_WORKFLOW.md)：
  从 rosbag、最终回环优化、pbstream 和逐帧位姿到 Replica 风格离线数据的零基础流程。

- [SEMANTIC_05_BAG_TEST.md](docs/SEMANTIC_05_BAG_TEST.md)：`semantic_05` 完整 ROS 2 bag 流程。
- [SEMANTIC_01_BAG_TEST.md](docs/SEMANTIC_01_BAG_TEST.md)：早期 `semantic_01` 流程。
- [SEMANTIC_BAGS_01_TEST.md](docs/SEMANTIC_BAGS_01_TEST.md)：带较完整 TF 的 bag 流程。
- [MEETING_ROOM_BAG_TEST.md](docs/MEETING_ROOM_BAG_TEST.md)：Replica Meeting Room ROS 1 bag 转换与测试。
- [evaluation/replica_bbox_tracking_all_0000_1999/REPORT.md](evaluation/replica_bbox_tracking_all_0000_1999/REPORT.md)：Replica 2000 帧检测框实验。
- [evaluation/replica_sam_tracking_all_0000_1999/REPORT.md](evaluation/replica_sam_tracking_all_0000_1999/REPORT.md)：Replica 2000 帧 SAM 实验。

## 本地测试启动流程
[本地测试启动流程，当前在本地进行测试脚本说明](docs/local_test.md)



## 限制

- 对象几何依赖 TF/SLAM 或数据集位姿；位姿漂移会造成点云重影和对象碎片化。
- 当前语义关联使用类别置信度历史，不包含视觉 embedding。
- 同一大型物体被检测成多个局部框时，可能生成多个对象，需要结合检测质量和关联参数处理。
- `semantic_05_*` 等专用 launch 依赖外部 `semantic_map_pkg` 的机器人模型和 SLAM 配置；
  两个通用 launch 不依赖该包。

## 测试

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon test --packages-select semantic_map_offline
colcon test-result --verbose
```

项目采用 Apache-2.0 许可证。MobileSAM 使用其子模块内的独立许可证。
