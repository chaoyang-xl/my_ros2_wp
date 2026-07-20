# Cartographer 最终轨迹离线语义建图指南

本文说明如何把 ROS 2 bag 经过 Cartographer 完整建图和回环优化后，整理成类似
Replica 的 RGB-D、检测结果和逐帧相机位姿数据，再交给 `semantic_map_offline`
进行离线投影与对象融合。示例使用本项目的 `semantic_05`。

本项目中的 traj.txt 不使用原始bag在线记录的历史 /map → camera TF。首先完整回放bag，由Cartographer完成建图、回环检测和Pose Graph全局优化；随后调用 finish_trajectory 结束轨迹，并通过 write_state 保存最终 .pbstream。重新加载该冻结状态后，使用 trajectory_query 获取最终Pose Graph中的优化轨迹节点。

由于Cartographer轨迹节点由运动过滤条件决定，并不与每一帧RGB-D图像一一对应，因此根据每帧深度图时间戳，在相邻优化轨迹节点之间进行SE(3)插值：平移采用线性插值，旋转采用四元数SLERP。最后结合从URDF或 /tf_static 恢复的相机静态外参，生成每帧相机在map坐标系下的位姿：

Tmap←camera​(t)=Tmap←trackingfinal​(t)⋅Ttracking←camera​


最终生成的 traj.txt 是“Cartographer最终优化轨迹节点经过时间插值和相机外参变换后得到的逐帧相机轨迹”，而不是原bag中的在线历史TF。

## 1. 为什么需要最终轨迹

Cartographer 在线运行时会不断发布 TF。检测到回环后，Pose Graph 会重新优化历史轨迹
节点和子地图，但已经写入原 bag 的历史 `/tf` 不会被撤回并重写。

因此不能直接把原 bag 中每个时刻的 `/map -> camera` 当作最终位姿：

```text
原始 rosbag
  -> Cartographer 完整建图
  -> finish_trajectory 最终优化
  -> 保存 pbstream
  -> 为每帧 RGB-D 生成 T_map_camera
  -> Replica 风格离线数据
  -> 投影、追踪和融合
```

最终 `.pbstream` 是 SLAM 的最优估计，并不是真值。

## 2. 当前实现状态

当前仓库可以运行 Cartographer、保存 pbstream/轨迹/二维地图，也可以处理已经整理好的
Replica 数据。当前尚未提供从 `.pbstream + rosbag2` 自动生成第 9 节目录的
`cartographer_dataset_exporter`。第 9 至 11 节是该导出器的接口规范，不是已经存在
的命令。导出器完成前，可以先按第 3 至 8 节保存最终 SLAM 结果。

## 3. 检查输入 bag

至少需要激光、RGB、深度、对应 `CameraInfo`、静态外参，以及 Cartographer 配置要求
的里程计和 IMU。

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 bag info \
  /home/weiyu/vscode_workspace/ros2_wp/ros_bag/semantic_05
```

`semantic_05` 应包含：

```text
/scan
/odom
/camera/accel/sample
/camera/gyro/sample
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_raw
/camera/depth/camera_info
/tf_static
```

该 bag 的深度已注册到 RGB 光学坐标系。更换 bag 后必须重新检查话题、分辨率、深度
注册和外参，不能直接沿用这个结论。

## 4. 安装与构建

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
sudo apt update
sudo apt install \
  ros-$ROS_DISTRO-cartographer \
  ros-$ROS_DISTRO-cartographer-ros \
  ros-$ROS_DISTRO-nav2-map-server \
  ros-$ROS_DISTRO-rviz2

cd ~/vscode_workspace/ros2_wp
rosdep install --from-paths src --ignore-src -r -y --skip-keys cartographer
colcon build \
  --packages-select semantic_map_pkg semantic_map_offline \
  --packages-ignore cartographer cartographer_ros cartographer_ros_msgs \
  --symlink-install
source install/setup.bash
```

工作空间里有 Cartographer 源码、但使用 `/opt/ros` 二进制包时，应保留
`--packages-ignore`。每个新终端都要执行：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/vscode_workspace/ros2_wp/install/setup.bash
```

`Package ... not found` 通常是第二条 `source` 没有执行。

## 5. 创建永久输出目录

正式结果不要写入 `/tmp`：

```bash
export BAG_DIR=/home/weiyu/vscode_workspace/ros2_wp/ros_bag/semantic_05
export OUTPUT_ROOT=$HOME/semantic_map_outputs/semantic_05_final
mkdir -p "$OUTPUT_ROOT/slam_map"
mkdir -p "$OUTPUT_ROOT/trajectory"
mkdir -p "$OUTPUT_ROOT/dataset"
```

新终端需要重新设置这两个环境变量。

## 6. 第一遍：完成最终 SLAM

终端 1：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/vscode_workspace/ros2_wp/install/setup.bash
export BAG_DIR=/home/weiyu/vscode_workspace/ros2_wp/ros_bag/semantic_05

ros2 launch semantic_map_pkg offline_cartographer_bag.launch.py \
  bag_uri:="$BAG_DIR" \
  configuration_basename:=offline_ros_bag_with_odom_imu.lua \
  play_color:=false \
  play_color_info:=false \
  play_depth:=false \
  play_imu:=true \
  enable_imu_sync:=true \
  publish_robot_state:=false \
  rate:=1.0 \
  enable_rviz:=true
```

该 launch 不播放原 bag 的动态 `/tf`，map TF 由本次 Cartographer 重新计算。
`publish_robot_state:=false` 表示使用 bag 的 `/tf_static`。如果新 bag 没有完整静态
TF，需要提供校准后的 URDF，并使用：

```text
publish_robot_state:=true
urdf_path:=/absolute/path/to/robot.urdf
```

RViz 中检查激光是否贴墙、墙面是否重影、运动方向是否正确、回到起点后是否闭合。等待
bag 完整播放结束，但不要关闭 launch，下一步还需要 Cartographer 服务。

## 7. 完成轨迹并保存

终端 2 查询轨迹编号：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/vscode_workspace/ros2_wp/install/setup.bash
ros2 service list | grep -E \
  'finish_trajectory|write_state|trajectory_query|get_trajectory_states'
ros2 service call /get_trajectory_states \
  cartographer_ros_msgs/srv/GetTrajectoryStates "{}"
```

第一条轨迹通常为 0，但应以返回值为准。完成轨迹：

```bash
ros2 service call /finish_trajectory \
  cartographer_ros_msgs/srv/FinishTrajectory \
  "{trajectory_id: 0}"
```

必须看到成功状态。然后保存优化后的 Pose Graph 节点：

```bash
export OUTPUT_ROOT=$HOME/semantic_map_outputs/semantic_05_final
ros2 service call /trajectory_query \
  cartographer_ros_msgs/srv/TrajectoryQuery \
  "{trajectory_id: 0}" \
  | tee "$OUTPUT_ROOT/trajectory/trajectory_query.txt"
```

这些不是每一帧相机位姿，后续仍需插值。保存 pbstream：

```bash
ros2 service call /write_state \
  cartographer_ros_msgs/srv/WriteState \
  "{filename: '$OUTPUT_ROOT/slam_map/semantic_05.pbstream', include_unfinished_submaps: false}"
ls -lh "$OUTPUT_ROOT/slam_map/semantic_05.pbstream"
```

文件存在后才可以在终端 1 按 `Ctrl+C`。

## 8. 从 pbstream 生成二维地图

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
export OUTPUT_ROOT=$HOME/semantic_map_outputs/semantic_05_final
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename="$OUTPUT_ROOT/slam_map/semantic_05.pbstream" \
  -map_filestem="$OUTPUT_ROOT/slam_map/semantic_05" \
  -resolution=0.05
ls -lh "$OUTPUT_ROOT/slam_map"
```

应得到 `semantic_05.pbstream`、`semantic_05.pgm` 和 `semantic_05.yaml`。

## 9. Replica 风格输出规范

```text
dataset/
├── color/000000.jpg
├── depth/000000.png
├── detections/000000.json
├── pose/000000.txt
├── intrinsics.json
├── timestamps.json
├── metadata.json
└── export_report.json
```

- RGB 保持原始宽高比。
- 深度使用无损 PNG，优先保留原始 `uint16` 毫米值。
- 检测时间戳来自对应 RGB 的 `header.stamp`。
- 没有检测的帧也保存空 JSON，保持编号对应。

每个 pose 文件保存 4 x 4 的 `T_map_camera`：

```text
p_map = T_map_camera * p_camera
```

单位为米。不要保存逆矩阵，也不要手动交换 X/Y 轴。

## 10. 为每帧生成最终位姿

导出器必须使用最终 pbstream 轨迹节点或完成后的 `trajectory_query`，不能复制原 bag
的历史动态 TF。对深度帧时间 `t`：

1. 找到 `t0 <= t <= t1` 的相邻最终轨迹节点。
2. 平移线性插值，旋转使用四元数 SLERP。
3. 不在轨迹首尾之外外推。
4. 插值间隔过大时丢弃并记录原因。

当前配置是：

```text
map_frame       = map
tracking_frame  = camera_gyro_frame
published_frame = base_link
```

最终彩色相机位姿为：

```text
T_map_color(t) = T_map_tracking(t) * T_tracking_color
```

`T_tracking_color` 必须来自同一份 `/tf_static` 或 URDF。建议以深度时间为投影时间，
选择最近 RGB，按 RGB 时间匹配检测，并把位姿插值到深度时间。`export_report.json`
必须记录每帧时间差，超过阈值时丢弃。

## 11. 导出器验收

报告至少记录 RGB/深度总数、成功配对数、缺少图像/内参/轨迹的帧数、同步时间差、
插值间隔和最终导出帧数。

```bash
find "$OUTPUT_ROOT/dataset/color" -type f | wc -l
find "$OUTPUT_ROOT/dataset/depth" -type f | wc -l
find "$OUTPUT_ROOT/dataset/pose" -type f | wc -l
find "$OUTPUT_ROOT/dataset/detections" -type f | wc -l
```

四个目录的编号必须一一对应。

## 12. 验证投影

单帧正投影再反投影只能验证内参、深度尺度、RGB-D 注册和投影数学，不能证明 SLAM
位姿正确，因为同一个错误 TF 可能在正逆变换中抵消。

跨帧验证应把第 i 帧静态点投到 map，再投到第 j 帧，比较深度残差、mask/检测框重叠、
静态对象中心漂移以及墙面和地面的重合。需要处理视野边界、遮挡和 z-buffer，并排除
`person` 等动态类别。

将语义点云压缩到 XY 并叠加最终 YAML 时，墙体应与占用栅格基本对齐，物体不应整体
嵌入墙中，结果不应旋转 90 度或镜像。

## 13. 常见问题

- 找不到包：重新 source ROS 和工作空间。
- 找不到服务：确认 Cartographer launch 仍在运行。
- pbstream 未生成：使用绝对路径，提前建目录并检查服务返回状态。
- 整体旋转：检查 `T_map_camera` 方向、光学坐标系旋转和静态外参。
- Replica 好而 bag 差：先检查 SLAM、外参、注册、同步和轨迹插值，再调关联参数。

不要通过交换点云轴来掩盖 TF 外参错误。

## 14. 最终检查清单

- [ ] 原 bag 未修改并已备份。
- [ ] Cartographer 完整播放全部数据。
- [ ] `finish_trajectory` 返回成功。
- [ ] pbstream 来自完成后的最终轨迹。
- [ ] PGM/YAML 由最终 pbstream 生成。
- [ ] 内参和外参来自本次 bag/机器人。
- [ ] 每帧保存 `T_map_camera`。
- [ ] 位姿来自最终轨迹插值，不是历史 `/tf`。
- [ ] RGB、深度、检测和 pose 编号对应。
- [ ] 单帧和跨帧验证通过。
- [ ] 语义 XY 结果与最终 SLAM 地图基本对齐。

全部通过后再运行全量追踪和融合，才能区分问题来自 SLAM/标定还是语义关联。

## 方案QA
Cartographer 回环修正了什么

  Cartographer 检测到回环后，会对 Pose Graph 中相互连接的：

  - 轨迹节点
  - 子地图位姿
  - 回环约束
  - 里程计、IMU 等约束

  进行全局优化。因此，不只是当前机器人位姿发生变化，历史轨迹节点的 /map 位姿也会被重新优化。Cartographer 配置文档
  (https://google-cartographer.readthedocs.io/en/latest/configuration.html?highlight=pose_graph)

  但要注意两个区别：

  1. Pose Graph 节点不一定对应每一帧相机图像。
  2. ROS 已经发布过的历史 TF 不会被撤回并重新发布。

  也就是说，bag 里在线记录的历史 /tf 仍然是“当时计算出来的 TF”。回环以后，Cartographer 内部保存的是优化后的完整轨迹，但不会把过去每个时间戳的 TF 重新写进原 bag。

  所以，不能简单读取原始 bag 里的 /map -> camera TF 当作最终轨迹。

  推荐流程

  建议采用两阶段处理。

  ### 第一阶段：完成 Cartographer 离线建图

  播放完整 bag，让 Cartographer 处理全部激光、里程计和 IMU，然后：

  1. 调用 finish_trajectory
  2. 执行最终全局优化
  3. 调用 write_state
  4. 保存 .pbstream
  5. 保存最终二维 SLAM 地图

  Cartographer 官方的 Assets Writer 本身就是这样工作的：使用原始 bag、最终 .pbstream 和传感器外参，重新处理全部传感器数据。Cartographer Assets Writer
  (https://google-cartographer-ros.readthedocs.io/en/stable/assets_writer.html)

  .pbstream 里保存的才是最终优化后的 Pose Graph 状态。

  ### 第二阶段：导出 Replica 风格数据

  重新读取原始 bag，按 RGB 或深度图时间戳输出：

  semantic_dataset/
  ├── color/
  │   ├── 000000.jpg
  │   └── 000001.jpg
  ├── depth/
  │   ├── 000000.png
  │   └── 000001.png
  ├── detections/
  │   ├── 000000.json
  │   └── 000001.json
  ├── pose/
  │   ├── 000000.txt
  │   └── 000001.txt
  ├── intrinsics.json
  ├── timestamps.json
  ├── metadata.json
  ├── map.yaml
  └── map.pgm

  其中每一个 pose/xxxxxx.txt 保存：

  T_map_camera(timestamp)

  计算过程是：

  T_map_camera(t)
      = T_map_base(t)
      × T_base_camera

  T_map_base(t) 来自最终优化后的 .pbstream，T_base_camera 来自 URDF 或 /tf_static。

  不是直接保存“所有 TF”

  更准确的说法应该是：

  > 从最终优化后的 Cartographer 轨迹，为每一帧 RGB-D 图像生成对应的相机位姿。

  Cartographer 只在满足运动过滤条件时创建轨迹节点，所以图像时间戳通常不会与轨迹节点完全相同。需要在相邻优化轨迹节点之间插值：

  - 平移：线性插值
  - 旋转：四元数 SLERP
  - 超出轨迹时间范围的图像：丢弃，不做外推

  这样每一帧都会得到最终优化轨迹下的 T_map_camera。

  这个方案的优势

  1. 所有帧统一使用最终回环优化后的轨迹。
  2. 不需要在线处理 map <- odom 跳变。
  3. 不需要对象在回环后重新 re-snap 或几何修正。
  4. 同一份数据重复处理可以得到一致结果。
  5. 可以直接复用目前 Replica 的投影、SAM 和对象融合流程。
  6. 方便区分 SLAM、投影、检测和对象关联各自的问题。

  这也意味着离线流程中，之前的 pending_resnap 等回环保护机制可以不参与，因为对象是在最终轨迹确定后一次性构建的。

  仍然需要验证

  最终 .pbstream 是 Cartographer 的最优估计，不是真值。导出后还要检查：

  - 静态场景跨帧重投影误差
  - 地面高度和法向稳定性
  - 同一个静态对象的点云重合程度
  - 语义点云和二维 SLAM 地图的对齐
  - 相邻帧位姿是否有异常跳变
  - 回到起点后点云是否闭合

  尤其建议先关闭追踪和融合，将相邻帧静态点云变换到 /map 后计算几何重叠。这样能先证明最终轨迹和相机外参是否可靠。


