# Cartographer Dataset Exporter 使用手册

cartographer_dataset_exporter 使用 Cartographer 最终优化轨迹，把 ROS 2 bag 导出成
本项目现有 Replica 评估脚本可直接读取的数据集。

## 输入

必须提供完成 finish_trajectory 后保存的 .pbstream、对应的原始 rosbag2，以及 bag
中的 RGB、深度、CameraInfo 和 /tf_static。YOLO results.jsonl 是可选输入；没有它时
每帧仍会生成空检测 JSON。

## 构建

~~~bash
cd ~/vscode_workspace/ros2_wp
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build \
  --packages-select semantic_map_offline \
  --packages-ignore cartographer cartographer_ros cartographer_ros_msgs \
  --symlink-install
source install/setup.bash
~~~

## 先导出 10 帧

~~~bash
export BAG_DIR=/home/weiyu/vscode_workspace/ros2_wp/ros_bag/semantic_05
export OUTPUT_ROOT=$HOME/semantic_map_outputs/semantic_05_final

ros2 launch semantic_map_offline cartographer_dataset_export.launch.py \
  pbstream_path:="$OUTPUT_ROOT/slam_map/semantic_05.pbstream" \
  bag_uri:="$BAG_DIR" \
  output_directory:="$OUTPUT_ROOT/dataset_smoke" \
  max_frames:=10
~~~

使用已有 YOLO JSONL 时增加：

~~~text
jsonl_path:=$HOME/semantic_map_outputs/semantic_05_yolo/results.jsonl
~~~

launch 会加载冻结 pbstream、禁止启动新轨迹、查询最终优化节点、两遍扫描 bag，并在
导出完成后自动退出。

## 检查冒烟测试

~~~bash
find "$OUTPUT_ROOT/dataset_smoke/results" -name 'frame*.jpg' | wc -l
find "$OUTPUT_ROOT/dataset_smoke/results" -name 'depth*.png' | wc -l
find "$OUTPUT_ROOT/dataset_smoke/pose" -name '*.txt' | wc -l
cat "$OUTPUT_ROOT/dataset_smoke/export_report.json"
~~~

三项数量应都是 10。重点检查 planning 中的拒绝数、RGB-D 时间差、轨迹插值间隔、
分辨率不匹配数和 static_tf_frames。

## 全量导出

~~~bash
ros2 launch semantic_map_offline cartographer_dataset_export.launch.py \
  pbstream_path:="$OUTPUT_ROOT/slam_map/semantic_05.pbstream" \
  bag_uri:="$BAG_DIR" \
  output_directory:="$OUTPUT_ROOT/dataset" \
  jsonl_path:="$HOME/semantic_map_outputs/semantic_05_yolo/results.jsonl" \
  max_frames:=0
~~~

没有 JSONL 时删除 jsonl_path。输出目录非空时默认拒绝覆盖；确认重新生成时显式添加
overwrite:=true。

## 输出格式

~~~text
dataset/
├── results/
│   ├── frame000000.jpg
│   ├── depth000000.png
│   └── ...
├── pose/000000.txt
├── detections/000000.json
├── traj.txt
├── cam_params.json
├── timestamps.json
├── metadata.json
└── export_report.json
~~~

- traj.txt 连续保存所有 4 x 4 T_map_camera，兼容现有 Replica 脚本。
- pose 目录保存相同位姿，方便逐帧检查。
- depth PNG 统一为 uint16 毫米，cam_params 的 scale 为 1000.0。
- detections 保存匹配的 JSONL 记录；没有匹配时保存空列表。
- timestamps 保存 RGB-D 时间差和轨迹插值区间。
- export_report 保存完整统计。

## 使用 YOLO-World 进行追踪与融合

导出的目录沿用 Replica 数据格式，因此可以直接交给相同的投影、关联和融合管线。
追踪入口会先检查请求范围内的 `detections/*.json`：文件完整时直接复用导出阶段匹配的
YOLO-World 结果，不再重复推理；没有检测目录的 Replica 数据才回退到现场 YOLO-World
推理，此时必须提供 `--model`、`--clip-model` 和 `--classes-path`。非 SAM 与 MobileSAM
入口的区别仅在于是否把检测框细化为 mask。

非 SAM：

~~~bash
cd ~/vscode_workspace/ros2_wp/src/semantic_map_offline
conda run --no-capture-output -n opi_yolo_eval python \
  scripts/evaluate_projection_roundtrip.py \
  --data-root "$OUTPUT_ROOT/dataset" \
  --output "$OUTPUT_ROOT/tracking_bbox" \
  --start 0 \
  --frames 0 \
  --progress-every 10
~~~

MobileSAM：

~~~bash
conda run --no-capture-output -n opi_yolo_eval python \
  scripts/evaluate_sam_projection_tracking.py \
  --data-root "$OUTPUT_ROOT/dataset" \
  --sam-source MobileSAM \
  --sam-checkpoint /home/weiyu/vscode_workspace/models/mobile_sam.pt \
  --output "$OUTPUT_ROOT/tracking_sam" \
  --start 0 \
  --frames 0 \
  --progress-every 10
~~~

`--frames 0` 表示从 `--start` 开始自动处理全部连续有效帧。显式填写正整数时，不能超过 `export_report.json` 中从起始帧算起的可用帧数。`--progress-every 10` 每 10 帧显示百分比、检测数、投影数、track 数、单帧耗时和 ETA。

## 主要参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| trajectory_id | 0 | pbstream 中要导出的轨迹 |
| tracking_frame | camera_gyro_frame | Pose Graph 节点对应 frame |
| camera_frame | camera_color_optical_frame | 输出相机光学 frame |
| max_rgb_depth_time_diff_s | 0.05 | RGB-D 最大配对时间差 |
| max_detection_time_diff_s | 0.15 | JSONL 与 RGB 最大时间差 |
| max_trajectory_interval_s | 10.0 | 允许插值的最大节点间隔 |
| input_depth_scale | 0.001 | 原深度数值乘该值转换成米 |
| start_frame | 0 | 从第几个深度消息开始 |
| max_frames | 0 | 0 表示处理全部 |
| overwrite | false | 是否覆盖已有输出 |
| allow_resolution_mismatch | false | 只用于诊断 |

## 坐标约定

导出器计算：

~~~text
T_map_camera = T_map_tracking * T_tracking_camera
~~~

输出相机使用 ROS optical/OpenCV 约定：X 向右、Y 向下、Z 向前。不要交换点云 X/Y，
也不要对 traj.txt 再求逆。

## 常见问题

- 轨迹没有节点：检查 trajectory_id，以及 pbstream 是否在完成轨迹后保存。
- 找不到静态 TF 路径：根据错误列出的 frame 修正 tracking_frame/camera_frame。
- RGB 和深度分辨率不同：默认终止；诊断参数不能代替真正的深度注册。
- 留下 .partial：导出中途失败；检查日志后用 overwrite:=true 重新运行。
