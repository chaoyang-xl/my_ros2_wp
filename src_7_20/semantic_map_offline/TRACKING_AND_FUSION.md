# 投影、追踪与融合

每帧检测投影到 `map/world` 坐标系，并直接与已有持久对象匹配、融合和更新属性。

## 独立入口

项目把三类任务拆成独立文件，不通过SAM参数开关混用：

| 任务 | 文件 | SAM |
| --- | --- | --- |
| 单帧正投影、反投影和检测框IoU | `scripts/evaluate_single_frame_roundtrip.py` | 否 |
| 多帧bbox投影、追踪和融合 | `scripts/evaluate_projection_roundtrip.py` | 否 |
| 多帧mask投影、追踪和融合 | `scripts/evaluate_sam_projection_tracking.py` | 是 |
| ROS bbox投影 | `offline_projector_node.py` | 否 |
| ROS MobileSAM投影 | `sam_offline_projector_node.py` | 是 |

公共数据集读取、关联、融合和对象保存位于 `tracking_pipeline.py`。默认项目和ROS launch不加载SAM；SAM入口只作为独立可选实验保留。

## 单帧投影精度

`evaluate_single_frame_roundtrip.py` 是唯一进行反投影和IoU计算的文件。它不追踪、不融合，输出逐检测PLY/NPZ、overlay、`metrics.csv`和summary。

```bash
python scripts/evaluate_single_frame_roundtrip.py \
  --data-root /absolute/path/to/my_data \
  --model /absolute/path/to/yolo11n.pt \
  --output /tmp/single_frame_iou \
  --start 800 --frames 200 --pixel-stride 2
```

## 多帧bbox追踪（默认）

`evaluate_projection_roundtrip.py` 不包含checkpoint、SAM source、SAM device或mask参数：

```bash
conda run -n opi_yolo_eval python scripts/evaluate_projection_roundtrip.py \
  --data-root /absolute/path/to/my_data \
  --model /absolute/path/to/yolo11n.pt \
  --output evaluation/replica_bbox \
  --start 800 --frames 300 --device 0 \
  --pose-convention replica --pixel-stride 2 \
  --voxel-size 0.02 --overlap-radius 0.04 \
  --observation-cluster-eps 0.10 \
  --observation-cluster-min-points 10 \
  --denoise-interval 20 --map-merge-interval 20 \
  --map-merge-overlap 0.80 --min-confirmed-observations 3
```

该流程不计算反投影和IoU，只输出 `objects/`、`associations.json` 和 `summary.json`。

## 多帧SAM追踪（独立可选）

```bash
conda run -n opi_yolo_eval python scripts/evaluate_sam_projection_tracking.py \
  --data-root /absolute/path/to/my_data \
  --model /absolute/path/to/yolo11n.pt \
  --output evaluation/replica_sam \
  --start 800 --frames 300 --device 0 \
  --pose-convention replica --pixel-stride 2 \
  --voxel-size 0.02 --overlap-radius 0.04 \
  --observation-cluster-eps 0.10 \
  --observation-cluster-min-points 10 \
  --denoise-interval 20 --map-merge-interval 20 \
  --map-merge-overlap 0.80 --min-confirmed-observations 3 \
  --sam-checkpoint /absolute/path/to/mobile_sam.pt \
  --sam-source MobileSAM --sam-device cuda --mask-erode-px 2
```

SAM脚本同样不计算反投影和IoU。

## 公共追踪算法

`ObjectTracker` 的处理顺序如下：

1. 每个观测体素降采样，并保留最大空间点簇。
2. 使用质心距离和3D AABB做快速门控。
3. KD-tree计算两个点云的双向最近邻重叠率，取较大的覆盖比例。
4. 根据对象累计的置信度加权类别历史计算语义相似度。
5. 组合几何和语义相似度，使用匈牙利算法完成整帧一对一分配。
6. 未匹配观测创建candidate；达到 `min_confirmed_observations` 后转为confirmed。
7. candidate连续丢失超过 `candidate_max_missed_frames` 后删除。
8. 每隔 `map_merge_interval` 帧在同一张地图内合并高度重叠的重复对象。
9. XYZ与RGB同步体素融合，类别、置信度、时间和观测次数同步更新。

当前没有视觉embedding，因此语义项使用YOLO类别置信度历史；类别偶发跳变时，只要几何
关系足够强仍可关联，随后由累计语义分数更新对象主类别。

标准Replica `traj.txt`直接作为camera-to-world矩阵。`replica_opengl_legacy`只用于复现早期错误结果。

## 对象输出

内部 `track_id` 只用于关联，对外保存逐物体点云：

```text
objects/object_0001_chair.ply
objects/object_0001_chair.npz
```

```bash
conda run -n semantic_map python scripts/view_tracked_objects_3d.py \
  --objects-dir evaluation/replica_bbox/objects \
  --min-observations 5 --display-voxel-size 0.02 \
  --show-boxes --show-origin
```

二维查看器直接读取相同的 NPZ，不重新追踪或融合：

```bash
conda run -n opi_yolo_eval python scripts/view_objects_2d.py \
  --objects-dir evaluation/replica_sam_tracking_all_0000_1999/objects \
  --min-observations 3 --point-radius 1
```

输出 `semantic_objects_xy.png` 和 `semantic_objects_xy.json`。PNG 保持地图米制比例，
默认按对象使用稳定伪彩色，包含 XY 矩形和居中类别标签；JSON 保留二维边界、中心、
尺寸、高度范围、追踪属性和源 NPZ 路径。图中只显示类别名，其他对象信息均保存在
JSON 中；相机颜色可通过 `--color-mode rgb` 启用。

## ROS默认bbox流程

默认项目不采用SAM：

```bash
ros2 launch semantic_map_offline offline_projection.launch.py \
  use_sim_time:=true \
  depth_topic:=/camera/depth/image_raw \
  input_topic:=/yolo/results_json
```

该launch启动纯bbox `offline_projector_node` 和公共 `object_fusion_node`，不会导入torch或MobileSAM。

## ROS独立SAM流程

```bash
ros2 launch semantic_map_offline sam_offline_projection.launch.py \
  use_sim_time:=true \
  color_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  input_topic:=/yolo/results_json \
  sam_checkpoint:=/absolute/path/to/mobile_sam.pt \
  sam_source:=/absolute/path/to/MobileSAM \
  sam_device:=cuda
```

SAM节点要求RGB、深度和检测JSON时间同步。RGB与深度分辨率不同时会用最近邻缩放mask，正式数据仍应优先使用已注册深度。

`semantic_05` 已提供完整的独立入口：

```bash
ros2 launch semantic_map_offline semantic_05_sam_bag_test.launch.py \
  run_yolo:=false \
  jsonl_path:=/tmp/semantic_05_yolo/results.jsonl \
  output_directory:=/tmp/semantic_05_sam_output \
  snapshot_path:=/tmp/semantic_05_sam_output/semantic_objects.json \
  rate:=0.25
```

该入口默认在 CUDA 上运行 MobileSAM，使用逐像素 mask 投影和 1 cm 融合体素。系统
Python 必须能导入 `torch`、`torchvision` 和 `timm`；依赖列表见
`requirements-sam.txt`。

两套ROS前端使用相同输出接口：`/semantic_offline/points`、`/semantic_offline/detections`、`/semantic_offline/fused_points` 和 `/semantic_offline/objects`。bbox 与 SAM 前端都按投影像素从同步彩色帧提取真实 RGB；颜色会与 XYZ 一起经过降采样和融合，并写入对象 NPZ/PLY。默认从检测和投影两层排除 `person`。

融合节点还实时发布 `/semantic_offline/object_markers`，标签中的 `#ID` 和 `n` 可用于
观察同一对象是否被稳定关联。SAM 前端额外发布
`/semantic_offline/sam_debug_image`，用于区分三类问题：mask 错误、单帧投影/TF 错误、
跨帧关联错误。`semantic_05_sam_bag_test.launch.py` 默认加载
`config/semantic_mapping_debug.rviz`，同时显示当前帧点云、融合点云、对象 marker 和二维
占据地图；mask 图像可通过 `rqt_image_view` 查看。

## 实验报告

- 单帧IoU：`evaluation/my_data_single_frame_0800_0999/REPORT.md`
- bbox正确位姿：`evaluation/replica_bbox_pose_fixed_0800_1099/REPORT.md`
- SAM正确位姿：`evaluation/replica_sam_geometric_pose_fixed_0800_1099/REPORT.md`
- bbox/SAM对比：`evaluation/replica_bbox_vs_sam_pose_fixed/REPORT.md`

## 导航JSON

融合完成时自动写入 `OUTPUT/semantic_objects.json`。导航JSON只保存confirmed对象；
candidate仍可在实时Marker中观察，其标签包含状态。JSON与
`semantic_navigation_pkg`兼容，同时保留完整3D边界、观测统计和点云文件路径。

```bash
python scripts/view_tracked_objects_3d.py \
  --objects-dir evaluation/replica_bbox/objects \
  --min-observations 5 \
  --classes chair couch \
  --json-output /absolute/path/to/semantic_objects.json
```
