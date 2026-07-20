# semantic_map_offline 启动流程说明

本文整理项目的四种主要运行方式：

1.  Replica 数据集——非 SAM；
2.  Replica 数据集——MobileSAM；
3.  SEMANTIC_05_BAG——非 SAM；
4.  SEMANTIC_05_BAG——MobileSAM。

# 1\. 四种流程总览

| 数据来源 | 非 SAM | MobileSAM |
| --- | --- | --- |
| Replica 数据集 | evaluate_projection_roundtrip.py | evaluate_sam_projection_tracking.py |
| semantic_05 rosbag | semantic_05_bag_test.launch.py | semantic_05_sam_bag_test.launch.py |

其中：

- Replica 流程直接读取数据集中的 RGB、深度图和 traj.txt，不依赖 ROS 2 topic 和 TF；
- semantic_05 流程通过 ROS 2 播放 rosbag，由 Cartographer 生成 map 坐标系 TF，再运行投影和对象融合；
- 非 SAM 流程使用 YOLO 检测框内部的全部有效深度点；
- SAM 流程先使用 YOLO 检测框提示 MobileSAM，再只投影 SAM mask 内部的有效深度点。

# 2\. 运行前准备

## 2.1 进入项目目录

cd ~/vscode_workspace/ros2_wp/src/semantic_map_offline

## 2.2 Replica Python 环境

Replica 评估使用此前建立的 Conda 环境：

conda activate opi_yolo_eval

安装评估依赖：

python -m pip install -r requirements-eval.txt

SAM 流程还需要：

python -m pip install -r requirements-sam.txt

确认 MobileSAM 子模块存在：

git submodule update --init --recursive

## 2.3 ROS 2 工作空间构建

semantic_05 流程需要先构建：

cd ~/vscode_workspace/ros2_wp  
<br/>source /opt/ros/jazzy/setup.bash  
<br/>colcon build \\  
\--packages-ignore cartographer cartographer_ros cartographer_ros_msgs \\  
\--packages-select semantic_map_pkg semantic_map_offline \\  
\--symlink-install  
<br/>source install/setup.bash

项目文档建议在工作空间中存在 Cartographer 源码、但实际使用系统版本时，通过 --packages-ignore 避免重复构建和包冲突。

每次打开新终端，都需要执行：

source /opt/ros/jazzy/setup.bash  
source ~/vscode_workspace/ros2_wp/install/setup.bash

# 3\. Replica 数据集目录要求

Replica 数据目录至少需要：

/home/weiyu/vscode_workspace/my_data/  
├── cam_params.json  
├── traj.txt  
└── results/  
├── frame000000.jpg  
├── depth000000.png  
├── frame000001.jpg  
├── depth000001.png  
└── ...

其中：

| 文件  | 作用  |
| --- | --- |
| cam_params.json | 相机内参和深度比例 |
| traj.txt | 每一帧相机到世界坐标系的位姿 |
| frameXXXXXX.jpg | RGB 图像 |
| depthXXXXXX.png | 深度图 |

程序会按照 --start 和 --frames 构造图像路径，并检查对应帧的 RGB、深度和位姿是否完整。

# 4\. Replica 非 SAM 流程

## 4.1 数据链路

Replica RGB图像  
↓  
YOLO检测  
↓  
获得类别、置信度和2D检测框  
↓  
读取对应深度图  
↓  
投影检测框内部的有效深度点  
↓  
camera坐标系点云  
↓ traj.txt  
world/map坐标系点云  
↓  
ObjectTracker跨帧关联与融合  
↓  
PLY + NPZ + semantic_objects.json

非 SAM 入口脚本是：

scripts/evaluate_projection_roundtrip.py

该脚本本身只负责创建公共参数解析器并调用：

tracking_pipeline.run_tracking()

因此真正的逐帧 YOLO、投影、位姿变换、关联和保存逻辑位于：

semantic_map_offline/tracking_pipeline.py

## 4.2 推荐启动命令

cd ~/vscode_workspace/ros2_wp/src/semantic_map_offline  
conda activate opi_yolo_eval  
<br/>python scripts/evaluate_projection_roundtrip.py \\  
\--data-root /home/weiyu/vscode_workspace/my_data \\  
\--model /home/weiyu/vscode_workspace/models/yolo11n.pt \\  
\--output evaluation/replica_bbox_0800_1099 \\  
\--start 800 \\  
\--frames 300 \\  
\--device 0 \\  
\--confidence 0.35 \\  
\--pose-convention replica \\  
\--pixel-stride 2 \\  
\--voxel-size 0.02 \\  
\--overlap-radius 0.04 \\  
\--max-centroid-distance-m 1.0 \\  
\--min-geometric-overlap 0.05 \\  
\--min-bbox-overlap 0.0 \\  
\--association-threshold 0.45 \\  
\--geometry-weight 0.7 \\  
\--semantic-weight 0.3 \\  
\--observation-cluster-eps 0.10 \\  
\--observation-cluster-min-points 10 \\  
\--max-extent-growth 2.0 \\  
\--denoise-interval 20 \\  
\--map-merge-interval 20 \\  
\--map-merge-overlap 0.80 \\  
\--min-confirmed-observations 3 \\  
\--candidate-max-missed-frames 30

当前公共参数定义在 tracking_pipeline.build_common_parser() 中。

## 4.3 主要参数说明

| 参数  | 含义  |
| --- | --- |
| \--data-root | Replica 数据根目录 |
| \--model | YOLO模型路径 |
| \--output | 实验结果输出目录 |
| \--start | 起始帧编号 |
| \--frames | 连续处理帧数 |
| \--device | YOLO推理设备，0表示第0块GPU |
| \--confidence | YOLO最低检测置信度 |
| \--pose-convention | Replica位姿坐标约定 |
| \--pixel-stride | 检测框内部深度像素采样步长 |
| \--voxel-size | 跨帧融合后的体素尺寸 |
| \--overlap-radius | 几何重叠计算的最近邻半径 |
| \--min-geometric-overlap | 几何关联最低重叠率 |
| \--association-threshold | 综合关联分数阈值 |
| \--min-confirmed-observations | 对象从candidate变为confirmed所需观测次数 |

标准 Replica 的 traj.txt 应使用：

\--pose-convention replica

当前代码中 replica 和 opencv 都直接使用读取到的 traj.txt；只有兼容旧实验的 replica_opengl_legacy 会额外执行坐标轴翻转。

## 4.4 当前参数名称注意事项

当前 main 分支中，不再使用以下旧参数：

\--geometric-fusion  
\--dbscan-eps  
\--dbscan-min-points  
\--min-aabb-iou  
\--min-observations-to-keep

当前对应参数为：

| 旧参数或旧叫法 | 当前参数 |
| --- | --- |
| \--dbscan-eps | \--observation-cluster-eps |
| \--dbscan-min-points | \--observation-cluster-min-points |
| \--min-aabb-iou | \--min-bbox-overlap |
| \--min-observations-to-keep | \--min-confirmed-observations |
| \--geometric-fusion | 不需要，当前流程默认执行对象追踪与几何融合 |

## 4.5 输出结果

evaluation/replica_bbox_0800_1099/  
├── objects/  
│ ├── object_0001_chair.ply  
│ ├── object_0001_chair.npz  
│ ├── object_0002_couch.ply  
│ └── ...  
├── semantic_objects.json  
├── associations.json  
└── summary.json

其中：

| 文件  | 内容  |
| --- | --- |
| objects/\*.ply | 融合后的彩色对象点云 |
| objects/\*.npz | 点云、类别、置信度、观测次数等完整数据 |
| semantic_objects.json | confirmed对象的语义地图记录 |
| associations.json | 每帧检测和track关联结果 |
| summary.json | 本次实验帧数、检测数和对象数汇总 |

对象文件是在全部帧处理完成、执行 fusion.finalize() 后统一写入。

# 5\. Replica MobileSAM 流程

## 5.1 数据链路

Replica RGB图像  
↓  
YOLO检测  
↓  
2D检测框作为MobileSAM提示框  
↓  
MobileSAM生成物体mask  
↓  
可选mask腐蚀  
↓  
读取mask内部的有效深度点  
↓  
camera坐标系点云  
↓ traj.txt  
world/map坐标系点云  
↓  
ObjectTracker跨帧关联与融合  
↓  
PLY + NPZ + semantic_objects.json

SAM入口脚本是：

scripts/evaluate_sam_projection_tracking.py

它在公共参数基础上增加：

\--sam-checkpoint  
\--sam-source  
\--sam-device  
\--mask-erode-px

随后仍然调用同一个：

tracking_pipeline.run_tracking()

因此 SAM 和非 SAM 共用对象关联、融合及输出代码，主要区别只在单帧投影区域。

## 5.2 推荐启动命令

cd ~/vscode_workspace/ros2_wp/src/semantic_map_offline  
conda activate opi_yolo_eval  
<br/>python scripts/evaluate_sam_projection_tracking.py \\  
\--data-root /home/weiyu/vscode_workspace/my_data \\  
\--model /home/weiyu/vscode_workspace/models/yolo11n.pt \\  
\--output evaluation/replica_sam_0800_1099 \\  
\--start 800 \\  
\--frames 300 \\  
\--device 0 \\  
\--confidence 0.35 \\  
\--pose-convention replica \\  
\--pixel-stride 1 \\  
\--voxel-size 0.01 \\  
\--overlap-radius 0.04 \\  
\--max-centroid-distance-m 1.0 \\  
\--min-geometric-overlap 0.05 \\  
\--min-bbox-overlap 0.0 \\  
\--association-threshold 0.45 \\  
\--geometry-weight 0.7 \\  
\--semantic-weight 0.3 \\  
\--observation-cluster-eps 0.10 \\  
\--observation-cluster-min-points 10 \\  
\--max-extent-growth 2.0 \\  
\--denoise-interval 20 \\  
\--map-merge-interval 20 \\  
\--map-merge-overlap 0.80 \\  
\--min-confirmed-observations 3 \\  
\--candidate-max-missed-frames 30 \\  
\--sam-checkpoint /home/weiyu/vscode_workspace/models/mobile_sam.pt \\  
\--sam-source /home/weiyu/vscode_workspace/ros2_wp/src/semantic_map_offline/MobileSAM \\  
\--sam-device cuda \\  
\--mask-erode-px 2

## 5.3 SAM专用参数

| 参数  | 含义  |
| --- | --- |
| \--sam-checkpoint | MobileSAM模型权重路径 |
| \--sam-source | MobileSAM Python源码目录 |
| \--sam-device | SAM推理设备，例如cuda或cpu |
| \--mask-erode-px | mask向内腐蚀像素数，减少物体边缘背景 |

SAM会为每个YOLO检测框生成一个mask；当 mask_erode_px > 0 时，程序使用 OpenCV 对mask执行一次腐蚀，再投影mask内部的深度点。

## 5.4 SAM和非SAM的区别

| 对比项 | 非 SAM | MobileSAM |
| --- | --- | --- |
| 2D区域 | YOLO矩形框 | SAM物体mask |
| 背景点 | 较多  | 通常更少 |
| 物体轮廓 | 矩形范围 | 更接近真实轮廓 |
| 点云密度 | 通常较低 | 可使用逐像素投影，通常更高 |
| 计算开销 | 较低  | 较高  |
| 额外模型 | 不需要 | mobile_sam.pt |
| 推荐pixel_stride | 2   | 1或2 |
| 推荐voxel_size | 0.02左右 | 0.01到0.015 |

需要注意：

SAM只改变“从二维图像中选择哪些像素投影到三维”的方式，不改变后续的对象关联和融合算法。

# 6\. 查看 Replica 结果

## 6.1 查看非SAM结果

cd ~/vscode_workspace/ros2_wp/src/semantic_map_offline  
conda activate opi_yolo_eval  
<br/>python scripts/view_tracked_objects_3d.py \\  
\--objects-dir evaluation/replica_bbox_0800_1099/objects \\  
\--min-observations 3 \\  
\--show-boxes \\  
\--show-origin

## 6.2 查看SAM结果

python scripts/view_tracked_objects_3d.py \\  
\--objects-dir evaluation/replica_sam_0800_1099/objects \\  
\--min-observations 3 \\  
\--show-boxes \\  
\--show-origin

二维俯视图可以使用：

python scripts/view_objects_2d.py \\  
\--objects-dir evaluation/replica_sam_0800_1099/objects

# 7\. SEMANTIC_05_BAG 数据说明

rosbag默认路径：

/home/weiyu/vscode_workspace/ros2_wp/ros_bag/semantic_05

主要数据包括：

/scan  
/odom  
/camera/accel/sample  
/camera/gyro/sample  
/camera/color/image_raw  
/camera/depth/image_raw  
/tf_static

该bag中的RGB和深度均为 640×480，深度已经注册到RGB相机坐标模型，因此不需要额外启动 depth_image_proc。使用的相机内参是：

fx = 365.1741638183594  
fy = 365.42144775390625  
cx = 318.27630615234375  
cy = 243.80377197265625  
depth_scale = 0.001

# 8\. SEMANTIC_05 推荐运行方式

对于 semantic_05，推荐使用两阶段流程：

第一阶段：只运行YOLO，生成results.jsonl  
↓  
第二阶段：复用results.jsonl，运行SLAM、投影和融合

这样可以：

1.  避免YOLO、SLAM、SAM和点云融合同时争抢GPU、CPU；
2.  使SAM和非SAM使用完全相同的检测结果；
3.  更方便重复调整投影和融合参数；
4.  避免每次测试重新执行YOLO。

项目测试文档同样推荐该两阶段运行方式。

# 9\. SEMANTIC_05 第一阶段：生成YOLO JSONL

SAM和非SAM可以共用同一份 YOLO 检测结果，因此第一阶段只需要执行一次。

## 9.1 终端1：启动YOLO-World节点

source /opt/ros/jazzy/setup.bash  
source ~/vscode_workspace/ros2_wp/install/setup.bash  
<br/>rm -rf /tmp/semantic_05_world_yolo  
<br/>ros2 run semantic_map_offline yolo_world_recorder_node --ros-args \\  
\-p use_sim_time:=true \\  
\-p image_topic:=/camera/color/image_raw \\  
\-p result_topic:=/yolo/results_json \\  
\-p output_dir:=/tmp/semantic_05_world_yolo \\  
\-p detect_model:=/home/weiyu/vscode_workspace/models/yolov8s-world.pt \\  
\-p clip_model:=/home/weiyu/vscode_workspace/models/clip/ViT-B-32.pt \\  
\-p classes_path:=$(ros2 pkg prefix semantic_map_offline)/share/semantic_map_offline/config/class_list/gpt_indoor_general.txt \\  
\-p excluded_labels:=person,wall,floor,ceiling,unknown \\  
\-p device:=0 \\  
\-p imgsz:=640 \\  
\-p conf:=0.35 \\  
\-p iou:=0.45 \\  
\-p frame_skip:=0 \\  
\-p save_annotated_images:=true \\  
\-p save_every_n:=30 \\  
\-p image_jpeg_quality:=90 \\  
\-p publish_debug_image:=true

## 9.2 终端2：只播放RGB图像

source /opt/ros/jazzy/setup.bash  
source ~/vscode_workspace/ros2_wp/install/setup.bash  
<br/>ros2 bag play \\  
~/vscode_workspace/ros2_wp/ros_bag/semantic_05 \\  
\--clock \\  
\--topics /camera/color/image_raw

第一阶段结束后，主要检测结果位于：

/tmp/semantic_05_world_yolo/  
├── latest.json  
├── results.jsonl  
├── latest.jpg  
└── frames/

后续投影真正需要的是：

/tmp/semantic_05_world_yolo/results.jsonl

记录时间戳来自输入RGB消息的 header.stamp，可在第二阶段与深度帧进行时间匹配。

# 10\. SEMANTIC_05 非 SAM 流程

## 10.1 节点链路

semantic_05 rosbag  
↓  
Cartographer离线建图  
↓  
生成 map → camera_color_optical_frame TF  
↓  
读取results.jsonl检测结果  
↓  
offline_projector_node  
↓ bbox内部深度点投影  
/semantic_offline/points  
↓  
object_fusion_node  
↓  
/semantic_offline/fused_points  
↓  
PLY + NPZ + semantic_objects.json

非 SAM 的专用入口为：

launch/semantic_05_bag_test.launch.py

该Launch会启动：

Cartographer和bag回放  
offline_projector_node  
object_fusion_node

当：

run_yolo:=true

时，还会额外启动：

yolo_world_recorder_node

## 10.2 推荐启动命令

完成第一阶段后运行：

source /opt/ros/jazzy/setup.bash  
source ~/vscode_workspace/ros2_wp/install/setup.bash  
<br/>rm -rf /tmp/semantic_05_bbox_output  
<br/>ros2 launch semantic_map_offline semantic_05_bag_test.launch.py \\  
bag_uri:=/home/weiyu/vscode_workspace/ros2_wp/ros_bag/semantic_05 \\  
run_yolo:=false \\  
jsonl_path:=/tmp/semantic_05_world_yolo/results.jsonl \\  
output_directory:=/tmp/semantic_05_bbox_output \\  
snapshot_path:=/tmp/semantic_05_bbox_output/semantic_objects.json \\  
projected_frames_dir:="" \\  
rate:=0.5 \\  
start_offset:=0.0 \\  
bag_delay_s:=5.0 \\  
enable_rviz:=true \\  
frame_skip:=0 \\  
pixel_stride:=2 \\  
voxel_size:=0.015 \\  
overlap_radius:=0.04 \\  
min_confidence:=0.35

项目文档中的已验证命令同样使用：

run_yolo=false  
jsonl_path=已经生成的results.jsonl  
rate=0.5

非SAM流程默认使用检测框内部点云，processing_delay_frames=5，用于等待对应深度时间戳的Cartographer TF可用。

## 10.3 启动后主要节点

可以使用：

ros2 node list

正常情况下应包含类似节点：

/semantic_map_offline_projector  
/semantic_map_offline_fusion  
/cartographer_node  
/cartographer_occupancy_grid_node

当 run_yolo:=false 时，不会启动：

/yolo_world_recorder_node

投影节点直接从：

/tmp/semantic_05_world_yolo/results.jsonl

读取检测结果。

## 10.4 主要话题

/map  
/semantic_offline/points  
/semantic_offline/detections  
/semantic_offline/fused_points  
/semantic_offline/objects  
/semantic_offline/object_markers

数据含义：

| 话题  | 内容  |
| --- | --- |
| /map | Cartographer二维占据栅格地图 |
| /semantic_offline/points | 当前帧投影后的对象点云 |
| /semantic_offline/detections | 当前帧投影元数据 |
| /semantic_offline/fused_points | 跨帧累计融合对象点云 |
| /semantic_offline/objects | 当前confirmed对象JSON |
| /semantic_offline/object_markers | 对象3D边界框和track状态 |

## 10.5 输出目录

/tmp/semantic_05_bbox_output/  
├── objects/  
│ ├── object_0001_chair.ply  
│ ├── object_0001_chair.npz  
│ └── ...  
└── semantic_objects.json

当：

projected_frames_dir:=""

时，不保存逐帧投影文件，只保存融合对象结果。

需要保存每帧投影点云时，可以改为：

projected_frames_dir:=/tmp/semantic_05_bbox_projected_frames

# 11\. SEMANTIC_05 MobileSAM 流程

## 11.1 节点链路

semantic_05 rosbag  
↓  
Cartographer离线建图  
↓  
map → camera_color_optical_frame TF  
↓  
读取results.jsonl中的YOLO检测框  
↓  
sam_offline_projector_node  
↓  
YOLO框提示MobileSAM  
↓  
生成并腐蚀mask  
↓  
投影mask内部深度点  
↓  
/semantic_offline/points  
↓  
object_fusion_node  
↓  
PLY + NPZ + semantic_objects.json

SAM专用入口是：

launch/semantic_05_sam_bag_test.launch.py

该Launch内部包含：

Cartographer与bag回放  
可选yolo_world_recorder_node  
sam_offline_projection.launch.py

而 sam_offline_projection.launch.py 会启动：

sam_offline_projector_node  
object_fusion_node

## 11.2 推荐启动命令

source /opt/ros/jazzy/setup.bash  
source ~/vscode_workspace/ros2_wp/install/setup.bash  
<br/>rm -rf /tmp/semantic_05_sam_output  
<br/>ros2 launch semantic_map_offline semantic_05_sam_bag_test.launch.py \\  
bag_uri:=/home/weiyu/vscode_workspace/ros2_wp/ros_bag/semantic_05 \\  
run_yolo:=false \\  
jsonl_path:=/tmp/semantic_05_world_yolo/results.jsonl \\  
sam_checkpoint:=/home/weiyu/vscode_workspace/models/mobile_sam.pt \\  
sam_source:=/home/weiyu/vscode_workspace/ros2_wp/src/semantic_map_offline/MobileSAM \\  
sam_device:=cuda \\  
mask_erode_px:=1 \\  
output_directory:=/tmp/semantic_05_sam_output \\  
snapshot_path:=/tmp/semantic_05_sam_output/semantic_objects.json \\  
projected_frames_dir:="" \\  
rate:=0.25 \\  
start_offset:=0.0 \\  
bag_delay_s:=5.0 \\  
enable_rviz:=true \\  
publish_debug_image:=true \\  
publish_markers:=true \\  
pixel_stride:=1 \\  
voxel_size:=0.01 \\  
overlap_radius:=0.04 \\  
min_confidence:=0.35

项目中当前SAM默认配置为：

sam_device=cuda  
mask_erode_px=1  
pixel_stride=1  
voxel_size=0.01  
rate=0.25

较低的bag播放速率用于给SAM推理和高密度点云融合留出处理时间。

## 11.3 SAM调试图

SAM节点会发布：

/semantic_offline/sam_debug_image

另开终端查看：

source /opt/ros/jazzy/setup.bash  
source ~/vscode_workspace/ros2_wp/install/setup.bash  
<br/>ros2 run rqt_image_view rqt_image_view \\  
/semantic_offline/sam_debug_image

建议按以下顺序检查：

SAM mask  
↓  
当前帧投影点云  
↓  
融合对象点云

如果mask正确、当前帧点云正确，但融合结果错误，问题通常集中在对象关联参数；如果当前帧点云已经错位，应优先检查RGB-depth注册、相机内参和TF。

# 12\. SEMANTIC_05 单阶段运行

两阶段流程更适合正式实验，但也可以在一次Launch中同时运行：

YOLO  
SLAM  
投影  
融合

## 12.1 非SAM单阶段预览

source /opt/ros/jazzy/setup.bash  
source ~/vscode_workspace/ros2_wp/install/setup.bash  
<br/>OUTPUT_ROOT=$HOME/semantic_map_outputs/semantic_05_bbox_full  
rm -rf "$OUTPUT_ROOT"  
mkdir -p "$OUTPUT_ROOT"  
<br/>ros2 launch semantic_map_offline semantic_05_bag_test.launch.py \\  
run_yolo:=true \\  
yolo_output_dir:="$OUTPUT_ROOT/yolo" \\  
save_yolo_images:=true \\  
yolo_save_every_n:=30 \\  
publish_yolo_debug_image:=true \\  
output_directory:="$OUTPUT_ROOT/map" \\  
snapshot_path:="$OUTPUT_ROOT/map/semantic_objects.json" \\  
rate:=0.5 \\  
enable_rviz:=true

数据链路：

RGB  
↓  
yolo_world_recorder_node  
├── 保存YOLO结果  
└── 发布/yolo/results_json  
↓  
offline_projector_node  
↓  
object_fusion_node

## 12.2 SAM单阶段运行

source /opt/ros/jazzy/setup.bash  
source ~/vscode_workspace/ros2_wp/install/setup.bash  
<br/>OUTPUT_ROOT=$HOME/semantic_map_outputs/semantic_05_sam_full  
rm -rf "$OUTPUT_ROOT"  
mkdir -p "$OUTPUT_ROOT"  
<br/>ros2 launch semantic_map_offline semantic_05_sam_bag_test.launch.py \\  
run_yolo:=true \\  
yolo_output_dir:="$OUTPUT_ROOT/yolo" \\  
save_yolo_images:=true \\  
yolo_save_every_n:=30 \\  
yolo_image_jpeg_quality:=90 \\  
publish_yolo_debug_image:=true \\  
sam_checkpoint:=/home/weiyu/vscode_workspace/models/mobile_sam.pt \\  
sam_source:=/home/weiyu/vscode_workspace/ros2_wp/src/semantic_map_offline/MobileSAM \\  
sam_device:=cuda \\  
mask_erode_px:=1 \\  
output_directory:="$OUTPUT_ROOT/map" \\  
snapshot_path:="$OUTPUT_ROOT/map/semantic_objects.json" \\  
rate:=0.25 \\  
enable_rviz:=true

项目测试文档也提供了一次回放同时运行YOLO、SAM和融合的完整示例。

# 13\. 两阶段和单阶段的选择

| 运行方式 | 优点  | 缺点  | 推荐用途 |
| --- | --- | --- | --- |
| 两阶段 | 资源压力小、结果可重复、便于调参 | 需要运行两次bag | 正式实验 |
| 单阶段 | 操作简单、可快速看到结果 | YOLO、SLAM、SAM和融合同时占资源 | 快速预览 |
| Replica脚本 | 不依赖ROS和TF，便于算法验证 | 只能使用数据集已有位姿 | 离线算法评估 |

正式对比 SAM 和非 SAM 时，推荐：

同一份results.jsonl  
├── semantic_05_bag_test.launch.py  
└── semantic_05_sam_bag_test.launch.py

这样可以保证两组实验使用完全相同的YOLO框，差异主要来自bbox投影与SAM mask投影。

# 14\. RViz和运行状态检查

## 14.1 检查核心话题频率

ros2 topic hz /map  
ros2 topic hz /semantic_offline/points  
ros2 topic hz /semantic_offline/fused_points

## 14.2 检查TF

ros2 run tf2_ros tf2_echo \\  
map \\  
camera_color_optical_frame

## 14.3 检查YOLO结果

实时运行YOLO时：

ros2 topic echo /yolo/results_json --once

## 14.4 查看YOLO检测图

ros2 run rqt_image_view rqt_image_view /yolo/debug_image

## 14.5 查看SAM mask

ros2 run rqt_image_view rqt_image_view \\  
/semantic_offline/sam_debug_image

# 15\. 查看SEMANTIC_05保存结果

## 15.1 非SAM

cd ~/vscode_workspace/ros2_wp/src/semantic_map_offline  
conda activate opi_yolo_eval  
<br/>python scripts/view_tracked_objects_3d.py \\  
\--objects-dir /tmp/semantic_05_bbox_output/objects \\  
\--min-observations 3 \\  
\--show-boxes \\  
\--show-origin

## 15.2 SAM

python scripts/view_tracked_objects_3d.py \\  
\--objects-dir /tmp/semantic_05_sam_output/objects \\  
\--min-observations 3 \\  
\--show-boxes \\  
\--show-origin

# 16\. 四种启动命令快速索引

## 16.1 Replica非SAM

python scripts/evaluate_projection_roundtrip.py \\  
\--data-root /home/weiyu/vscode_workspace/my_data \\  
\--model /home/weiyu/vscode_workspace/models/yolo11n.pt \\  
\--output evaluation/replica_bbox_0800_1099 \\  
\--start 800 --frames 300 \\  
\--device 0 \\  
\--pose-convention replica \\  
\--pixel-stride 2

## 16.2 Replica MobileSAM

python scripts/evaluate_sam_projection_tracking.py \\  
\--data-root /home/weiyu/vscode_workspace/my_data \\  
\--model /home/weiyu/vscode_workspace/models/yolo11n.pt \\  
\--output evaluation/replica_sam_0800_1099 \\  
\--start 800 --frames 300 \\  
\--device 0 \\  
\--pose-convention replica \\  
\--pixel-stride 1 \\  
\--sam-checkpoint /home/weiyu/vscode_workspace/models/mobile_sam.pt \\  
\--sam-source /home/weiyu/vscode_workspace/ros2_wp/src/semantic_map_offline/MobileSAM \\  
\--sam-device cuda \\  
\--mask-erode-px 2

## 16.3 SEMANTIC_05非SAM

ros2 launch semantic_map_offline semantic_05_bag_test.launch.py \\  
run_yolo:=false \\  
jsonl_path:=/tmp/semantic_05_world_yolo/results.jsonl \\  
output_directory:=/tmp/semantic_05_bbox_output \\  
snapshot_path:=/tmp/semantic_05_bbox_output/semantic_objects.json \\  
rate:=0.5 \\  
enable_rviz:=true

## 16.4 SEMANTIC_05 MobileSAM

ros2 launch semantic_map_offline semantic_05_sam_bag_test.launch.py \\  
run_yolo:=false \\  
jsonl_path:=/tmp/semantic_05_world_yolo/results.jsonl \\  
sam_checkpoint:=/home/weiyu/vscode_workspace/models/mobile_sam.pt \\  
sam_source:=/home/weiyu/vscode_workspace/ros2_wp/src/semantic_map_offline/MobileSAM \\  
sam_device:=cuda \\  
output_directory:=/tmp/semantic_05_sam_output \\  
snapshot_path:=/tmp/semantic_05_sam_output/semantic_objects.json \\  
rate:=0.25 \\  
enable_rviz:=true

# 17\. 最终流程关系图

semantic_map_offline  
│  
┌───────────────────┴───────────────────┐  
│ │  
Replica SEMANTIC_05_BAG  
│ │  
直接读取本地文件 ROS 2播放rosbag  
RGB+Depth+traj.txt │  
│ Cartographer生成TF  
┌───────┴────────┐ ┌────────┴─────────┐  
│ │ │ │  
非SAM SAM 非SAM SAM  
│ │ │ │  
YOLO检测框 YOLO框+SAM mask offline_projector sam_offline_projector  
│ │ │ │  
bbox深度投影 mask深度投影 bbox深度投影 mask深度投影  
│ │ │ │  
└───────┬────────┘ └────────┬─────────┘  
│ │  
ObjectTracker object_fusion_node  
│ │  
└───────────────────┬───────────────────┘  
│  
跨帧对象关联和融合  
│  
PLY + NPZ + semantic_objects.json