# semantic_map_offline 项目参数说明

## 1\. 参数层级

项目中存在三类参数：

1.  **ROS 2节点参数**：通过 ros2 run ... --ros-args -p 设置；
2.  **Launch参数**：通过 ros2 launch ... 参数名:=值 设置，可能会映射成不同名称的节点参数；
3.  **离线脚本参数**：Replica评估和可视化脚本使用 --参数名 设置。

主要数据链路：

YOLO检测  
↓  
offline_projector_node / sam_offline_projector_node  
↓  
object_fusion_node  
↓  
PLY、NPZ、semantic_objects.json

# 2\. YOLO-World检测节点参数

节点：

yolo_world_recorder_node

作用文件：

semantic_map_offline/yolo_world_recorder_node.py  
semantic_map_offline/world_detection.py

## 2.1 输入、输出与模型

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| image_topic | /camera/color/image_raw | RGB输入话题 |
| result_topic | /yolo/results_json | 检测JSON发布话题 |
| debug_image_topic | /yolo/debug_image | 检测框调试图话题 |
| output_dir | /tmp/yolo_world_results | JSON和检测图保存目录 |
| detect_model | 空   | YOLO-World模型路径 |
| clip_model | 空   | CLIP模型路径 |
| classes_path | 空   | 开放词汇类别文件 |
| excluded_labels | person,wall,floor,ceiling,unknown | 从类别词表中排除的类别 |
| device | 0   | 推理设备，例如0、1、cpu |

excluded_labels 在模型加载前过滤类别词表，因此输出的 class_id 是过滤后类别列表中的序号。

## 2.2 推理参数

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| imgsz | 640 | YOLO推理输入尺寸 |
| conf | 0.25 | 最低检测置信度 |
| iou | 0.45 | NMS重叠框抑制阈值 |
| frame_skip | 0   | 每次推理后跳过的输入帧数 |

frame_skip=0 表示每帧推理；frame_skip=2 表示每3帧推理一次。

## 2.3 保存与调试参数

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| save_annotated_images | true | 是否保存检测框图片 |
| save_every_n | 30  | 每N次推理归档一张检测图 |
| image_jpeg_quality | 85  | JPEG质量，范围1～100 |
| publish_debug_image | true | 是否发布/yolo/debug_image |

输出目录：

output_dir/  
├── latest.json  
├── results.jsonl  
├── latest.jpg  
└── frames/\*.jpg

其中：

- latest.json：每次覆盖；
- results.jsonl：每次追加；
- latest.jpg和frames/\*.jpg受 save_annotated_images 控制；
- 当前代码没有单独关闭JSON保存的参数。

# 3\. 普通检测框投影节点参数

节点：

offline_projector_node

作用文件：

semantic_map_offline/offline_projector_node.py  
semantic_map_offline/bbox_projection.py

## 3.1 输入输出

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| input_topic | /yolo/results_json | 实时检测JSON输入 |
| jsonl_path | 空   | 离线检测JSONL路径 |
| depth_topic | /camera/depth/image_raw | 深度图话题 |
| color_topic | /camera/color/image_raw | RGB图像话题 |
| cloud_topic | /semantic_offline/points | 当前帧对象点云输出 |
| metadata_topic | /semantic_offline/detections | 当前帧投影元数据输出 |
| save_directory | 空   | 逐帧投影NPZ和JSON保存目录 |

当 jsonl_path 非空时，节点读取JSONL文件，不再订阅 input_topic。

当 save_directory="" 时，不保存逐帧中间结果，但点云发布和对象融合不受影响。

## 3.2 坐标系与相机参数

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| camera_frame | 空   | 深度点所在相机坐标系；为空时使用深度消息的frame_id |
| target_frame | map | 投影后的目标坐标系 |
| camera_fx | 311.3878784 | 水平方向焦距 |
| camera_fy | 311.3878784 | 垂直方向焦距 |
| camera_cx | 317.5 | 主点横坐标 |
| camera_cy | 198.5 | 主点纵坐标 |
| depth_scale | 0.001 | 原始深度值到米的比例 |
| min_depth_m | 0.3 | 最小有效深度，单位米 |
| max_depth_m | 5.0 | 最大有效深度，单位米 |

相机内参必须对应投影使用的深度图。如果RGB和深度未注册，仅缩放检测框分辨率无法补偿两相机之间的外参误差。

## 3.3 投影清理参数

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| pixel_stride | 2   | 检测区域内每隔多少像素取一个深度点 |
| projection_voxel_size | 0.0 | 单帧投影体素尺寸；0表示关闭 |
| projection_cluster_eps | 0.0 | 单帧最大空间簇邻域半径；0表示关闭 |
| projection_cluster_min_points | 10  | 单帧空间簇最少点数 |

这组参数只清理**当前帧投影点云**：

检测框深度点  
↓ projection_voxel_size  
单帧体素降采样  
↓ projection_cluster_eps  
保留最大的空间连通簇

projection_voxel_size 与融合阶段的 voxel_size 不是同一个参数。前者处理单帧，后者处理跨帧对象模型。

注意：当前 offline_projection.launch.py 没有暴露这三个单帧清理参数，直接启动节点时可以设置；SAM通用Launch已暴露这些参数。

## 3.4 同步与过滤参数

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| max_time_diff_s | 0.15 | 检测、深度和RGB允许的最大时间差 |
| detection_buffer_size | 100 | 检测和深度缓存容量 |
| color_buffer_size | 30  | RGB缓存容量 |
| processing_delay_frames | 0   | 延迟多少帧后再处理深度，用于等待TF |
| tf_timeout_s | 0.3 | 查询TF时的等待时间 |
| min_confidence | 0.0 | 投影前再次过滤低置信度检测 |
| excluded_labels | person | 不参与投影的类别名称 |
| excluded_class_ids | 空   | 不参与投影的类别ID |
| image_qos_reliable | false | 图像订阅使用Reliable还是Best Effort |

processing_delay_frames 只延迟处理时机，TF查询仍使用深度图原始时间戳。

# 4\. MobileSAM投影节点参数

节点：

sam_offline_projector_node

作用文件：

semantic_map_offline/sam_offline_projector_node.py  
semantic_map_offline/mask_projection.py

该节点继承普通投影节点，因此支持第3节的全部参数，并额外增加：

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| sam_checkpoint | 空，必填 | MobileSAM权重路径 |
| sam_source | 空   | MobileSAM源码目录 |
| sam_device | 空   | SAM设备；为空时自动选择CUDA或CPU |
| mask_erode_px | 2   | mask向内腐蚀像素数 |
| publish_debug_image | true | 是否发布SAM调试叠加图 |
| debug_image_topic | /semantic_offline/sam_debug_image | SAM调试图话题 |
| debug_mask_alpha | 0.45 | mask叠加透明度，范围0～1 |

mask_erode_px=0 表示不腐蚀。设置过大会损失物体边缘，设置适当可减少检测框边缘背景。

SAM流程：

YOLO检测框  
↓  
MobileSAM mask  
↓ mask_erode_px  
mask腐蚀  
↓  
只投影mask内部有效深度点

# 5\. 对象关联与融合参数

节点：

object_fusion_node

作用文件：

semantic_map_offline/object_fusion_node.py  
semantic_map_offline/object_tracker.py

## 5.1 话题与保存

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| cloud_topic | /semantic_offline/points | 当前帧对象点云输入 |
| metadata_topic | /semantic_offline/detections | 投影元数据输入 |
| fused_cloud_topic | /semantic_offline/fused_points | 融合点云输出 |
| objects_topic | /semantic_offline/objects | 对象JSON输出 |
| marker_topic | /semantic_offline/object_markers | 对象边界框Marker输出 |
| publish_markers | true | 是否发布对象Marker |
| output_directory | 空   | 对象PLY、NPZ保存目录 |
| snapshot_path | 空   | semantic_objects.json保存路径 |
| snapshot_interval_s | 2.0 | 磁盘快照最小时间间隔 |
| sync_buffer_size | 50  | 点云与元数据同步缓存容量 |

output_directory 和 snapshot_path 为空时不写对应磁盘文件，但ROS话题仍可正常发布。

## 5.2 单帧观测清理

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| voxel_size | 0.02 | 对象观测和融合模型的体素尺寸，单位米 |
| observation_cluster_eps | 0.10 | 空间聚类邻域半径，单位米 |
| observation_cluster_min_points | 10  | 空间簇最少点数 |

处理顺序：

单个对象观测  
↓ voxel_size  
体素降采样  
↓ observation_cluster_eps  
保留最大空间簇

## 5.3 对象关联门限

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| max_centroid_distance_m | 1.0 | 观测与对象质心最大距离 |
| overlap_radius | 0.04 | 最近邻几何重叠半径 |
| min_geometric_overlap | 0.05 | 最低几何重叠率 |
| min_bbox_overlap | 0.0 | 最低3D AABB重叠率 |
| association_threshold | 0.45 | 最低综合关联分数 |
| geometry_weight | 0.7 | 几何重叠权重 |
| semantic_weight | 0.3 | 类别历史相似度权重 |
| max_extent_growth | 2.0 | 融合后各轴尺寸允许的最大增长倍数 |

关联分数：

score =  
geometry_weight × geometric_overlap  
+  
semantic_weight × semantic_similarity

两个权重会在代码中自动归一化，因此不要求手动保证和为1，但两者必须至少有一个大于0。

需要注意：

- geometric_overlap 是双向最近邻覆盖率中较大的一个；
- min_bbox_overlap 使用“交集体积÷较小AABB体积”，不是标准3D IoU；
- 所有门限满足后才会进入全局一对一分配。

## 5.4 状态维护和地图清理

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| min_confirmed_observations | 3   | candidate转confirmed所需观测次数 |
| candidate_max_missed_frames | 30  | candidate允许连续未匹配帧数 |
| stale_after_s | 0.0 | 超过多少秒未观测后删除；0关闭 |
| denoise_interval | 20  | 每N帧对所有对象执行一次最大簇去噪 |
| map_merge_interval | 20  | 每N帧检查一次重复对象 |
| map_merge_overlap | 0.80 | 同类别对象自动合并所需重叠率 |
| non_fusing_labels | person | 不累计历史几何的类别 |

non_fusing_labels 中的类别仍会进行追踪和语义更新，但几何点云使用最新观测替换，不持续累积历史点。

# 6\. Replica多帧评估参数

入口：

scripts/evaluate_projection_roundtrip.py  
scripts/evaluate_sam_projection_tracking.py

参数实际定义和生效文件：

semantic_map_offline/tracking_pipeline.py

## 6.1 公共参数

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| \--data-root | 必填  | Replica数据根目录 |
| \--model | 必填  | YOLO模型路径 |
| \--output | 必填  | 输出目录 |
| \--start | 800 | 起始帧 |
| \--frames | 15  | 处理帧数 |
| \--confidence | 0.35 | YOLO置信度阈值 |
| \--device | 空   | YOLO设备，空表示自动选择 |
| \--pixel-stride | 2   | 深度像素采样步长 |
| \--pose-convention | replica | 相机位姿约定 |

\--pose-convention 可选：

| 值   | 含义  |
| --- | --- |
| replica | 标准Replica traj.txt |
| opencv | 当前实现与replica相同，直接使用c2w矩阵 |
| replica_opengl_legacy | 兼容旧实验，额外翻转Y、Z轴 |

## 6.2 Replica融合参数

| 参数  | 默认值 |
| --- | --- |
| \--voxel-size | 0.02 |
| \--overlap-radius | 0.04 |
| \--max-centroid-distance-m | 1.0 |
| \--min-geometric-overlap | 0.05 |
| \--min-bbox-overlap | 0.0 |
| \--association-threshold | 0.45 |
| \--geometry-weight | 0.7 |
| \--semantic-weight | 0.3 |
| \--observation-cluster-eps | 0.10 |
| \--observation-cluster-min-points | 10  |
| \--max-extent-growth | 2.0 |
| \--denoise-interval | 20  |
| \--map-merge-interval | 20  |
| \--map-merge-overlap | 0.80 |
| \--min-confirmed-observations | 3   |
| \--candidate-max-missed-frames | 30  |

这些参数与ROS融合节点中的同名参数作用一致。

## 6.3 Replica SAM附加参数

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| \--sam-checkpoint | 必填  | MobileSAM权重 |
| \--sam-source | 项目下MobileSAM | MobileSAM源码目录 |
| \--sam-device | 空   | SAM设备，空时自动选择 |
| \--mask-erode-px | 2   | mask腐蚀像素数 |

# 7\. Replica单帧闭环验证参数

入口：

scripts/evaluate_single_frame_roundtrip.py

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| \--data-root | 必填  | Replica数据目录 |
| \--model | 必填  | YOLO模型路径 |
| \--output | 必填  | 输出目录 |
| \--start | 800 | 起始帧 |
| \--frames | 15  | 验证帧数 |
| \--confidence | 0.35 | 检测阈值 |
| \--pixel-stride | 2   | 深度采样步长 |
| \--visibility-tolerance-m | 0.10 | 反投影深度与当前深度表面允许误差 |

该脚本不进行跨帧追踪，只验证：

2D检测框 → 3D点云 → 2D反投影

并计算像素误差和检测框IoU。

# 8\. rosbag专用Launch参数

## 8.1 通用回放参数

| Launch参数 | 作用  |
| --- | --- |
| bag_uri | rosbag目录 |
| rate | rosbag播放倍速 |
| start_offset | 从bag第几秒开始播放 |
| bag_delay_s | 启动节点后延迟多少秒播放bag |
| enable_rviz | 是否启动RViz |
| rviz_config | RViz配置路径 |
| run_yolo | 是否同时启动检测节点 |
| jsonl_path | 使用已经保存的YOLO JSONL |

推荐二阶段方式：

第一阶段：run_yolo=true，生成results.jsonl  
第二阶段：run_yolo=false，jsonl_path=results.jsonl

## 8.2 YOLO-World Launch别名

仅 semantic_05 的包内YOLO-World流程使用：

| Launch参数 | 实际节点参数 |
| --- | --- |
| yolo_device | device |
| yolo_excluded_labels | excluded_labels |
| yolo_output_dir | output_dir |
| save_yolo_images | save_annotated_images |
| yolo_save_every_n | save_every_n |
| yolo_image_jpeg_quality | image_jpeg_quality |
| publish_yolo_debug_image | publish_debug_image |
| min_confidence | conf，同时也传给投影节点 |

## 8.3 输出目录别名

| Launch参数 | 实际节点参数 | 保存内容 |
| --- | --- | --- |
| yolo_output_dir | YOLO的output_dir | 检测JSON和检测图片 |
| projected_frames_dir | 投影节点的save_directory | 每帧投影NPZ和JSON |
| output_directory | 融合节点同名参数 | 对象PLY和NPZ |
| snapshot_path | 融合节点同名参数 | semantic_objects.json |

# 9\. 各bag专用Launch差异

## 9.1 semantic_05_bag_test.launch.py

非SAM，使用包内YOLO-World。

主要默认值：

| 参数  | 默认值 |
| --- | --- |
| run_yolo | false |
| rate | 0.5 |
| pixel_stride | 2   |
| voxel_size | 0.015 |
| overlap_radius | 0.04 |
| min_confidence | 0.35 |
| processing_delay_frames | 固定为5 |
| image_qos_reliable | 固定为true |

深度已注册到RGB，目标坐标系为 map。

## 9.2 semantic_05_sam_bag_test.launch.py

主要SAM默认值：

| 参数  | 默认值 |
| --- | --- |
| rate | 0.25 |
| pixel_stride | 1   |
| voxel_size | 0.01 |
| mask_erode_px | 1   |
| sam_device | cuda |
| publish_debug_image | true |
| publish_markers | true |

较低播放速率用于给SAM推理和高密度点云处理留出计算时间。

## 9.3 semantic_01_bag_test.launch.py

该Launch使用外部包：

opi_yolo_rknn_recorder/ai_recorder_node

专用参数：

| 参数  | 作用  |
| --- | --- |
| classes_filter | 外部YOLO允许检测的类别ID |
| enable_http | 是否启动外部检测结果HTTP服务 |
| http_port | HTTP端口 |

默认使用无里程计Cartographer配置，pixel_stride=4。外部检测节点的完整参数定义不在本仓库中。

## 9.4 semantic_bags_01_test.launch.py

主要差异：

- 使用带轮式里程计的Cartographer配置；
- 启动 depth_image_proc/register_node；
- 将深度注册到RGB相机；
- 投影节点使用 /camera/depth_registered/image_raw；
- 默认 pixel_stride=4。

## 9.5 meeting_room_sam_test.launch.py

该流程额外启动：

compressed_rgbd_decoder_node  
odom_camera_tf_node  
sam_offline_projector_node

专用参数：

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| camera_x/y/z | 0   | 雷达到相机的平移外参 |
| camera_qx/qy/qz/qw | 标准光学轴旋转 | 雷达到相机的旋转外参 |
| projection_cluster_eps | 0.03 | 单帧点云最大簇邻域半径 |
| projection_cluster_min_points | 10  | 单帧空间簇最少点数 |
| rate | 0.15 | bag播放速度 |
| target_frame | 固定为odom | 语义点云目标坐标系 |

该bag缺少准确的雷达到相机外参，正式使用前应标定并替换 camera_x/y/z/qx/qy/qz/qw。

# 10\. 压缩RGB-D解码节点

节点：

compressed_rgbd_decoder_node

作用文件：

semantic_map_offline/compressed_rgbd_decoder_node.py

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| color_compressed_topic | /orbbec_camera/color/image_raw/compressed | 压缩RGB输入 |
| depth_compressed_topic | /orbbec_camera/depth/image_raw/compressedDepth | 压缩深度输入 |
| color_raw_topic | /camera/color/image_raw | 解码后RGB输出 |
| depth_raw_topic | /camera/depth/image_raw | 解码后深度输出 |

深度输出格式为 16UC1。

# 11\. 里程计与相机TF节点

节点：

odom_camera_tf_node

作用文件：

semantic_map_offline/odom_camera_tf_node.py

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| odom_topic | /fastlio_odom | 里程计输入 |
| odom_frame | odom | 世界/里程计坐标系 |
| lidar_frame | lidar_link | 雷达坐标系 |
| camera_frame | orbbec_camera_color_optical_frame | 相机光学坐标系 |
| camera_x/y/z | 0   | 雷达到相机平移外参 |
| camera_qx/qy/qz/qw | 标准光学轴旋转 | 雷达到相机旋转外参 |
| path_topic | /meeting_room/trajectory | 轨迹输出话题 |
| path_max_poses | 2000 | 最多保存的轨迹位姿数量 |

节点发布：

odom → lidar_link 动态TF  
lidar_link → camera 静态TF

# 12\. 三维结果查看参数

脚本：

scripts/view_tracked_objects_3d.py

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| \--objects-dir | 必填  | 对象NPZ目录 |
| \--min-observations | 5   | 最低观测次数 |
| \--json-output | 自动  | 导航对象JSON输出 |
| \--classes | 空   | 类别过滤 |
| \--max-tracks | 0   | 最大显示对象数；0不限制 |
| \--display-voxel-size | 0   | 仅用于显示的体素降采样 |
| \--point-size | 2.0 | 点大小 |
| \--color-mode | rgb | 使用真实RGB或track颜色 |
| \--show-boxes | 关闭  | 显示3D包围框 |
| \--show-origin | 关闭  | 显示坐标原点 |
| \--background | dark | 深色或浅色背景 |

# 13\. 二维俯视图参数

脚本：

scripts/view_objects_2d.py

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| \--objects-dir | 必填  | 对象NPZ目录 |
| \--output | 自动  | PNG输出 |
| \--json-output | 自动  | 2D对象JSON输出 |
| \--min-observations | 5   | 最低观测次数 |
| \--classes | 空   | 类别过滤 |
| \--max-objects | 0   | 最大对象数 |
| \--pixels-per-meter | 180 | 每米对应像素数 |
| \--margin-m | 0.5 | 图像边缘留白，单位米 |
| \--min-canvas-px | 700 | 最小画布尺寸 |
| \--max-canvas-px | 2400 | 最大画布尺寸 |
| \--point-radius | 1   | 点绘制半径 |
| \--color-mode | object | 对象颜色或真实RGB |
| \--background | light | 背景模式 |
| \--show | 关闭  | 是否弹窗显示 |

# 14\. 语义对象叠加SLAM地图参数

脚本：

scripts/overlay_objects_on_slam_map.py

| 参数  | 默认值 | 作用  |
| --- | --- | --- |
| \--map-yaml | 必填  | ROS地图YAML |
| \--objects-dir | 必填  | 对象NPZ目录 |
| \--output | 必填  | 叠加图输出路径 |
| \--json-output | 空   | 像素坐标JSON输出 |
| \--min-observations | 5   | 最低观测次数 |
| \--point-radius | 1   | 点绘制半径 |
| \--point-opacity | 0.82 | 语义点透明度 |
| \--show | 关闭  | 是否弹窗显示 |

# 15\. 最容易混淆的参数

| 参数  | 所属阶段 | 说明  |
| --- | --- | --- |
| conf | YOLO检测 | 检测模型内部过滤 |
| min_confidence | 3D投影 | 投影前再次过滤 |
| pixel_stride | 单帧投影 | 控制从深度图取点的密度 |
| projection_voxel_size | 单帧投影 | 当前帧点云降采样 |
| voxel_size | 对象融合 | 跨帧对象模型降采样 |
| projection_cluster_eps | 单帧投影 | 当前帧最大簇清理 |
| observation_cluster_eps | 对象融合 | 每个融合观测和对象模型清理 |
| min_geometric_overlap | 对象关联 | 最近邻几何覆盖率门限 |
| min_bbox_overlap | 对象关联 | AABB交集占较小框体积的比例 |
| map_merge_overlap | 地图维护 | 已有重复track之间的合并门限 |
| save_directory | 单帧投影 | 保存每帧中间结果 |
| output_directory | 对象融合 | 保存最终对象PLY和NPZ |
| snapshot_path | 对象融合 | 保存最终语义对象JSON |

# 16\. 参数作用文件索引

| 模块  | 参数生效文件 |
| --- | --- |
| YOLO-World检测 | yolo_world_recorder_node.py、world_detection.py |
| 检测框投影 | offline_projector_node.py、bbox_projection.py |
| SAM投影 | sam_offline_projector_node.py、mask_projection.py |
| 对象关联与融合 | object_fusion_node.py、object_tracker.py |
| Replica多帧评估 | tracking_pipeline.py |
| Replica单帧验证 | evaluate_single_frame_roundtrip.py |
| 压缩图像解码 | compressed_rgbd_decoder_node.py |
| 里程计与相机TF | odom_camera_tf_node.py |
| 3D查看 | view_tracked_objects_3d.py |
| 2D俯视图 | view_objects_2d.py |
| SLAM地图叠加 | overlay_objects_on_slam_map.py |