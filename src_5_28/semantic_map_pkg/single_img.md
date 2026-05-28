针对你现有的这套语义建图代码，当一帧 YOLO 检测结果（图像）到来时，完整的处理流程如下。整个管线可以划分为 检测 → 投影 → 融合 → 可视化 四个阶段，下面结合你已有的类逐一说明。

1. 获取原始检测结果（图像空间）
从 YOLO 等检测器得到一帧图像上的所有目标，将其封装为 Detection2D 列表：

python
detections: list[Detection2D] = []
for det in raw_yolo_results:
    detections.append(Detection2D(
        label=det.class_name,          # 如 "chair"
        confidence=det.confidence,
        center_u=det.box_center_x,     # 像素坐标
        center_v=det.box_center_y,
        width=det.box_width,
        height=det.box_height,
        track_id=det.track_id,         # 可选，若检测器自带跟踪则传入
    ))
如果 YOLO 输出已是归一化坐标，需乘以图像宽高转为像素值。

2. 准备投影所需的外部数据
投影需要 深度图、相机内参、相机→地图的变换，且这帧图像必须有时间对齐的深度图（如 RGB‑D 相机或双目估算）。

python
# 从 ROS 话题获取深度图（已转换为米，None 表示无效点）
depth_image: DepthImage = current_depth_meters

# 从 CameraInfo 或配置文件初始化内参
intrinsics = CameraIntrinsics.from_k_matrix(camera_info.k)

# 从 TF 树或里程计获取当前相机在地图坐标系下的位姿
# 构造 Transform3D：旋转用四元数，平移用相机光心坐标
camera_to_map = Transform3D(
    translation=(cam_x, cam_y, cam_z),
    rotation=Quaternion(qx, qy, qz, qw),
    source_frame="camera_link",
    target_frame="map",
)
3. 投影：图像目标 → 地图目标
使用 SemanticProjector 将 Detection2D 列表转换为 ProjectedObject 列表。
这一步完成了从像素到世界坐标的转换，并赋予物体类别先验尺寸。

python
projector = SemanticProjector(
    intrinsics=intrinsics,
    classes=custom_semantic_classes,   # 你定义的语义配置，可含尺寸、颜色
    depth_window_px=5,                 # 深度邻域大小
    min_confidence=0.35,               # 最低置信度
)

projected_objects = projector.project_detections(
    detections=detections,
    depth_image_m=depth_image,
    camera_to_map=camera_to_map,
)
内部原理：

低置信度、未在 classes 中注册的类别会被过滤。

取检测框中心的深度邻域中值，提高鲁棒性。

通过 pixel_to_camera + camera_to_map.apply 得到地图坐标 (center_x, center_y)。

物体的 yaw 暂时固定为 0.0，尺寸采用 classes 中预设的 size_x、size_y。

4. 多帧融合：维护稳定语义地图
将投影后的 ProjectedObject 喂给 SemanticObjectTracker，它会进行数据关联、指数平滑、过期清理，最终输出融合后的 SemanticObject 列表。

python
tracker = SemanticObjectTracker(
    association_distance_m=0.8,
    smoothing_alpha=0.35,
    max_age_s=30.0,
)

# 每帧调用一次 update
current_time = node.get_clock().now().nanoseconds * 1e-9
stable_objects = tracker.update(projected_objects, now=current_time)
stable_objects 就是当前地图中所有活跃、经过平滑的语义物体，它们的中心位置、尺寸都更加稳定，并保留了 first_seen、observations 等元信息。

5. 可视化（以 RViz MarkerArray 为例）
将每个 SemanticObject 通过 object_to_rectangle 转换为 Rectangle2D，再构建 Marker 并发布。

python
markers = MarkerArray()
for obj in stable_objects:
    rect = object_to_rectangle(obj)
    # 创建 LINE_STRIP 显示矩形边框
    line = Marker()
    line.header.frame_id = "map"
    line.ns = "semantic_rectangles"
    line.id = next_id()
    line.type = Marker.LINE_STRIP
    line.scale.x = 0.06
    line.color = ...  # 取自 rect.color_rgba
    line.points = [Point(x=x, y=y, z=0.05) for x, y in rect.points]
    markers.markers.append(line)

    # 创建 TEXT_VIEW_FACING 显示标签
    text = Marker()
    text.header = line.header
    text.ns = "semantic_labels"
    text.id = next_id()
    text.type = Marker.TEXT_VIEW_FACING
    text.text = f"{obj.display_label} ({obj.id})"
    text.pose.position.x = obj.center_x
    text.pose.position.y = obj.center_y
    text.pose.position.z = 0.35
    text.scale.z = 0.25
    text.color = ...  # 取自 rect.color_rgba
    markers.markers.append(text)

publisher.publish(markers)
注意：Marker.lifetime 可设置为 1 秒，这样一旦节点停止发布，旧标记会自动消失；但如果融合结果本身就是持续发布的，设置 lifetime 为 0（永久）也未尝不可。

6. 完整流程示意图
text
图像帧到达
    ↓
YOLO 检测 → [Detection2D, ...]
    ↓
深度图 + 内参 + 相机位姿
    ↓
SemanticProjector.project_detections() → [ProjectedObject, ...]
    ↓
SemanticObjectTracker.update() → [SemanticObject, ...] （稳定地图目标）
    ↓
object_to_rectangle() → Rectangle2D → MarkerArray → RViz 显示
7. 实际集成时的注意事项
时序对齐：同一帧的 RGB 图像和深度图必须时间戳匹配，否则投影位置会错位。

TF 变换获取：camera_to_map 需实时查询 TF 树（例如 lookup_transform("map", "camera_link", timestamp)）。

深度图格式：DepthImage 定义为 Sequence[Sequence[float|int|None]]，你可以从 ROS 的 sensor_msgs/Image 解码得到二维数组，单位要转换成米。

跟踪器配置：

association_distance_m 要根据物体距离和定位误差调整，通常 0.5~1.0 m 合适。

max_age_s 应大于物体可能被遮挡或检测漏检的最长时间，室内一般 10~30 秒。

类别先验：classes 字典必须为所有你关心的物体预先定义好 SemanticClass，包含 size_x、size_y 和 color_rgba；缺失的类别会在投影阶段被直接丢弃。