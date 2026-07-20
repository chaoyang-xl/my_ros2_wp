> 把二维检测结果与深度图结合，生成位于 `map/world` 坐标系下的三维对象点云，再把多帧中属于同一个物体的点云关联、融合并保存。

整体代码可以拆成五层：

```text
检测层
  ↓
单帧投影层
  ↓
坐标变换层
  ↓
跨帧追踪融合层
  ↓
保存与可视化层
```

---

# 一、整个项目的主流程

```text
RGB图像
  ↓
YOLO / YOLO-World检测
  ↓
类别、置信度、检测框xyxy、时间戳
  ↓
深度图时间匹配
  ↓
bbox投影 或 MobileSAM mask投影
  ↓
相机坐标系 XYZRGB 点云
  ↓
TF或Replica traj.txt
  ↓
map/world坐标系对象观测
  ↓
观测清理
  ↓
与已有track进行关联
  ↓
点云融合、类别更新、状态维护
  ↓
PLY + NPZ + semantic_objects.json
```

项目本身也将数据链路定义为：检测JSON和深度图生成相机系点云，再通过TF或相机位姿变换到地图系，随后执行对象关联和融合。

---

# 二、程序入口在哪里

ROS 2可执行节点注册在：

```text
setup.py
```

主要入口包括：

```text
offline_projector_node
sam_offline_projector_node
object_fusion_node
compressed_rgbd_decoder_node
odom_camera_tf_node
yolo_world_recorder_node
```

对应关系如下：

```text
ROS可执行程序                 Python文件

offline_projector_node
    → offline_projector_node.py

sam_offline_projector_node
    → sam_offline_projector_node.py

object_fusion_node
    → object_fusion_node.py

yolo_world_recorder_node
    → yolo_world_recorder_node.py

compressed_rgbd_decoder_node
    → compressed_rgbd_decoder_node.py

odom_camera_tf_node
    → odom_camera_tf_node.py
```

普通ROS投影Launch会同时启动：

```text
offline_projector_node
object_fusion_node
```

SAM版本则启动：

```text
sam_offline_projector_node
object_fusion_node
```

---

# 三、第一步：YOLO-World检测

主要文件：

```text
semantic_map_offline/yolo_world_recorder_node.py
semantic_map_offline/world_detection.py
```

核心类：

```python
YoloWorldRecorderNode
```

## 3.1 初始化阶段

`YoloWorldRecorderNode.__init__()` 主要完成：

```text
读取ROS参数
  ↓
加载类别词表
  ↓
过滤person、wall等排除类别
  ↓
加载YOLO-World模型
  ↓
加载CLIP文本模型
  ↓
创建RGB订阅器
  ↓
创建检测JSON和调试图发布器
  ↓
创建结果保存目录
```

节点订阅：

```text
/camera/color/image_raw
```

发布：

```text
/yolo/results_json
/yolo/debug_image
```

---

## 3.2 收到RGB图像后

回调函数：

```python
image_callback()
```

执行过程：

```text
收到RGB消息
  ↓
frame_count加1
  ↓
根据frame_skip判断是否跳过
  ↓
ROS Image转OpenCV BGR图
  ↓
model.predict()
  ↓
转换成统一检测JSON格式
  ↓
绘制检测框图
  ↓
保存JSON和JPG
  ↓
发布/yolo/results_json
  ↓
发布/yolo/debug_image
```

模型推理核心调用：

```python
results = self.model.predict(
    source=frame,
    imgsz=self.imgsz,
    conf=self.conf,
    iou=self.iou,
    device=self.device,
)
```

每个检测结果包含：

```json
{
  "class_id": 0,
  "class_name": "chair",
  "confidence": 0.83,
  "xyxy": [100, 120, 300, 420]
}
```

检测结果使用输入RGB消息的时间戳，这一点很重要，因为后面需要用它和深度图匹配。

---

## 3.3 检测结果保存

方法：

```python
_write_outputs()
```

会写入：

```text
output_dir/
├── latest.json
├── results.jsonl
├── latest.jpg
└── frames/*.jpg
```

其中：

* `latest.json`：最近一帧结果；
* `results.jsonl`：所有检测结果，每帧追加一行；
* `latest.jpg`：最近一帧检测框图；
* `frames/*.jpg`：按照 `save_every_n` 周期保存。

---

# 四、第二步：检测结果和深度图匹配

主要文件：

```text
semantic_map_offline/offline_projector_node.py
```

核心类：

```python
OfflineProjectorNode
```

这个节点负责：

```text
检测JSON
+
深度图
+
RGB图
+
TF
  ↓
当前帧map坐标系对象点云
```

---

## 4.1 支持两种检测输入模式

### 实时Topic模式

节点订阅：

```text
/yolo/results_json
```

检测结果存入：

```python
self._detections
```

### 离线JSONL模式

设置：

```bash
jsonl_path:=.../results.jsonl
```

节点启动时一次性加载JSONL，并按照时间戳排序。

只要 `jsonl_path` 非空，就不会再订阅 `/yolo/results_json`。

---

## 4.2 三种缓存

节点内部主要有：

```python
self._detections
self._depth_buffer
self._color_buffer
```

分别缓存：

```text
检测JSON
深度图
RGB图
```

因为三个消息不一定同时到达，所以不能收到一个消息就立刻直接使用，需要进行时间戳匹配。

---

## 4.3 深度回调

方法：

```python
_depth_cb()
```

主要逻辑：

```text
收到深度图
  ↓
存入depth_buffer
  ↓
检查SAM是否需要RGB
  ↓
检查processing_delay_frames
  ↓
选择延迟后的深度帧
  ↓
调用_process_depth()
```

`processing_delay_frames` 的作用不是改变数据时间戳，而是延迟处理，等待SLAM对应时刻的TF进入TF缓存。

---

# 五、第三步：处理一帧深度图

核心方法：

```python
_process_depth()
```

这是ROS投影链路中最重要的方法。

完整过程：

```text
1. 检查该深度帧是否已经处理
2. 查找时间最近的检测JSON
3. 判断时间差是否小于max_time_diff_s
4. 将ROS深度消息转成NumPy数组
5. 乘depth_scale转成米
6. 查询camera_frame → target_frame的TF
7. 准备RGB或SAM mask
8. 遍历每一个检测结果
9. 置信度与类别过滤
10. 将检测区域投影成相机系3D点
11. 用TF转换到map坐标系
12. 单帧体素和空间簇清理
13. 为每个点附加RGB、类别、检测ID和置信度
14. 发布PointCloud2
15. 发布检测元数据
16. 可选保存逐帧NPZ和JSON
```

节点声明了检测、深度、相机、TF、置信度和单帧清理相关参数。

---

# 六、第四步：检测框内深度投影

主要文件：

```text
semantic_map_offline/bbox_projection.py
```

核心方法：

```python
project_bbox_depth()
```

输入：

```text
深度图depth_m
检测框xyxy
检测图像尺寸
相机内参
pixel_stride
有效深度范围
```

输出：

```python
ProjectedBox
```

其中保存：

```text
points_camera    相机坐标系3D点
depth_uv         深度图上的像素坐标
image_uv         RGB图上的像素坐标
depth_bbox       映射到深度图后的检测框
```

---

## 6.1 检测框映射到深度图

YOLO框位于RGB图像坐标系，但深度图尺寸可能不同，因此先执行：

```python
scale_and_clip_bbox()
```

按比例计算：

```text
depth_x = rgb_x × depth_width / rgb_width
depth_y = rgb_y × depth_height / rgb_height
```

然后将检测框裁剪到深度图有效范围内。

需要注意：

> 这种方式只能解决分辨率不同的问题。如果RGB和深度来自不同相机且没有完成注册，单纯缩放无法补偿外参和视场差异。

---

## 6.2 检测框内取深度点

代码使用：

```python
np.arange(left, right, pixel_stride)
np.arange(top, bottom, pixel_stride)
np.meshgrid()
```

生成检测框内部采样像素。

例如：

```text
pixel_stride=1：每个像素都取
pixel_stride=2：每隔2个像素取
pixel_stride=4：每隔4个像素取
```

然后过滤：

```text
NaN
Inf
深度为0
小于min_depth_m
大于max_depth_m
```

---

## 6.3 像素反投影为三维点

使用标准针孔相机模型：

```text
X = (u - cx) × Z / fx
Y = (v - cy) × Z / fy
Z = depth
```

代码对应：

```python
x_valid = (u_valid - intrinsics.cx) * z_valid / intrinsics.fx
y_valid = (v_valid - intrinsics.cy) * z_valid / intrinsics.fy
```

最终生成：

```python
points_camera = [X, Y, Z]
```

这些点此时还在**相机光学坐标系**中。

---

# 七、SAM版本和普通版本的区别

SAM节点：

```text
sam_offline_projector_node.py
```

核心类：

```python
SamOfflineProjectorNode(OfflineProjectorNode)
```

它继承普通投影节点，因此：

```text
时间匹配
TF查询
置信度过滤
点云发布
元数据发布
逐帧保存
```

全部复用普通投影节点。

它只重写了两个关键步骤：

```python
_prepare_projection_frame()
_project_detection()
```

---

## 7.1 生成SAM mask

`_prepare_projection_frame()` 中：

```text
读取同步RGB
  ↓
将YOLO检测框缩放到RGB图尺寸
  ↓
sam_predictor.set_image(rgb)
  ↓
用检测框作为SAM提示
  ↓
生成每个检测框对应的mask
  ↓
可选进行mask腐蚀
```

SAM使用：

```python
multimask_output=False
```

所以每个检测框只生成一个最终mask。

---

## 7.2 mask腐蚀

当：

```text
mask_erode_px > 0
```

代码使用OpenCV腐蚀：

```python
cv2.erode()
```

目的是去掉mask边缘，降低物体边缘背景进入点云的概率。

---

## 7.3 mask投影

普通版本：

```text
投影YOLO矩形框中的有效深度点
```

SAM版本：

```text
只投影mask=true位置中的有效深度点
```

因此SAM通常能减少：

```text
物体后方墙面
地板
检测框角落背景
相邻物体
```

SAM最终仍生成相机坐标系点云，然后走和普通版本完全相同的TF、关联和融合链路。

---

# 八、第五步：相机坐标系变到地图坐标系

ROS流程使用TF：

```text
camera_frame → target_frame
```

通常为：

```text
camera_color_optical_frame → map
```

核心函数：

```python
transform_points()
```

计算：

```text
P_map = R × P_camera + t
```

代码将TF中的四元数转换为旋转矩阵，再给每个点应用旋转和平移。

转换后得到：

```text
points_map
```

后面的对象关联必须在同一个固定坐标系中进行，否则不同帧看到的同一个物体无法对齐。

---

# 九、投影节点发布了什么

投影节点将所有检测对象拼成一个 `PointCloud2`。

每个点包含：

```text
x
y
z
rgb
class_id
detection_id
confidence
image_u
image_v
```

其中最关键的是：

```text
detection_id
```

因为一帧中多个物体的点被合并进一个PointCloud2，融合节点需要依靠 `detection_id` 再把它们拆开。

同时还发布：

```text
/semantic_offline/detections
```

该JSON中保存：

```text
detection_id
class_id
class_name
confidence
xyxy
point_count
投影模式
时间差
```

---

# 十、第六步：融合节点同步点云和元数据

主要文件：

```text
semantic_map_offline/object_fusion_node.py
```

核心类：

```python
ObjectFusionNode
```

它订阅：

```text
/semantic_offline/points
/semantic_offline/detections
```

这两个消息来自同一帧，但到达顺序不确定，因此节点建立两个字典：

```python
self._cloud_buffer
self._metadata_buffer
```

键都是完整ROS时间戳：

```python
(sec, nanosec)
```

只有相同时间戳的点云和元数据都到齐，才调用：

```python
_try_process()
```

---

## 10.1 将组合点云重新拆成对象

融合节点读取PointCloud2后，根据：

```python
np.unique(cloud_arrays["detection_id"])
```

找到当前帧的所有检测对象。

然后对每个 `detection_id` 建立：

```python
ObjectObservation
```

包含：

```text
detection_id
class_id
class_name
confidence
stamp
points
colors
fuse_geometry
```

最后形成：

```python
observations = [
    ObjectObservation(...),
    ObjectObservation(...),
]
```

然后调用：

```python
self._fusion.update(observations)
```

---

# 十一、第七步：ObjectTracker进行对象关联

真正的追踪融合算法在：

```text
semantic_map_offline/object_tracker.py
```

核心类：

```python
ObjectTracker
```

每次调用：

```python
update(observations)
```

完整流程是：

```text
当前帧ObjectObservation列表
  ↓
已有track的missed_frames全部+1
  ↓
清理当前帧观测
  ↓
计算观测与已有track的关联分数
  ↓
全局一对一匹配
  ↓
匹配成功：融合到已有track
  ↓
匹配失败：创建新track
  ↓
删除失效candidate
  ↓
周期性合并重复track
  ↓
周期性去噪
```

---

# 十二、观测清理

方法：

```python
_clean_observation()
```

执行：

```text
原始对象点云
  ↓
voxel_downsample_with_colors()
  ↓
体素降采样
  ↓
largest_spatial_cluster_indices()
  ↓
只保留最大空间簇
  ↓
点数不足3则丢弃
```

这是为了减少：

```text
重复点
离散背景点
错误深度点
检测框内远处墙面形成的小簇
```

---

# 十三、对象关联如何判断

方法：

```python
_assign()
```

对于每一个当前帧观测和每一个已有track，按顺序检查五道门。

## 13.1 质心距离

```text
|| observation.centroid - track.centroid ||
    <= max_centroid_distance_m
```

距离太远，直接不可能是同一个对象。

---

## 13.2 AABB重叠

计算两个点云的三维轴对齐包围框重叠。

当前项目计算的是：

```text
交集体积 / 两个AABB中较小者的体积
```

它不是标准3D IoU。

低于：

```text
min_bbox_overlap
```

则拒绝关联。

---

## 13.3 几何重叠率

函数：

```python
nearest_neighbor_overlap()
```

对于点云A中的每个点，在点云B中找最近邻；距离小于 `overlap_radius` 就认为该点被覆盖。

会计算：

```text
A被B覆盖的比例
B被A覆盖的比例
```

最终取两者中较大的值。

这有利于处理：

```text
当前帧只看到物体局部
历史track已经积累了完整物体
```

当前帧虽然只覆盖历史模型一小部分，但历史模型可能很好地覆盖当前观测。

---

## 13.4 语义相似度

track内部维护：

```python
semantic_scores
```

例如一个track历史观测为：

```text
chair: 4.8
couch: 0.7
```

当前观测是 `chair`，语义相似度就会较高。

它不是简单要求类别完全一致，而是使用置信度加权的历史类别分布。

---

## 13.5 综合分数

```text
score =
geometry_weight × geometric_overlap
+
semantic_weight × semantic_similarity
```

默认：

```text
geometry_weight = 0.7
semantic_weight = 0.3
```

低于：

```text
association_threshold
```

则不关联。

---

## 13.6 尺寸增长限制

即使距离、重叠和语义都通过，还需要检查：

```python
_extent_growth_is_valid()
```

防止错误关联后，融合对象的长宽高突然增大。

例如椅子点云错误吸入墙面后，某个轴的尺寸可能从：

```text
0.6 m → 4.0 m
```

这时就会被 `max_extent_growth` 拒绝。

以上门限通过后，才会进入全局分配矩阵。

---

# 十四、为什么使用匈牙利分配

代码优先使用：

```python
scipy.optimize.linear_sum_assignment()
```

解决的是当前帧观测和历史track之间的**全局一对一匹配**。

例如：

```text
观测A与track1分数0.8
观测A与track2分数0.7
观测B与track1分数0.75
观测B与track2分数0.2
```

不能简单让每个观测各自选择最高分，否则A和B可能同时选择track1。

匈牙利算法会在整张分数矩阵上寻找总体最优的一对一组合。

如果没有SciPy，则退化为按分数从高到低的贪心匹配。

---

# 十五、匹配成功后如何融合

方法：

```python
_merge_observation()
```

更新内容包括：

```text
平均置信度
观测次数
最后观测时间
missed_frames归零
类别历史分数
主类别
点云
颜色
candidate/confirmed状态
```

点云融合方式：

```text
历史点云
+
当前帧点云
  ↓
拼接
  ↓
voxel_downsample_with_colors()
  ↓
新的对象模型
```

也就是说，系统不会无限保存所有重复点，而是在每次融合后进行体素化。

---

# 十六、匹配失败后发生什么

当前观测没有匹配任何历史track时：

```python
_create_track()
```

创建新对象：

```text
track_id自动递增
observation_count=1
status=candidate
points=当前观测点云
semantic_scores=当前类别置信度
```

状态变化：

```text
观测次数 < min_confirmed_observations
    → candidate

观测次数 >= min_confirmed_observations
    → confirmed
```

只有 `confirmed` 对象才会进入最终导航语义地图。

---

# 十七、candidate和confirmed的作用

假设：

```text
min_confirmed_observations = 3
```

状态变化为：

```text
第1次看到 → candidate
第2次看到 → candidate
第3次看到 → confirmed
```

这样可以过滤：

```text
单帧误检
偶然噪声
短暂错误投影
```

如果candidate长时间没有再次匹配，并超过：

```text
candidate_max_missed_frames
```

就会删除。

---

# 十八、重复track合并

每隔：

```text
map_merge_interval
```

调用：

```python
_merge_duplicate_tracks()
```

检查已有track之间是否重复。

合并条件主要是：

```text
类别相同
质心距离足够近
几何重叠率 >= map_merge_overlap
```

满足后使用并查集把多个track合并为一个，通常保留较小的track ID。

这一步主要解决：

```text
同一个对象早期被拆成两个track
机器人从不同视角观察导致临时碎片化
```

---

# 十九、周期去噪

每隔：

```text
denoise_interval
```

调用：

```python
_denoise_tracks()
```

对每个track再次执行最大空间簇过滤。

主要用于清理随着多帧融合逐渐积累的离散噪声。

---

# 二十、融合节点最终发布和保存什么

ROS实时发布：

```text
/semantic_offline/fused_points
/semantic_offline/objects
/semantic_offline/object_markers
```

磁盘输出：

```text
output_directory/
└── objects/
    ├── object_0001_chair.ply
    ├── object_0001_chair.npz
    ├── object_0002_table.ply
    └── object_0002_table.npz

snapshot_path
└── semantic_objects.json
```

主要内容：

```text
PLY
  对象XYZRGB点云

NPZ
  points_map
  rgb
  track_id
  class_id
  class_name
  confidence
  observation_count
  first_seen
  last_seen
  status

semantic_objects.json
  类别
  位置
  三维边界
  观测统计
  PLY和NPZ路径
```

项目输出结构和各文件内容在README中有明确说明。

---

# 二十一、Replica流程有什么不同

Replica不使用ROS topic和TF，而是直接读取：

```text
cam_params.json
traj.txt
frameXXXXXX.jpg
depthXXXXXX.png
```

入口：

```text
scripts/evaluate_projection_roundtrip.py
```

SAM入口：

```text
scripts/evaluate_sam_projection_tracking.py
```

真正主流程在：

```text
semantic_map_offline/tracking_pipeline.py
```

Replica流程：

```text
读取相机内参
  ↓
读取traj.txt相机位姿
  ↓
逐帧读取RGB和深度
  ↓
YOLO检测
  ↓
bbox或SAM mask投影
  ↓
相机坐标系点云
  ↓
使用traj.txt转换到world坐标系
  ↓
ObjectTracker.update()
  ↓
处理完全部帧
  ↓
fusion.finalize()
  ↓
统一保存对象文件
```

Replica和ROS最终共用同一个：

```python
ObjectTracker
```

所以两条链路的关联算法是一致的，区别只在于数据接入和坐标变换来源。Replica使用 `traj.txt`，ROS使用TF。

---

# 二十二、单帧闭环验证脚本不做融合

文件：

```text
scripts/evaluate_single_frame_roundtrip.py
```

它的流程是：

```text
2D检测框
  ↓
深度投影到3D
  ↓
相机位姿变换到world
  ↓
再从world反变换回相机
  ↓
投影回2D图像
  ↓
计算像素误差和bbox IoU
```

它只验证单帧投影数学和位姿是否正确，不调用 `ObjectTracker`，也不进行跨帧融合。

---

# 二十三、辅助节点

## `compressed_rgbd_decoder_node.py`

用于Meeting Room这类压缩bag：

```text
压缩RGB → 普通sensor_msgs/Image
压缩深度 → 16UC1深度图
```

## `odom_camera_tf_node.py`

用于补充：

```text
odom → lidar_link 动态TF
lidar_link → camera_frame 静态TF
```

主要服务于缺少完整TF树的bag。

---

# 二十四、建议的代码阅读顺序

建议按这个顺序看：

```text
1. README.md
   先理解项目目标和总链路

2. scripts/evaluate_projection_roundtrip.py
   看Replica入口，非常薄

3. semantic_map_offline/tracking_pipeline.py
   看一条完整的非ROS数据链路

4. semantic_map_offline/bbox_projection.py
   理解2D框怎么变成3D点

5. semantic_map_offline/object_tracker.py
   理解关联和融合算法

6. semantic_map_offline/offline_projector_node.py
   理解算法如何接入ROS

7. semantic_map_offline/object_fusion_node.py
   理解ROS消息如何转换成ObjectObservation

8. semantic_map_offline/sam_offline_projector_node.py
   最后看SAM相对普通投影增加了什么

9. launch/*.launch.py
   理解不同bag如何组合这些节点
```

最关键的三个文件是：

```text
bbox_projection.py
object_tracker.py
offline_projector_node.py
```

可以把它们分别理解为：

```text
bbox_projection.py
    负责：一个二维框如何生成三维点

offline_projector_node.py
    负责：一帧ROS数据如何完成同步、TF和发布

object_tracker.py
    负责：多帧三维对象如何判断是同一个物体并融合
```

当前项目最大的工程依赖是：**RGB-depth注册、时间同步和TF/SLAM质量必须先正确**。如果单帧点云已经错位，调整关联门限通常无法解决；如果单帧正确但最终对象碎片化或错误合并，再检查 `ObjectTracker` 的关联参数。项目README也明确指出，位姿漂移会造成点云重影和对象碎片化，而当前语义关联不包含视觉embedding。
