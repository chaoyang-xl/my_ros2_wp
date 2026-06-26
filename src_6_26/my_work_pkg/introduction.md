#### 1、项目背景与目标

智慧康养移位机器人需要在室内护理、床椅对接、人员辅助转移等场景中理解周围环境与人体状态。视觉语义感知模块承担从相机图像中提取“人、家具、人体姿态”等语义信息的任务，并将关键语义结果转换到机器人导航地图坐标系中，为导航、避障、对接和上层任务决策提供结构化输入。系统环境以当前 Orange Pi 5 Plus +
Orbbec GEMINI 336L + ROS 2 + RKNN YOLO/YOLO-pose 环境为基础。



#### 2、总体架构设计

| **层级** | **主要功能**                                                 | **推荐 ROS 节点**                           |
| -------- | ------------------------------------------------------------ | ------------------------------------------- |
| 采集层   | 前置/后置相机  RGB-D 图像采集、时间戳同步、CameraInfo 发布   | orbbec_camera  或相机驱动节点               |
| 推理层   | NPU  加载 RKNN 模型，完成 YOLO 检测与 YOLO-pose 姿态估计     | front_vision_node、rear_pose_alignment_node |
| 语义层   | 目标类别过滤、姿态状态识别、目标跟踪、置信度平滑             | semantic_extractor                          |
| 投影层   | 2D 语义 + Depth + TF 转换为 map 坐标语义目标                 | semantic_projection_node                    |
| 地图层   | 将语义目标写入导航地图/代价地图，可按类别设置生命周期与代价值 | semantic_costmap_layer                      |
| 调试层   | 生成  debug 图像、JSON、Marker、日志和网页预览               | vision_debug_server  / recorder             |

#### 3、上游功能及数据

| **功能**     | **说明**                                                     | **输出示例**                                             |
| ------------ | ------------------------------------------------------------ | -------------------------------------------------------- |
| 家具识别     | 识别床、椅子、沙发、桌子、柜体、马桶等对护理场景有意义的家具；初版可基于 COCO 类别，后续补充护理场景数据微调。 | chair、couch、bed、dining  table、toilet、cabinet/custom |
| 人体检测     | 检测前方人员。                                               | person  bbox + map position                              |
| 人体姿态估计 | 输出人体关键点，辅助判断姿态语义。                           | keypoints[17]  + pose_state                              |



#### 4、订阅话题

| **话题**                                                  | **类型**                                            | **用途**       | **备注**                                                     |
| --------------------------------------------------------- | --------------------------------------------------- | -------------- | ------------------------------------------------------------ |
| /camera/color/image_raw  或 /front_camera/color/image_raw | sensor_msgs/msg/Image                               | 前置  RGB 输入 | 当前环境已有  /camera/color/image_raw，可后续重命名为 front 命名空间。 |
| /camera/depth/image_raw  或 /front_camera/depth/image_raw | sensor_msgs/msg/Image                               | 前置深度输入   | 用于  2D 结果反投影。                                        |
| /front_camera/color/camera_info                           | sensor_msgs/msg/CameraInfo                          | 前置相机内参   | 必须保证与图像分辨率一致。                                   |
| /rear_camera/color/image_raw                              | sensor_msgs/msg/Image                               | 后置  RGB 输入 | 用于对接阶段人体姿态估计。                                   |
| /rear_camera/depth/image_raw                              | sensor_msgs/msg/Image                               | 后置深度输入   | 用于人体相对位置估计。                                       |
| /tf、/tf_static                                           | tf2_msgs/msg/TFMessage                              | 坐标变换       | camera_link、base_link、map  必须连通。                      |
| /map、/odom                                               | nav_msgs/msg/OccupancyGrid  / nav_msgs/msg/Odometry | 地图和定位参考 | 用于语义地图融合和调试。                                     |

#### 5、前置处理流程

·      订阅前置 RGB 图像、深度图、CameraInfo 和 TF；采用ApproximateTime 同步 RGB-D。

·      对 RGB 图像进行 letterbox、归一化和 RKNN 输入预处理，送入YOLO11n/YOLO11n-pose。

·       对检测结果执行 NMS、置信度过滤、类别过滤和小目标过滤。

·       在检测框内提取深度 ROI，使用中值深度或分位数深度减少噪声影响。

·       将目标底部中心点、中心点或关键点反投影到三维坐标，并转换到 map 坐标系。

·       对连续帧语义目标进行 ID 关联与时序滤波，减少跳变。

·       发布语义对象、调试图像、JSON 和地图层更新。



#### 6. 语义信息反向投影到导航地图

语义反投影的目标是把图像中的二维语义结果转换为导航可使用的地图坐标信息。前置摄像头输出的家具、人体需要被转换为 map 坐标系下的点、区域或代价层，必要时再通过 TF 转到 map。

##### 6.1 坐标变换链路

像素坐标 (u, v) + 深度 z
  → 相机坐标 camera_link:  X=(u-cx)·z/fx, Y=(v-cy)·z/fy, Z=z
  → 机器人坐标 base_link: T_base_camera ·P_camera
  → 地图坐标 map: T_map_base · P_base
  → 语义地图: semantic object / semantic grid/ costmap layer

| **步骤**   | **输入**                               | **输出**               |
| ---------- | -------------------------------------- | ---------------------- |
| 像素选点   | bbox、keypoints、mask/ROI              | 代表性像素点或像素区域 |
| 深度估计   | Depth  image 或 PointCloud2            | 目标深度  z 和置信度   |
| 三维反投影 | u、v、z、相机内参                      | P_camera               |
| TF  转换   | camera_link、base_link、map  外参和 TF | P_base  / P_map        |
| 地图融合   | 语义类别、位置、尺寸、置信度、生命周期 | semantic_costmap_layer |

 

##### 6.2 语义地图更新策略

| **语义类型** | **地图表示**                                 | **生命周期** | **导航策略**                             |
| ------------ | -------------------------------------------- | ------------ | ---------------------------------------- |
| 静态家具     | 语义点 + 语义区域/多边形；可写入长期语义地图 | 长           | 作为高代价区域，也可作为任务目标参考。   |
| 人体         | 动态目标点                                   | 短           | 提高代价、保持安全距离、必要时触发暂停。 |

 

##### 6.3 与导航模块的推荐集成方式

·       长期语义：将稳定家具写入 semantic_map，供任务规划或人机交互查询，不直接覆盖 SLAM 原始地图。

·       动态人体：作为高代价动态障碍，设置比普通障碍更大的安全膨胀半径。

·       地图更新应可开关：调试阶段允许只发布Marker/JSON，不写入导航 costmap；验收阶段再接入导航闭环。