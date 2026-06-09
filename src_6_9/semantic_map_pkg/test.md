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

#### 5、教师反馈后的需求澄清：Cartographer 二维语义地图

老师提供的 Cartographer 激光雷达建图结果本质上是二维占据栅格地图，当前语义地图模块的阶段目标不是重新建图，而是在已有 `/map` 坐标系上叠加由视觉识别得到的语义对象框。系统需要把 YOLO/YOLO-pose 或后续自定义检测模型识别出的家具、卫浴设施、人员等目标，通过 RGB-D 深度和 TF 坐标变换映射到 Cartographer 地图中，并在地图上形成带类别标签的二维矩形区域，例如“沙发”“茶几”“马桶”“床”“椅子”等。

该二维语义地图应服务于视觉语言导航：当上层任务接收到“去卫生间”等自然语言指令时，可以先解析出目标语义或关联物体（例如卫生间 → 马桶），再在语义地图中查询对应目标的 map 坐标和可到达导航点，最后交给 Nav2 或现有导航模块执行到点导航。

#### 6、语义地图模块的输入、输出与数据结构

| **数据方向** | **名称/话题** | **类型建议** | **说明** |
| ------------ | ------------- | ------------ | -------- |
| 输入 | `/map` | `nav_msgs/msg/OccupancyGrid` | Cartographer 输出的二维占据栅格地图，作为语义叠加底图。 |
| 输入 | `/tf`、`/tf_static` | `tf2_msgs/msg/TFMessage` | 保证 `camera_link`、`base_link`、`map` 坐标系连通。 |
| 输入 | `/semantic/detections` | 自定义 `SemanticDetectionArray` 或 `vision_msgs/msg/Detection2DArray` | 上游视觉节点输出的类别、置信度、2D bbox、深度统计值。 |
| 输出 | `/semantic_map/objects` | 自定义 `SemanticObjectArray` | 已投影到 map 坐标系的语义对象列表。 |
| 输出 | `/semantic_map/markers` | `visualization_msgs/msg/MarkerArray` | RViz 中显示矩形框、类别文字、目标中心点。 |
| 输出 | `/semantic_map/grid`（可选） | `nav_msgs/msg/OccupancyGrid` | 将语义对象栅格化后的语义层，便于调试或叠加代价地图。 |
| 输出 | `/semantic_map/query`（服务） | 自定义 srv | 根据类别或自然语言解析结果查询目标位置和导航候选点。 |

建议的单个语义对象字段如下：

| **字段** | **类型** | **说明** |
| -------- | -------- | -------- |
| `id` | `string` | 跟踪后的稳定对象 ID。 |
| `label` | `string` | 语义类别，例如 `sofa`、`tea_table`、`toilet`。 |
| `confidence` | `float32` | 多帧融合后的置信度。 |
| `center_map` | `geometry_msgs/msg/Point` | 对象中心点在 `map` 坐标系下的位置。 |
| `yaw` | `float32` | 矩形框在地图中的朝向；初版可使用 0 或由点云/多帧估计。 |
| `size_x` / `size_y` | `float32` | 语义矩形框在地图中的长宽，单位米。 |
| `last_seen` | `builtin_interfaces/msg/Time` | 最近一次观测时间，用于生命周期管理。 |
| `source_frame` | `string` | 观测来源相机坐标系。 |

#### 7、从视觉检测到二维地图矩形框的处理流程

1. **目标检测与类别过滤**：前置相机节点运行 YOLO/RKNN 模型，保留护理和室内导航相关类别，例如 `person`、`chair`、`couch/sofa`、`bed`、`dining table/tea table`、`toilet`、`cabinet`。
2. **深度反投影**：对检测框中心区域或目标 mask 内的深度取中位数，结合 CameraInfo 内参将像素点反投影到 `camera_link` 三维坐标。
3. **坐标变换**：通过 TF 将目标点从 `camera_link` 转换到 `map` 坐标系，得到目标中心在 Cartographer 地图中的位置。
4. **矩形框估计**：初版根据类别预设典型尺寸生成二维矩形框，例如马桶约 `0.8m × 0.8m`、茶几约 `1.0m × 0.6m`、沙发约 `1.8m × 0.9m`；后续可结合深度点云外接矩形或多帧观测更新尺寸。
5. **多帧融合与去抖**：同类别且中心距离小于阈值的观测合并为同一对象，采用滑动平均或卡尔曼滤波平滑中心点，避免语义框跳动。
6. **地图叠加显示**：在 RViz 中用 `MarkerArray` 画出二维矩形框和文字标签，同时可生成调试 JSON，记录每个对象的类别、坐标、尺寸和更新时间。
7. **导航目标生成**：语义查询服务根据对象类别返回对象中心点附近的可达候选点，候选点需要避开障碍栅格并满足机器人 footprint 半径。

#### 8、第一阶段交付范围

| **优先级** | **任务** | **验收标准** |
| ---------- | -------- | ------------ |
| P0 | 读取 Cartographer `/map` 并在 RViz 中显示语义对象框 | 给定一组模拟检测结果，能在地图对应位置画出带标签的矩形框。 |
| P0 | 完成 RGB-D 检测框到 `map` 坐标的投影链路 | 输入 2D bbox + depth + CameraInfo + TF 后，输出稳定的 `center_map`。 |
| P0 | 支持沙发、茶几、马桶、床、椅子等室内目标类别 | 每个类别有默认尺寸、显示颜色和中文/英文标签映射。 |
| P1 | 多帧融合、对象 ID 和生命周期管理 | 同一物体不会在地图上重复生成大量框，长时间未观测对象可降置信或过期。 |
| P1 | 语义查询接口 | 输入 `toilet` 或“卫生间”相关语义，返回马桶目标及可导航候选点。 |
| P2 | 结合点云估计物体真实外接矩形 | 语义框尺寸不再只依赖类别先验。 |

#### 9、当前待确认问题

1. 老师所说的“Uo 识别”是否指 YOLO 识别；如果不是，需要明确具体模型或算法名称。
2. Cartographer 地图的分辨率、原点、坐标系命名是否与当前 ROS 2 TF 树一致。
3. 是否已有前置相机到 `base_link` 的准确外参标定；如果没有，语义框映射到地图上会有明显偏移。
4. 初版语义矩形框是否只需要 RViz 可视化，还是需要真正写入 Nav2 costmap 参与规划。
5. “去卫生间”等语言指令由哪个上层模块解析；语义地图模块初版可先提供按类别查询接口。
