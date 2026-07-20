# my_work_pkg — 语义地图投影系统

> **一句话概括**：让机器人"看到"的东西（摄像头检测）和"摸到"的东西（激光雷达/深度相机地图）自动对齐融合，形成带语义标签的环境地图。

---

## 目录

- [1. 前置知识：机器人是怎么"看世界"的](#1-前置知识机器人是怎么看世界的)
- [2. 系统全景：解决什么问题](#2-系统全景解决什么问题)
- [3. 架构总览：数据从哪里来，到哪里去](#3-架构总览数据从哪里来到哪里去)
- [4. 模块详解](#4-模块详解)
  - [4.1 投影引擎 (yolo_seed_projection)](#41-投影引擎-yolo_seed_projection)
  - [4.2 语义投影与记忆 (semantic_projection)](#42-语义投影与记忆-semantic_projection)
  - [4.3 节点一：标准 YOLO 接入 (yolo_seed_projection_node)](#43-节点一标准-yolo-接入-yolo_seed_projection_node)
  - [4.4 节点二：前端桥接 (frontend_bridge_node)](#44-节点二前端桥接-frontend_bridge_node)
  - [4.5 节点三：语义吸附与追踪 (semantic_projection_node)](#45-节点三语义吸附与追踪-semantic_projection_node)
  - [4.6 节点四：回环修正 (loop_closure_guard_node)](#46-节点四回环修正-loop_closure_guard_node)
- [5. 数据格式速查](#5-数据格式速查)
- [6. 关键参数对照表](#6-关键参数对照表)
- [7. 运行指南](#7-运行指南)
- [8. 测试](#8-测试)
- [9. 故障排查](#9-故障排查)

---

## 1. 前置知识：机器人是怎么"看世界"的

如果你是第一次接触机器人，以下概念可以帮助你理解代码为什么这样写：

### 1.1 三只"眼睛"：彩色相机、深度相机、激光雷达

| 传感器 | 它看到什么 | 输出形式 | 局限 |
|--------|-----------|---------|------|
| 彩色相机 (RGB Camera) | 彩色照片，每个像素是颜色值 (R,G,B) | 图像 640×480 或 1280×720 | 没有深度信息，不知道物体离多远 |
| 深度相机 (Depth Camera) | 每个像素不是颜色，而是距离 (单位：米或毫米) | 图像，每个像素值就是距离 | 有噪声，远距离不准确 |
| 激光雷达 (LiDAR) | 360° 一圈的距离值（像一把尺子转一圈） | 一维数组，每个角度对应一个距离 | 没有颜色，不知道看到的是什么物体 |

**本系统用到的**：彩色相机（做物体识别）+ 深度相机（获取距离）+ 激光雷达（建占用栅格地图）。

### 1.2 坐标系：同一个点，不同的"视角"

想象你在房间里，你的手机 GPS 显示你在 `(120.1, 30.2)`，而你眼中的桌子在"前方两米"。

机器人面对同样的问题——同一个物体、同一个位置，在不同参照系下有不同坐标：

```
                            map 坐标系
                         (全局地图，固定不动)
                               │
                               │  ← tf 变换
                               ▼
                         odom 坐标系
                        (里程计，告诉机器人走了多远)
                               │
                               │  ← tf 变换
                               ▼
                ┌──────────────┴──────────────┐
                │       base_link 坐标系        │
                │   (机器人底盘，跟着它一起动)    │
                │                              │
                │   ┌──────────────────────┐   │
                │   │  camera_optical_frame │   │
                │   │  (相机光心，X→右,Y→下, │   │
                │   │   Z→前方)              │   │
                │   └──────────────────────┘   │
                └──────────────────────────────┘
```

- **map 坐标系**：全局地图坐标系，原点固定。建图、导航都用它。代码里的 `gx, gy` 就是指 map 下的坐标。
- **camera_optical_frame**：相机光学坐标系。YOLO 检测到的物体中心点最初就是相对这个坐标系的。
- **tf (transform)**：ROS 中的"定位转换服务"。你可以问它："`camera_optical_frame` 里的 (1, 0, 2) 这个点在 `map` 里是什么坐标？" 它算出答案后返回。

**为什么需要坐标变换？** 因为视觉检测点（相机系）和激光地图（map 系）如果不统一到同一个坐标系，就无法比较、无法融合。

### 1.3 占用栅格地图 (Occupancy Grid Map)

激光雷达扫描环境后，ROS 会构建一张"占用栅格地图"：

```
   ┌───┬───┬───┬───┬───┐
   │ 0 │ 0 │ -1│100│ 0 │   0   = 空闲（free）
   ├───┼───┼───┼───┼───┤  -1   = 未知（unknown）
   │ 0 │100│100│ 0 │ 0 │  100  = 被占用的（occupied，有障碍物）
   ├───┼───┼───┼───┼───┤
   │ 0 │100│100│ 0 │ 0 │
   ├───┼───┼───┼───┼───┤
   │ 0 │ 0 │ 0 │ 0 │ 0 │     ← 上面这些"100"连成一片，代表一个物体
   └───┴───┴───┴───┴───┘     （比如一把椅子、一个垃圾桶）
```

每个格子叫一个"cell"，值是它的"占用概率"（0=空的，100=有东西）。一片连成块的 `100` 代表一个物理物体。

---

## 2. 系统全景：解决什么问题

### 问题

摄像头（YOLO）能看到物体并告诉你"这是一个垃圾桶，在图像坐标 (200, 300)"。但：
- 图像坐标 (200, 300) 在真实世界的哪里？
- 激光雷达能看到障碍物轮廓但不能识别"这是什么"
- 同一物体被多次看到会产生多个原始观测，需要去重和追踪

### 我们的方案

用一根管道把视觉检测转化为地图上的语义标记：

```
摄像头图像 ──► [ YOLO 检测 ] ──► 2D 图像坐标
                                    │
深度图     ─────────────────────────┤  (深度反投影: 2D → 3D)
                                    │
tf 变换    ─────────────────────────┤  (相机系 → 地图系)
                                    │
                              /semantic_seed
                           (地图坐标系下的原始种子)
                                    │
占用栅格地图 ──► [ 语义吸附 ] ───────┤  (把种子"吸附"到最近的栅格物体)
                                    │
                          [ 记忆追踪 ]  (同一物体的多次观测→平滑合并)
                                    │
                              RViz 可视化
                          (地图上显示带标签的框)
```

### 效果

最终你可以在 RViz 的地图上看到：
- 一个绿色框标出 `chair_0 (88%) n=5`（椅子，置信度 88%，已被观测 5 次）
- 一个绿色框标出 `person_0 (93%) n=12`（人，带有姿态"站立"）

---

## 3. 架构总览：数据从哪里来到哪里去

```
┌─────────────────────────────────────────────────────────────┐
│                        数据源 (前端)                          │
│                                                             │
│  /frontend/detections   (String, 自定义 JSON)                │
│  ─OR─                                                       │
│  /yolo/detections       (vision_msgs/Detection2DArray)       │
│                                                             │
│  /camera/depth/image_rect_raw  (sensor_msgs/Image)           │
└─────────────────────┬───────────────────────────────────────┘
                      │
          ┌───────────┴───────────┐
          │                       │
          ▼                       ▼
┌──────────────────┐    ┌──────────────────────┐
│ frontend_bridge  │    │ yolo_seed_projection │
│     _node        │    │        _node          │
│                  │    │                       │
│ 解析自定义 JSON   │    │ 解析 ROS 标准消息      │
│ xyxy→中心点       │    │ Detection2DArray      │
│ IoU 姿态匹配      │    │                       │
│       │          │    │         │              │
│       └──────────┼────┼─────────┘              │
│                  │    │                        │
│  共用: 深度反投影 + tf 变换 → map 坐标           │
│  共用: 复用 YoloLidarSeedProjector              │
└────────┬─────────┘    └───────────┬────────────┘
         │                          │
         └──────────┬───────────────┘
                    │
                    ▼
         /semantic_seed  (std_msgs/String, JSON)
                    │
         ┌──────────┴──────────┐
         │                     │
         ▼                     ▼
  ┌──────────────┐    ┌────────────────┐    ┌────────────────────┐
  │  /map        │    │ semantic_      │    │ loop_closure_      │
  │ OccupancyGrid│◄───│ projection_node│    │ guard_node  🆕     │
  │ (激光雷达地图) │    │                │    │                    │
  └──────────────┘    │ 种子→吸附→岛屿  │    │ = semantic_proj... │
                      │ 记忆池追踪     │    │ + 回环检测         │
                      │ Marker 可视化  │    │ + 坐标自动修正      │
                      └───────┬────────┘    └─────────┬──────────┘
                              │                       │
                              └───────────┬───────────┘
                                          │
                                          ▼
                                 ~/markers  (MarkerArray)
                                          │
                                          ▼
                                     RViz 显示
```

---

## 4. 模块详解

### 文件名速查

```
my_work_pkg/
├── package.xml                        # ROS 包元数据（依赖声明）
├── setup.py                           # Python 包入口 & 节点注册
├── setup.cfg                          # Python 安装配置
├── readme.md                          # 测试流程备忘
│
├── deepseek_json_20260605_3313ab.json # 前端检测样例数据
├── deepseek_json_20260605_b5b105.json # 前端检测样例数据
│
├── my_work_pkg/
│   ├── __init__.py                    # 导出核心类
│   │
│   ├── yolo_seed_projection.py        # 🧮 深度反投影引擎（纯算法，无 ROS 依赖）
│   ├── yolo_seed_projection_node.py   # 🔵 标准 ROS YOLO 接入节点
│   ├── frontend_bridge_node.py        # 🟠 前端自定义 JSON 桥接节点
│   │
│   ├── semantic_projection.py         # 🧮 栅格吸附 + 记忆追踪引擎（纯算法）
│   ├── semantic_projection_node.py    # 🟢 语义吸附与可视化节点
│   ├── loop_closure_guard_node.py     # 🔴 带回环修正的语义吸附节点（替代前者）
│
├── test/
│   ├── test_yolo_seed_projection.py      # 投影引擎单元测试
│   └── test_single_point_projection.py   # 吸附引擎单元测试
│
└── resource/
    └── my_work_pkg                    # ROS 包标记文件
```

### 4.1 投影引擎 (`yolo_seed_projection.py`)

**职责**：把一个 2D 图像上的检测框中心 + 深度图像 → 计算出 3D 坐标。

**类比**：你知道照片中某个人位于画面的正中央，但你不知道他离你多远。如果同时有一张"深度图"告诉你画面上每个像素的距离，你就可以算出这个人在 3D 空间里的位置。

#### 核心数据类

##### `YoloDetection2D`
```python
YoloDetection2D(
    label="chair",         # 物体类别名称（英语名）
    confidence=0.95,       # 置信度 (0~1)
    center_u=320.0,        # 边界框中心 X (图像像素坐标)
    center_v=240.0,        # 边界框中心 Y (图像像素坐标)
    image_width=640,       # 图像宽度
    image_height=480,      # 图像高度
    track_id=None,         # 可选追踪 ID（前端一般不提供）
)
```

##### `CameraFrameSeed`
```python
CameraFrameSeed(
    label="chair",         # 物体类别
    confidence=0.95,       # 置信度
    x=0.5, y=-0.2, z=2.0, # 相机光心坐标系下的 3D 位置（米）
    track_id=None,
)
```

#### 核心算法 `YoloLidarSeedProjector.project()`

```
输入:  YoloDetection2D + depth_image (深度图 numpy 数组)
输出:  CameraFrameSeed 或 None（投影失败）

步骤:
  1. 取 (center_u, center_v) 周围 5×5 小窗口
  2. 过滤掉无效值 (NaN, inf, <0.3m, >5m)
  3. 取中位数深度 (比取单点更抗噪)
  4. 针孔相机模型反投影:

         z = 中位数深度
         x = (center_u - cx) * z / fx
         y = (center_v - cy) * z / fy

         其中 fx, fy, cx, cy 是相机内参
         (cx, cy) = 图像光心坐标
         fx, fy   = 焦距（像素单位）
```

**为什么用中位数而不是单点？** 因为深度相机有噪声，单点可能刚好落在物体的边缘或噪声上。取小窗口中位数更鲁棒。

---

### 4.2 语义投影与记忆 (`semantic_projection.py`)

这是系统的第二个核心引擎，负责"将 map 坐标下的种子精确吸附到栅格地图上"。

#### 核心数据类

| 类 | 用途 |
|----|------|
| `SemanticSeed` | 输入：地图坐标系下的粗糙种子 `(label, confidence, gx, gy)` |
| `OccupancyGridMap` | 输入：激光雷达建的占用栅格地图 |
| `OccupancyIsland` | 中间：BFS 洪水填充找到的一片连通占用区域 |
| `ProjectedSemanticObject` | 中间：吸附后精确定位的语义对象 |
| `TrackedObject` | 输出：经过记忆平滑和时间追踪的最终对象 |
| `SemanticMemory` | 记忆池管理器（匹配、平滑、遗忘） |

#### 核心算法 `SinglePointSemanticProjector.project()`

```
输入:  SemanticSeed (map 系下的粗估计) + OccupancyGridMap
输出:  ProjectedSemanticObject 或 None (被过滤掉)

步骤:
  1. 置信度检查: seed.confidence < 最低置信度 → 直接丢弃

  2. 搜索最近占用栅格:
     以 (gx, gy) 为中心, search_radius_m 半径内
     找到欧氏距离最近的 occupied>threshold 栅格

  3. BFS 洪水填充 (Flood Fill):
     从找到的占用栅格出发，4连通或8连通扩散
     把所有连在一起的 occupied 栅格收集起来 → "OccupancyIsland"
     (就像 Photoshop 的魔棒工具: 点击一点 → 选中整片连续区域)

  4. 几何验证:
     - 总像素数 < min_total_pixels → 拒绝（太小，可能是噪声）
     - 长宽比 > max_aspect_ratio → 拒绝（太细长，可能是墙壁）
     - 贴地图边界 (wall_margin_m) → 拒绝（可能是墙段残留）

  5. 测量岛屿几何:
     - 质心坐标 (所有的 x, y 平均) → 替代粗糙种子的位置
     - 主轴方向 (PCA) → 知道物体朝向
     - 包围盒尺寸 (size_x, size_y)
     - 长宽比

  6. 输出 ProjectedSemanticObject
```

#### 记忆池 `SemanticMemory`

```
update(snapped_object, current_time):
  1. 全局匹配: 在所有同名 (同 label) 的记忆对象中
     找欧氏距离最近且 < match_distance 的

  2. 如果找到匹配:
     EMA 指数加权平滑:
      新位置 = alpha × 新观测 + (1-alpha) × 旧位置
      新尺寸 = alpha × 新观测 + (1-alpha) × 旧尺寸
      置信度 = 历史平均值
      times_seen += 1

  3. 如果没找到匹配:
     创建新的 TrackedObject (ID: label_0, label_1, ...)

age(current_time):
  清理 last_seen 超过 timeout 秒的老对象
```

**EMA 方程示例** (alpha=0.3)：若旧位置是 (2.0, 3.0)，新观测是 (2.4, 3.2)，则更新后位置 = 0.3×(2.4, 3.2) + 0.7×(2.0, 3.0) = (2.12, 3.06)。越多次观测，位置越平滑稳定。

---

### 4.3 节点一：标准 YOLO 接入 (`yolo_seed_projection_node`)

**节点名**：`yolo_seed_projection_node`

**数据路径**：
```
/yolo/detections (Detection2DArray) ─┐
                                     ├─► [同步] ─► [投影] ─► [tf变换] ─► /semantic_seed
/camera/depth/image_rect_raw (Image)─┘
```

**适用场景**：当上游发布的是标准 ROS `vision_msgs/Detection2DArray` 消息时使用（比如直接用 ROS 包装的 YOLO 节点）。

**特殊处理**：
- 用 `message_filters.ApproximateTimeSynchronizer` 做时间同步（两张图到达时间可能在 0.1 秒内波动）
- z=0（平面投影，因为机器人在平面上移动，我们只关心 x, y 坐标）
- 输出的 JSON 只包含 `label, confidence, gx, gy, track_id`

---

### 4.4 节点二：前端桥接 (`frontend_bridge_node`)

**节点名**：`frontend_bridge_node`

**数据路径**：
```
/frontend/detections (String, 自定义JSON) ─┐
                                          ├─► [同步] ─► [投影] ─► [tf变换] ─► /semantic_seed
/camera/depth/image_rect_raw (Image)──────┘
```

**适用场景**：当上游是 Orangepi 等嵌入式设备，发布自定义 JSON 格式的检测数据时使用。

**关键差异 vs 标准节点**：

| 方面 | yolo_seed_projection_node | frontend_bridge_node |
|------|--------------------------|----------------------|
| 检测输入 | ROS `Detection2DArray` | 自定义 JSON `String` |
| 框格式 | 已给出中心点 | xyxy → 需要算中心 |
| 姿态/动作 | 无 | person 检测可附带 pose |
| 检测-姿态匹配 | 不需要 | IoU 贪婪匹配 |
| 输出 JSON | 基础字段 | 基础字段 + pose |

**姿态匹配 (IoU 贪婪)**：

```
    检测框 A (person, xyxy=[100,50,300,500])
    姿态框 P1 (person, xyxy=[105,48,295,510])
    姿态框 P2 (person, xyxy=[600,200,800,700])

    IoU(A, P1) = 0.92  ← 最高，匹配！
    IoU(A, P2) = 0.0

    结果: P1 姿态数据附加到 A 的输出中，P1 从匹配池中移除
```

**输出示例** (含姿态的人):
```json
{
  "label": "person",
  "confidence": 0.885,
  "gx": -2.62,
  "gy": 3.81,
  "class_id": 0,
  "pose": {
    "posture": "unknown",
    "posture_confidence": 0.2,
    "action_tags": [],
    "keypoints": [
      {"name": "nose", "x": 1280.0, "y": 181.5, "confidence": 0.037}
    ]
  }
}
```

**12 个可配置参数**：见 [第 6 节](#6-关键参数对照表)。

---

### 4.5 节点三：语义吸附与追踪 (`semantic_projection_node`)

**节点名**：`semantic_projection_node`

**数据路径**：
```
/semantic_seed (String) ──► [吸附到栅格] ──► [记忆池更新] ──► [定时发布 Marker]
       │                          ▲
/map (OccupancyGrid) ─────────────┘
```

**三个核心阶段**：

**阶段 1：吸附 (`_seed_cb`)**
1. 收到 `/semantic_seed` 上的种子 JSON
2. 用 `SinglePointSemanticProjector.project()` 找到最近占用岛屿
3. 如果找不到（种子落在空地 / 噪声被过滤），丢弃

**阶段 2：记忆池 (`memory.update()`)**
1. 同名对象欧氏距离匹配
2. EMA 平滑或新建 Tracker
3. 累计 `times_seen`

**阶段 3：可视化 (`_publish_memory_cb`, 每 0.5 秒)**
1. 老化清理（超时未观测的对象被删除）
2. 遍历活跃对象，为每个对象创建两个 RViz Marker：
   - **矩形框** (LINE_STRIP)：显示对象在地图上的位置和尺寸
   - **文本标签** (TEXT_VIEW_FACING)：显示 `object_id(置信度%) n=观测次数`

---

### 4.6 节点四：回环修正 (`loop_closure_guard_node`)

**节点名**：`loop_closure_guard_node`

**是什么**：`semantic_projection_node` 的增强版。在全部原有功能（种子吸附、记忆追踪、Marker 可视化）之上，增加了 **SLAM 回环检测的应对能力**。

**为什么需要**：Cartographer 检测到回环后会重新优化全局位姿图，此时 `map` 坐标系整体发生偏移。"旧 map"下记录的物体位置会在"新 map"中整体错位——对象框悬空出现在错误位置。

**数据路径**：
```
/semantic_seed ──┐
                 ├──► [吸附] ──► [记忆追踪] ──► [Marker 可视化] ──► ~/markers
/map ────────────┘         │
                            │
              ┌─────────────┘
              │  (每秒检查一次)
              ▼
     tf: odom → map
              │
              ├── 变化 < 阈值 (0.20m, 0.08rad) → 正常
              │
              └── 变化 > 阈值 → 回环！
                    │
                    ▼
              计算修正量 delta = T_new · T_old⁻¹
                    │
                    ▼
              对所有 TrackedObject 应用修正
              (位置 x,y 做 2D 刚体变换)
                    │
                    ├── source=snapped: 进入 pending_resnap
                    │        └── 等新 /map + TF 连续稳定后 re-snap
                    └── source=visual_only: 只保留几何修正
```

#### 回环修正原理

Cartographer 回环时会改变 `odom → map` 的 TF 变换。代码中通过
`lookup_transform("map", "odom", ...)` 读取该变换，即把 odom 坐标下的点转换到 map 坐标下的 `T_map_odom`。

```
回环前: T_old: odom → map_old    (记录的值)
回环后: T_new: odom → map_new    (当前值)

对于世界中的一个固定点 P：
  P_old_map = T_old(P_odom)
  P_new_map = T_new(P_odom)

先把旧 map 坐标转回 odom：
  P_odom = T_old⁻¹(P_old_map)

再投到新 map：
  P_new_map = T_new · T_old⁻¹ · P_old_map

其中 T_new · T_old⁻¹ 就是修正变换 (delta)
——对每个追踪对象的位置 (x, y) 应用这个 2D 刚体变换即可。

修正后还会按对象来源做不同处理：

- `source=snapped`：对象已经和 OccupancyGrid 结构绑定，先做几何修正并进入 `pending_resnap`；收到回环检测之后的新 `/map` 且 TF 连续稳定后，再以修正后的位置为种子进行小半径吸附，刷新对象中心与尺寸。
- `source=visual_only`：对象不依赖 OccupancyGrid，只保留几何修正结果，不重新吸附，避免动态/纯视觉目标被拉到墙体或静态障碍上。

re-snap 是地图维护步骤，不是一次新观测，因此不会增加 `times_seen`，也不会更新 `confidence` 或 `last_seen`。
```

**与 `semantic_projection_node` 的关系**：二选一运行，不要同时跑两个。

| 场景 | 用哪个 |
|------|--------|
| 不需要回环处理，轻量运行 | `semantic_projection_node` |
| 使用 Cartographer，需要回环容错 | `loop_closure_guard_node` |

#### 专属参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `lc_check_period` | 1.0 | TF 检查频率 (秒) |
| `lc_translation_threshold` | 0.20 | 位移跳变阈值 (米) — 超过即判定回环 |
| `lc_rotation_threshold` | 0.08 | 旋转跳变阈值 (弧度) — ≈4.6° |
| `lc_source_frame` | `odom` | 监控 `odom → map` 的变化 |
| `lc_resnap_enabled` | true | 回环后是否对 snapped 对象在新 `/map` 上重新小半径吸附 |
| `lc_resnap_radius_m` | 0.3 | snapped 对象首次 re-snap 搜索半径 (米) |
| `lc_resnap_max_attempts` | 4 | 失败对象跨新地图帧重试次数 |
| `lc_resnap_radius_step_m` | 0.15 | 每轮搜索半径增量 (米) |
| `lc_resnap_max_radius_m` | 0.6 | 搜索半径上限 (米) |
| `lc_stable_checks` | 2 | re-snap 前 TF 连续稳定检查次数 |
| `lc_stable_translation_threshold` | 0.02 | TF 稳定检查最大平移变化 (米) |
| `lc_stable_rotation_threshold` | 0.01 | TF 稳定检查最大旋转变化 (弧度) |

> 其他所有参数（`search_radius_m`、`match_distance`、`memory_timeout` 等）与 `semantic_projection_node` 完全相同，见 [第 6 节](#6-关键参数对照表)。

#### 运行时日志示例

```
[WARN] [LoopClosure] TF jump detected: dtrans=0.323m, drot=0.094rad.
       Old=(0.500,0.200,0.010) New=(0.623,0.177,-0.024)
[INFO] [LoopClosure] Applied correction to 5/7 objects
```

---

## 5. 数据格式速查

### 前端检测 JSON (topic: `/frontend/detections`)

```json
{
  "stamp_sec": 1780639825,
  "stamp_nanosec": 790170112,
  "wall_time": "2026-06-05T14:10:26.318",
  "image_topic": "/camera/color/image_raw",
  "image_shape": [720, 1280, 3],
  "detections": [
    {
      "class_id": 0,
      "class_name": "person",
      "confidence": 0.8857421875,
      "xyxy": [855.0, 2.75, 1279.0, 711.25]
    }
  ],
  "poses": [
    {
      "class_id": 0,
      "class_name": "person",
      "confidence": 0.93408203125,
      "xyxy": [856.25, 0.0, 1279.75, 709.75],
      "keypoints": [
        {"name": "nose", "x": 1280.0, "y": 181.5, "confidence": 0.037}
      ],
      "posture": "unknown",
      "posture_confidence": 0.2,
      "action_tags": [],
      "posture_features": {
        "bbox_aspect_w_div_h": 0.597,
        "valid_core_keypoints": 1
      },
      "posture_reasons": ["bbox_tall", "not_enough_core_keypoints"]
    }
  ]
}
```

### 语义种子 JSON (topic: `/semantic_seed`)

**最小格式** (所有节点都兼容):
```json
{
  "label": "chair",
  "confidence": 0.88,
  "gx": -2.62,
  "gy": 3.81
}
```

**完整格式** (frontend_bridge_node 输出的带姿态版本):
```json
{
  "label": "person",
  "confidence": 0.885,
  "gx": -2.62,
  "gy": 3.81,
  "class_id": 0,
  "track_id": null,
  "pose": {
    "posture": "unknown",
    "posture_confidence": 0.2,
    "action_tags": [],
    "keypoints": [
      {"name": "nose", "x": 1280.0, "y": 181.5, "confidence": 0.037}
    ]
  }
}
```

> **兼容性**：下游 `semantic_projection_node` 只会读取 `label`、`confidence`、`gx`、`gy`、`track_id` 这 5 个字段。多余的 `class_id`、`pose` 等会被忽略但不报错。

---

## 6. 关键参数对照表

### 投影参数 (三个节点共享)

| 参数 | 默认值 | 含义 | 何时需要改 |
|------|--------|------|-----------|
| `camera_fx` | 380.0 | 相机焦距(像素) | 换相机时必须改，否则 3D 位置算不准 |
| `camera_fy` | 380.0 | 相机焦距(像素) | 同上 |
| `camera_cx` | 320.0 | 光心 X (像素) | 同上，通常是图像宽度/2 |
| `camera_cy` | 240.0 | 光心 Y (像素) | 同上，通常是图像高度/2 |
| `depth_scale` | 0.001 | 深度值→米的系数 | 16UC1(mm) 图片填 0.001；32FC1(m) 填 1.0 |
| `target_frame` | `map` | TF 目标坐标 | cartographer 用 `map`；某些 SLAM 用 `odom` |
| `tf_timeout_s` | 0.1 | TF 查询超时 | 网络负载重或 TF 链长时可加大 |

### 吸附参数 (semantic_projection_node)

| 参数 | 默认值 | 含义 | 调优建议 |
|------|--------|------|---------|
| `search_radius_m` | 0.5 | 种子周围搜索占用栅格的半径（米） | 增大=能找到更远的障碍物但可能吸附到错误物体 |
| `occupied_threshold` | 50 | 栅格值超过此值视为占用 | 50 是 ROS 标准；可降低到 30 让吸附更宽松 |
| `min_island_pixels` | 2 | 占用岛屿最少像素数 | 增加=过滤更多噪声碎片 |
| `match_distance` | 1.0 | 记忆匹配的欧氏距离阈值（米） | 物体间距远时增大；密集场景缩小 |
| `smoothing_alpha` | 0.3 | EMA 平滑系数 (0~1) | 越小越平滑但反应越慢；越大越灵敏但越抖 |
| `memory_timeout` | inf | 对象遗忘时间（秒） | inf=永不过期；动态场景建议 10~30 秒 |

### 前端桥接专属参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `input_topic` | `/frontend/detections` | 前端 JSON 话题名 |
| `depth_topic` | `/camera/depth/image_rect_raw` | 深度图话题名 |
| `pose_iou_threshold` | 0.3 | 检测框与姿态框的 IoU 匹配阈值 |
| `enable_pose` | true | 是否输出姿态/动作信息 |
| `slop` | 0.1 | 时间同步容差（秒） |

### 回环节点专属参数 (loop_closure_guard_node)

| 参数 | 默认值 | 含义 | 调优建议 |
|------|--------|------|---------|
| `lc_check_period` | 1.0 | TF 监控频率 (秒) | 太快浪费 CPU；太慢可能漏掉回环事件 |
| `lc_translation_threshold` | 0.20 | 位移跳变阈值 (米) | 太小→正常漂移也被误判；太大→小范围回环漏检 |
| `lc_rotation_threshold` | 0.08 | 旋转跳变阈值 (弧度) | 根据定位噪声和回环幅度调节 |
| `lc_source_frame` | `odom` | 监控参考帧 | 默认通过 `lookup_transform("map", "odom")` 监控 `odom→map`；如果 TF 树不同可改为 `base_footprint` |
| `lc_resnap_enabled` | true | 回环后 snapped 对象是否重吸附 | 若 re-snap 导致不稳定，可临时关闭 |
| `lc_resnap_radius_m` | 0.3 | 首次重吸附半径 | 从较小半径开始，避免直接吸到邻近墙体 |
| `lc_resnap_max_attempts` | 4 | 跨新地图帧最大重试次数 | 地图更新较慢时可适当增加 |
| `lc_resnap_radius_step_m` | 0.15 | 每轮半径增量 | 增量过大会增加错误吸附风险 |
| `lc_resnap_max_radius_m` | 0.6 | 搜索半径上限 | 应结合室内物体间距设置 |
| `lc_stable_checks` | 2 | re-snap 前 TF 连续稳定次数 | 增大更稳健，但会增加等待时间 |
| `lc_stable_translation_threshold` | 0.02 | 稳定检查最大平移变化 | 应显著小于 TF jump 阈值 |
| `lc_stable_rotation_threshold` | 0.01 | 稳定检查最大旋转变化 | 应显著小于 TF jump 阈值 |

---

## 7. 运行指南

### 前置条件

- ROS 2 Jazzy (或 Humble/Iron，`rclpy` 基本通用)
- Cartographer 或其它 SLAM 提供 `/map` 主题
- 深度相机发布 `/camera/depth/image_rect_raw`
- 前端发布检测 JSON 到 `/frontend/detections`（或使用测试数据）

### 启动步骤

```bash
# 1. 构建
cd ~/vscode_workspace/ros2_wp
colcon build --packages-select my_work_pkg
source install/setup.bash

# 2. 启动 Cartographer 建图 (二选一)
ros2 launch fishbot_description cartographer.launch.py

# 3. 启动 RViz 可视化
ros2 launch fishbot_description display_rviz2.launch.py

# 4. 启动语义投影节点 (二选一)
ros2 run my_work_pkg semantic_projection_node
# 或使用带回环修正的版本：
ros2 run my_work_pkg loop_closure_guard_node

# 5. 启动前端桥接节点 (或标准 YOLO 节点，二选一)
ros2 run my_work_pkg frontend_bridge_node
# ros2 run my_work_pkg yolo_seed_projection_node  # 如果用标准 ROS 消息

# 6. 手动发布测试数据 (没有实际前端时)
#    用 JSON 文件模拟前端:
ros2 topic pub /frontend/detections std_msgs/msg/String \
  "data: '$(cat ~/vscode_workspace/ros2_wp/src/my_work_pkg/deepseek_json_20260605_3313ab.json)'" -1
```

### 带参数启动示例

```bash
# 自定义相机参数
ros2 run my_work_pkg frontend_bridge_node \
  --ros-args \
  -p camera_fx:=520.0 \
  -p camera_fy:=520.0 \
  -p camera_cx:=640.0 \
  -p camera_cy:=360.0 \
  -p depth_scale:=1.0 \
  -p enable_pose:=false
```

### 查看输出

```bash
# 监听种子输出
ros2 topic echo /semantic_seed

# 监听可视化 Marker
ros2 topic echo /semantic_projection_node/markers
```

---

## 8. 测试

### 运行单元测试

```bash
cd ~/vscode_workspace/ros2_wp
colcon test --packages-select my_work_pkg
colcon test-result --verbose  # 查看测试结果
```

### 测试内容

| 测试文件 | 测试什么 |
|---------|---------|
| `test_yolo_seed_projection.py` | 深度反投影算法：中心投影、无效深度过滤 |
| `test_single_point_projection.py` | 吸附算法：质心、拒绝无效种子、拒绝墙壁形状、贴边过滤 |

### 手动端到端测试

```bash
# 终端 1: 启动语义投影节点
ros2 run my_work_pkg semantic_projection_node

# 终端 2: 发布测试种子 (无需实际传感器)
ros2 topic pub /semantic_seed std_msgs/msg/String \
  "data: '{\"label\": \"bin\", \"confidence\": 0.9, \"gx\": -2.6, \"gy\": 3.8}'" -1

# 在 RViz 中查看 /semantic_projection_node/markers 是否有绿色框出现
```

---

## 9. 故障排查

### 节点启动了但没有种子输出

1. **确认话题有数据**：`ros2 topic echo /frontend/detections` 有内容吗？
2. **确认深度图有数据**：`ros2 topic hz /camera/depth/image_rect_raw` 有频率吗？
3. **检查时间同步**：两个消息的时间戳在 slop (默认 0.1s) 内吗？如果延迟大，增大 slop 参数。
4. **检查置信度**：检测置信度低于投影器的默认阈值 (0.35) 会被丢弃。

### TF 变换失败

日志中出现 `TF camera_frame -> map failed`：
1. **检查 TF 树**：`ros2 run tf2_tools view_frames` 看 `camera_frame` 到 `map` 的变换链是否完整
2. **SLAM 是否在运行**：map 坐标系通常由 Cartographer 发布，不跑 SLAM 就没有 `map`
3. **增大超时**：`-p tf_timeout_s:=1.0`

### 投影后的坐标明显不对

1. **核对相机内参**：`fx, fy, cx, cy` 和实际相机匹配吗？
2. **核对 depth_scale**：深度图是 16UC1(mm) 还是 32FC1(m)？
3. **核对图像尺寸**：深度图和彩色图分辨率一致吗？不一致时 bbox 坐标会错位。

### 吸附总是拒绝种子

1. **检查占用阈值**：`-p occupied_threshold:=30` 降低门槛
2. **增大搜索半径**：`-p search_radius_m:=1.0`
3. **减小最小像素**：`-p min_island_pixels:=1`
4. **检查种子位置**：种子是否真的落在了地图上有栅格物体的位置？

### 回环后 Marker 位置错位

1. **确认你用的是回环节点**：`ros2 run my_work_pkg loop_closure_guard_node`，而不是 `semantic_projection_node`
2. **检查 TF 监控是否正常工作**：观察节点日志，是否有 `Initialised odom->map reference` 这条消息
3. **检查阈值设置**：如果回环修正没有触发，尝试降低阈值 `-p lc_translation_threshold:=0.02`
4. **确认 TF 树中 `odom` 与 `map` 连通**：节点读取 `lookup_transform("map", "odom")`，Cartographer 必须提供这条链路。`ros2 run tf2_tools view_frames` 确认

### 回环修正过于频繁 / 误触发

1. **增大阈值**：`-p lc_translation_threshold:=0.1` `-p lc_rotation_threshold:=0.05`
2. **降低检查频率**：`-p lc_check_period:=2.0`

---

## 参考资料

- [ROS 2 官方文档](https://docs.ros.org/en/jazzy/)
- [Cartographer SLAM](https://google-cartographer-ros.readthedocs.io/)
- [tf2 教程](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [针孔相机模型](https://en.wikipedia.org/wiki/Pinhole_camera_model)
- [占用栅格地图](https://en.wikipedia.org/wiki/Occupancy_grid_mapping)
