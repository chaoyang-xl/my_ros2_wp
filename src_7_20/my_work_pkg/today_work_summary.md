# 语义建图阶段性开发记录

日期：2026-06-23

## 1. 背景

当前项目面向智慧康养移位机器人，目标是在 ROS 2 环境中把 Orange Pi 5 Plus + Orbbec GEMINI 336L + YOLO/YOLO-pose 的视觉结果转换为导航地图坐标系下的结构化语义信息。

今天主要围绕 `my_work_pkg` 中的语义投影链路进行功能优化，重点解决：

- 语义目标容易吸附到墙体 OccupancyGrid 上。
- 单帧检测直接显示，容易误检、闪烁。
- 不同类别目标缺少差异化策略。
- 现场测试时缺少统一诊断信息。

涉及的主要节点：

- `frontend_bridge_node`
- `semantic_projection_node`
- `loop_closure_guard_node`
- `raw_seed_visualizer_node`
- `semantic_diag_node`

## 2. 当前主链路

当前语义建图链路如下：

```text
opi_yolo_rknn_recorder
  /yolo/results_json
        |
        v
frontend_bridge_node
  JSON + depth + TF(camera -> map)
        |
        v
/semantic_seed
        |
        v
semantic_projection_node
  seed -> occupancy snap / visual fallback -> memory tracking
        |
        v
RViz MarkerArray
```

其中：

- `opi_yolo_rknn_recorder` 负责 YOLO/YOLO-pose 推理、网页预览、JSON 输出。
- `frontend_bridge_node` 把 2D bbox + depth 反投影到相机坐标，再通过 TF 转到 `map`。
- `semantic_projection_node` 负责语义吸附、过滤、追踪、平滑、Marker 发布。

## 3. 已完成优化

### 3.1 拒绝墙体大连通分量

问题：

`semantic_projection_node` 原本会在 seed 附近寻找最近的占据栅格。如果 `/map` 中最近的稳定占据结构是墙，语义对象就会被吸附到墙上。

优化：

在 `SinglePointSemanticProjector` 中增加大岛屿过滤：

- `max_island_size_m`
- `max_total_pixels`

当 flood fill 得到的占用岛屿尺寸过大时，认为它更可能是墙体、房间轮廓或大结构，而不是可语义标注的独立物体，直接拒绝吸附。

默认参数：

```text
max_island_size_m = 2.0
max_total_pixels = 0
```

调参建议：

```bash
ros2 run my_work_pkg semantic_projection_node --ros-args \
  -p max_island_size_m:=1.5 \
  -p search_radius_m:=0.2
```

如果仍然吸墙，可尝试：

```text
max_island_size_m = 1.0 ~ 1.5
```

如果正常家具也被过滤，可放宽到：

```text
max_island_size_m = 2.0 ~ 2.5
```

### 3.2 candidate / confirmed 状态机

问题：

单帧检测结果如果直接进入语义地图，会带来误检、抖动和短时闪烁。

优化：

`SemanticMemory` 增加对象状态：

```text
candidate -> confirmed
```

默认行为：

- 新对象第一次出现为 `candidate`。
- 达到 `min_confirmed_seen` 次观测后变成 `confirmed`。
- 默认 Marker 只显示 `confirmed`。
- 可通过 `show_candidates:=true` 显示候选对象。

关键参数：

```text
min_confirmed_seen = 2
show_candidates = false
```

运行示例：

```bash
ros2 launch my_work_pkg semantic_map.launch.py \
  camera_frame:=camera_optical_link \
  show_candidates:=true
```

### 3.3 类别策略 label policy

问题：

不同目标不适合使用同一套吸附和生命周期规则。例如：

- `person` 是动态目标，不应该吸附到墙上，也不应该写入长期语义地图。
- `chair/table/bed` 等家具更适合做稳定语义目标。

优化：

`semantic_projection_node` 增加默认类别策略：

```python
default:
  snap_to_occupancy: true
  fallback_visual_on_reject: false
  min_confirmed_seen: 2
  visual_size_m: 0.35
  visual_clearance_m: 0.35

person:
  snap_to_occupancy: false
  fallback_visual_on_reject: true
  min_confirmed_seen: 3
  visual_size_m: 0.45
  visual_clearance_m: 0.35
```

含义：

- 普通物体优先吸附 OccupancyGrid。
- 普通物体吸附失败后默认丢弃，避免墙边误检形成大量 Marker。
- `person` 默认不做占据栅格吸附，只使用视觉投影点，避免吸墙。
- `person` 需要 3 次观测才 confirmed。
- 允许 `visual_only` 的类别也必须满足 `visual_clearance_m`，避免贴墙保留。

可通过 `label_policy_json` 覆盖：

```bash
ros2 launch my_work_pkg semantic_map.launch.py \
  label_policy_json:='{"person":{"min_confirmed_seen":4},"chair":{"visual_size_m":0.5}}'
```

### 3.4 visual_only fallback

问题：

真实场景中并非所有语义目标都会在 `/map` 中形成独立占据岛屿。强制吸附 OccupancyGrid 会导致：

- 吸附到墙。
- 吸附失败后完全丢失目标。

优化：

当吸附失败时，如果策略允许 `fallback_visual_on_reject`，节点会保留原始视觉 seed 位置，生成：

```text
source = visual_only
```

这样可以区分：

```text
snapped      已成功吸附到占据小岛
visual_only 视觉位置保留，未吸附占据栅格
```

后续长期语义地图和导航代价层可以按 `source` 做不同处理。

### 3.5 semantic_diag_node 诊断节点

新增节点：

```bash
ros2 run my_work_pkg semantic_diag_node
```

统一 launch 中默认启动，可通过：

```bash
ros2 launch my_work_pkg semantic_map.launch.py enable_diag:=false
```

关闭。

诊断节点发布：

```text
/semantic_diag
```

内容包括：

- `/yolo/results_json` 频率
- depth 频率
- `/semantic_seed` 频率
- `/map` 是否收到
- `camera_frame -> map` TF 是否可用
- 每个 topic 的消息总数、Hz、age、stale 状态

查看方式：

```bash
ros2 topic echo /semantic_diag
```

示例字段：

```json
{
  "ok": true,
  "map_received": true,
  "tf": {
    "source": "camera_optical_link",
    "target": "map",
    "ok": true
  },
  "topics": {
    "json": {"hz": 8.0, "total": 100, "age_s": 0.1, "stale": false},
    "depth": {"hz": 30.0, "total": 300, "age_s": 0.03, "stale": false},
    "seed": {"hz": 5.0, "total": 80, "age_s": 0.2, "stale": false},
    "map": {"hz": 0.0, "total": 1, "age_s": 10.0, "stale": true}
  }
}
```

## 4. 修改文件

主要修改：

```text
my_work_pkg/my_work_pkg/semantic_projection.py
my_work_pkg/my_work_pkg/semantic_projection_node.py
my_work_pkg/my_work_pkg/loop_closure_guard_node.py
my_work_pkg/my_work_pkg/semantic_diag_node.py
my_work_pkg/launch/semantic_map.launch.py
my_work_pkg/setup.py
```

新增测试：

```text
my_work_pkg/test/test_semantic_memory.py
```

更新测试：

```text
my_work_pkg/test/test_single_point_projection.py
```

## 5. 验证结果

已运行：

```bash
python3 -m pytest \
  my_work_pkg/test/test_single_point_projection.py \
  my_work_pkg/test/test_semantic_memory.py
```

结果：

```text
8 passed
```

已运行语法检查：

```bash
python3 -m py_compile \
  my_work_pkg/my_work_pkg/semantic_projection.py \
  my_work_pkg/my_work_pkg/semantic_projection_node.py \
  my_work_pkg/my_work_pkg/loop_closure_guard_node.py \
  my_work_pkg/my_work_pkg/semantic_diag_node.py \
  my_work_pkg/launch/semantic_map.launch.py \
  my_work_pkg/setup.py
```

结果：

```text
通过
```

注意：

`test_yolo_seed_projection.py` 目前仍是旧版 LiDAR 投影接口测试，而当前代码中的 `YoloLidarSeedProjector` 已经改为深度图反投影接口，因此该测试需要后续单独更新。

## 6. 推荐运行方式

仿真或当前测试推荐：

```bash
ros2 launch my_work_pkg semantic_map.launch.py \
  camera_frame:=camera_optical_link \
  show_candidates:=true \
  max_island_size_m:=1.5
```

只运行语义吸附节点：

```bash
ros2 run my_work_pkg semantic_projection_node --ros-args \
  -p search_radius_m:=0.2 \
  -p max_island_size_m:=1.5 \
  -p min_confirmed_seen:=2 \
  -p show_candidates:=true
```

查看诊断：

```bash
ros2 topic echo /semantic_diag
```

查看原始 seed：

```bash
ros2 run my_work_pkg raw_seed_visualizer_node
```

## 7. 后续建议

后续等具体需求明确后，建议优先做以下方向：

### 7.1 CameraInfo 自动内参

当前 `frontend_bridge_node` 仍依赖手动参数：

```text
camera_fx
camera_fy
camera_cx
camera_cy
```

落地时建议订阅 `sensor_msgs/msg/CameraInfo`，自动读取相机内参，避免相机分辨率或驱动配置变化导致投影偏差。

### 7.2 静态家具语义地图

将 `chair/bed/table/toilet/cabinet` 等 confirmed 对象写入长期语义对象列表，例如：

```text
semantic_objects.json
```

暂时不直接覆盖 SLAM `/map`。

### 7.3 动态人体安全层

`person` 作为短生命周期动态目标：

- 不吸附墙。
- 不写长期地图。
- 后续接入 costmap 或安全策略。

可用于：

- 保持安全距离。
- 暂停移动。
- 辅助护理姿态状态判断。

### 7.4 semantic_costmap_layer

根据语义对象生成导航代价：

- 静态家具：长期高代价或任务目标参考。
- 人体：短期动态高代价，安全膨胀更大。

### 7.5 诊断信息增强

后续可在 `/semantic_diag` 中增加拒绝原因统计：

```text
no_map
invalid_json
no_nearby_occupied
wall_like
too_large
visual_only
confirmed
candidate
```

这样现场调试时可以直接判断问题出在检测、深度、TF、地图还是吸附策略。
