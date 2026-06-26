# 语义对象接口与语义建图主文档

## 1. 项目目标

本模块面向智慧康养移位机器人，将视觉检测结果转换为 `map` 坐标系下的结构化语义对象，为后续导航、避障、床椅对接、人体状态理解和上层任务决策提供输入。

当前阶段不直接修改 SLAM 原始地图，而是先稳定输出：

- RViz Marker：用于调试和现场观察。
- `/semantic_objects`：给上层模块订阅的 JSON 语义对象快照。
- `semantic_objects.json`：给外部脚本、Web 页面或离线调试读取的当前状态文件。

## 2. 当前主链路

```text
opi_yolo_rknn_recorder
  /yolo/results_json
        |
        v
frontend_bridge_node
  detection JSON + depth + TF(camera -> map)
        |
        v
/semantic_seed
        |
        v
semantic_projection_node
  seed -> occupancy snap / visual fallback -> memory tracking
        |
        v
~/markers
/semantic_objects
/tmp/semantic_objects.json
```

如果需要处理 Cartographer 回环，使用：

```text
loop_closure_guard_node
```

替代：

```text
semantic_projection_node
```

二者不要同时运行。

## 3. 节点职责

| 节点 | 作用 |
| --- | --- |
| `opi_yolo_rknn_recorder` | 运行 YOLO/YOLO-pose，发布 `/yolo/results_json`，提供网页预览 |
| `frontend_bridge_node` | 将检测框 + 深度图反投影到 3D，再通过 TF 转为 `/semantic_seed` |
| `semantic_projection_node` | 语义 seed 吸附、记忆追踪、状态确认、Marker 和 JSON 发布 |
| `loop_closure_guard_node` | 在 `semantic_projection_node` 基础上增加回环后的语义对象坐标修正 |
| `raw_seed_visualizer_node` | 直接显示原始 `/semantic_seed`，用于排查投影前误差 |
| `semantic_diag_node` | 诊断 topic 频率、TF 状态、map 是否收到 |

## 4. 输入接口

### 4.1 `/semantic_seed`

类型：

```text
std_msgs/msg/String
```

示例：

```json
{
  "label": "chair",
  "confidence": 0.82,
  "gx": 1.23,
  "gy": 4.56,
  "class_id": 56
}
```

字段说明：

| 字段 | 说明 |
| --- | --- |
| `label` | 语义类别 |
| `confidence` | 检测置信度 |
| `gx`, `gy` | `map` 坐标系下的粗略语义点 |
| `track_id` | 可选，上游跟踪 ID |
| `class_id` | 可选，检测类别 ID |
| `pose` | 可选，人体姿态摘要 |

### 4.2 `/map`

类型：

```text
nav_msgs/msg/OccupancyGrid
```

用途：

- 为 `snapped` 对象提供 OccupancyGrid 吸附依据。
- 过滤墙体大连通分量。
- 回环后为 `snapped` 对象提供新地图上的 re-snap 依据。

## 5. 输出接口

### 5.1 RViz Marker

默认发布：

```text
~/markers
```

Marker 用于调试显示：

- confirmed 对象：绿色框。
- candidate 对象：橙色框，仅当 `show_candidates:=true` 时显示。

### 5.2 `/semantic_objects`

类型：

```text
std_msgs/msg/String
```

内容是 JSON 快照，只包含 `confirmed` 对象。

默认 topic：

```text
/semantic_objects
```

### 5.3 JSON 文件

默认路径：

```text
/tmp/semantic_objects.json
```

该文件是当前状态快照，不是历史日志。每次发布都会覆盖写一次。

如果不想写文件：

```bash
semantic_objects_path:=""
```

## 6. `/semantic_objects` JSON 格式

示例：

```json
{
  "stamp_sec": 1782288000.123,
  "frame_id": "map",
  "count": 1,
  "objects": [
    {
      "id": "chair_0",
      "label": "chair",
      "state": "confirmed",
      "source": "snapped",
      "x": 1.23,
      "y": 4.56,
      "size_x": 0.6,
      "size_y": 0.5,
      "confidence": 0.82,
      "times_seen": 50,
      "last_seen": 1782287999.9,
      "created_at": 1782287995.1
    }
  ]
}
```

字段说明：

| 字段 | 说明 |
| --- | --- |
| `stamp_sec` | 快照生成时间，单位秒 |
| `frame_id` | 坐标系，目前为 `map` |
| `count` | 当前 confirmed 对象数量 |
| `objects` | confirmed 对象数组 |
| `id` | 内部对象 ID，如 `chair_0` |
| `label` | 语义类别 |
| `state` | 当前只会输出 `confirmed` |
| `source` | `snapped` 或 `visual_only` |
| `x`, `y` | `map` 坐标系下位置 |
| `size_x`, `size_y` | 估计尺寸 |
| `confidence` | 历史平均置信度 |
| `times_seen` | 累计观测次数 |
| `last_seen` | 最近一次观测时间 |
| `created_at` | 第一次创建时间 |

## 7. 对象状态

对象状态分为：

```text
candidate -> confirmed
```

规则：

- 新对象先进入 `candidate`。
- 当 `times_seen >= min_confirmed_seen` 时变为 `confirmed`。
- `candidate` 超过 `memory_timeout` 未继续观测，会被 `age()` 清理。
- `confirmed` 不会被 `age()` 删除，会持续保留。
- `/semantic_objects` 和 JSON 文件只保存 `confirmed`。

当前默认：

```text
min_confirmed_seen = 50
memory_timeout = 5.0
```

注意：`DEFAULT_LABEL_POLICIES` 中可以给具体类别设置不同的 `min_confirmed_seen`。当前 `person` 默认是 30。

## 8. 对象来源

### 8.1 `snapped`

表示对象已经吸附到 OccupancyGrid 的局部占据结构。

适合：

- 椅子
- 桌子
- 床
- 柜体
- 垃圾桶等静态物体

特点：

- 会利用 `/map` 做几何校正。
- 会过滤墙体大连通分量。
- 回环后会先做几何修正，再在新 `/map` 上小半径 re-snap。

### 8.2 `visual_only`

表示对象没有吸附到 OccupancyGrid，只保留视觉投影点。

适合：

- 人体
- 动态目标
- 不稳定或不适合写入静态地图的目标

特点：

- 不依赖 `/map` 的局部占据结构。
- 回环后只做 TF 几何修正，不做 re-snap。
- 会检查 `visual_clearance_m`，避免贴墙生成大量 marker。

## 9. 类别策略

默认策略在代码中的 `DEFAULT_LABEL_POLICIES`。

当前核心策略：

```text
default:
  snap_to_occupancy: true
  fallback_visual_on_reject: false
  min_confirmed_seen: 50
  visual_size_m: 0.25
  visual_clearance_m: 0.35

person:
  snap_to_occupancy: false
  fallback_visual_on_reject: true
  min_confirmed_seen: 30
  visual_size_m: 0.25
  visual_clearance_m: 0.35
```

含义：

- 普通类别优先吸附 OccupancyGrid。
- 普通类别吸附失败后默认丢弃，避免墙边误检生成大量对象。
- `person` 不吸附 OccupancyGrid，走 `visual_only`。
- `person` 仍要求距离 occupied cell 至少 `visual_clearance_m`。

可通过 `label_policy_json` 覆盖：

```bash
ros2 launch my_work_pkg semantic_map.launch.py \
  label_policy_json:='{"person":{"min_confirmed_seen":50,"visual_clearance_m":0.6}}'
```

## 10. 回环处理

回环处理节点：

```text
loop_closure_guard_node
```

它通过：

```text
lookup_transform("map", "odom")
```

读取 `odom -> map` 变换，即 `T_map_odom`。

当检测到 TF 跳变后，对对象执行：

```text
P_new_map = T_new * T_old^-1 * P_old_map
```

然后按对象来源分两类处理：

- `source=snapped`：在新 `/map` 上小半径 re-snap，更新 `x/y/size_x/size_y/source`。
- `source=visual_only`：只做几何修正，不 re-snap。

re-snap 不代表新观测，因此不会更新：

- `times_seen`
- `confidence`
- `last_seen`

回环相关参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_loop_closure` | false | 是否用 `loop_closure_guard_node` 替代 `semantic_projection_node` |
| `lc_check_period` | 1.0 | TF 检查周期，单位秒 |
| `lc_translation_threshold` | launch 默认 0.05 | 位移跳变阈值 |
| `lc_rotation_threshold` | launch 默认 0.02 | 旋转跳变阈值 |
| `lc_source_frame` | odom | 通过 `lookup_transform("map", lc_source_frame)` 监控 |
| `lc_resnap_enabled` | true | 回环后是否对 `snapped` 对象 re-snap |
| `lc_resnap_radius_m` | 0.3 | re-snap 搜索半径 |

## 11. 运行方式

### 11.1 普通语义投影

```bash
ros2 launch my_work_pkg semantic_map.launch.py \
  camera_frame:=camera_optical_link \
  semantic_objects_path:=/tmp/semantic_objects.json
```

### 11.2 带回环修正

```bash
ros2 launch my_work_pkg semantic_map.launch.py \
  use_loop_closure:=true \
  camera_frame:=camera_optical_link \
  semantic_objects_path:=/tmp/semantic_objects.json \
  lc_resnap_enabled:=true \
  lc_resnap_radius_m:=0.3
```

### 11.3 查看输出

```bash
ros2 topic echo /semantic_objects
cat /tmp/semantic_objects.json
```

### 11.4 查看诊断

```bash
ros2 topic echo /semantic_diag
```

## 12. 调试建议

### 12.1 原始 seed 是否正确

启动：

```bash
ros2 run my_work_pkg raw_seed_visualizer_node
```

如果原始 seed 已经偏到墙上，问题在前端投影、深度或 TF。

### 12.2 confirmed 数量是否合理

查看：

```bash
ros2 topic echo /semantic_objects
```

如果长时间没有对象：

- 检查 `min_confirmed_seen` 是否过高。
- 检查 `/semantic_seed` 是否持续发布。
- 检查对象是否因 `memory_timeout` 在 candidate 阶段被清理。

### 12.3 墙边 marker 过多

优先检查：

```text
fallback_visual_on_reject
visual_clearance_m
max_island_size_m
search_radius_m
```

普通静态物体建议：

```text
fallback_visual_on_reject = false
```

### 12.4 回环后对象错位

检查：

- 是否使用 `loop_closure_guard_node`。
- 是否有 `Initialised odom->map reference` 日志。
- 是否出现 `[LoopClosure] TF jump detected` 日志。
- 日志中的 `resnapped`、`resnap_failed`、`visual_only` 计数是否符合预期。
- `lc_resnap_radius_m` 是否过小或过大。

## 13. 当前限制

- `/semantic_objects` 仍使用 JSON String，后续可升级为自定义 ROS msg。
- confirmed 对象目前不由 `age()` 自动删除，后续需要单独设计长期语义地图删除/编辑策略。
- re-snap 依赖当前 `/map` 的 OccupancyGrid 质量，如果地图墙体连通严重，仍需要调 `max_island_size_m`。
- 前端内参目前主要依赖参数配置，后续建议接入 `CameraInfo` 自动读取。
- 当前 JSON 文件是当前状态快照，不保存历史轨迹。

## 14. 后续扩展

建议后续继续做：

- 长期语义地图持久化。
- 静态家具对象的人工确认/删除接口。
- `/semantic_objects` 自定义 msg。
- `semantic_costmap_layer`。
- Web 前端展示语义对象列表。
- CameraInfo 自动内参。
