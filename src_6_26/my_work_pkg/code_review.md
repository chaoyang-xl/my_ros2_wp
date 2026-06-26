# my_work_pkg 代码审查报告

> 审查时间：2026-06-11
> 审查范围：`my_work_pkg/` 全部源码、测试、配置

---

## 目录

- [第一部分：代码问题与修改建议](#第一部分代码问题与修改建议)
  - [一、严重问题（会导致运行失败）](#一严重问题会导致运行失败)
  - [二、架构设计问题](#二架构设计问题)
  - [三、算法与逻辑问题](#三算法与逻辑问题)
  - [四、代码质量问题](#四代码质量问题)
  - [五、优先级排序](#五优先级排序)
- [第二部分：工程级补充建议](#第二部分工程级补充建议)
  - [一、部署运行](#一部署运行)
  - [二、鲁棒性](#二鲁棒性)
  - [三、可观测性](#三可观测性)
  - [四、质量保障](#四质量保障)
  - [五、性能](#五性能)
  - [六、总结优先级矩阵](#六总结优先级矩阵)

---

# 第一部分：代码问题与修改建议

## 一、严重问题（会导致运行失败）

### 1. 导入名不匹配 — `yolo_seed_projection_node.py`

`yolo_seed_projection_node.py` 第 22 行导入了 `YoloDepthSeedProjector`，但 `yolo_seed_projection.py` 中定义的类名是 `YoloLidarSeedProjector`。这个节点**启动即崩溃**。

```python
# 当前（错误）
from my_work_pkg.yolo_seed_projection import (
    YoloDetection2D,
    YoloDepthSeedProjector,   # ← 不存在这个类
)
```

**建议**：统一为 `YoloLidarSeedProjector`（或反过来改类名，但要全局一致）。

### 2. 测试文件完全过期 — `test_yolo_seed_projection.py`

`test_yolo_seed_projection.py` 测试的是一个 **LiDAR-based** 的旧版投影器（用 `camera_hfov_rad`、`scan_window_half`、`pixel_to_bearing`、`robust_range` 等），但当前 `yolo_seed_projection.py` 已经重写为 **depth-image-based** 的 `YoloLidarSeedProjector`（用 `fx/fy/cx/cy`、`depth_image`）。所有 4 个测试用例**全部会失败**。

**建议**：重写测试，基于当前的深度图反投影 API（`project(det, depth_image)`）。

---

## 二、架构设计问题

### 3. `SnappedObject` vs `ProjectedSemanticObject` 类型混乱

当前数据流：

```
project() → ProjectedSemanticObject → 传给 SemanticMemory.update(snapped: SnappedObject)
```

`SnappedObject` 和 `ProjectedSemanticObject` 是两套独立的数据类，字段部分重叠但不完全一致。Python 鸭子类型让代码"碰巧能跑"，但：
- `SnappedObject` 缺少 `yaw`、`total_pixels`、`aspect_ratio`、`points_xy` 等字段
- 语义上 `update()` 声称接收 `SnappedObject`，实际传的是 `ProjectedSemanticObject`

**建议**：让 `SnappedObject` 继承自 `ProjectedSemanticObject`，或者直接删掉 `SnappedObject`，统一用 `ProjectedSemanticObject`。

### 4. `TrackedObject` 丢失了投影阶段的关键几何信息

`TrackedObject` 只有 `x, y, size_x, size_y`，丢失了：
- **`yaw`**（物体朝向）→ RViz Marker 永远画轴对齐矩形，无法展示真实朝向
- **`total_pixels`** / **`aspect_ratio`** → 后续无法用于过滤或决策
- **`points_xy`** → 投影时算了但完全没用（白算了）

**建议**：
- 给 `TrackedObject` 增加 `yaw` 字段，EMA 平滑时也更新它
- Marker 绘制时用 `yaw` 做旋转变换，让矩形框贴合物体真实朝向
- 要么在 `TrackedObject` 里保存 `points_xy`，要么在 `project()` 中不要每次都算

### 5. `loop_closure_guard_node` 与 `semantic_projection_node` 大量重复

`loop_closure_guard_node.py` 把 `_map_cb`、`_seed_cb`、`_publish_memory_cb` 三个回调以及所有参数声明全部复制了一份（约 150 行重复代码）。

**建议**：让 `LoopClosureGuardNode` **继承** `SemanticProjectionNode`，只覆写需要差异的部分 + 增加回环检测回调。这样后续改 Marker 样式或吸附逻辑不用改两处。

---

## 三、算法与逻辑问题

### 6. 洪水填充没有面积上限，大物体性能风险

`_flood_fill_island` 会探索所有连通的占用像素。如果种子恰好落在一大片墙壁或大型家具上，BFS 可能遍历数千甚至上万个像素。

**建议**：增加一个 `max_island_pixels` 参数（如 5000），BFS 过程中超过上限就提前终止并拒绝，避免在大型连通区域上浪费计算。

### 7. `_passes_geometry_check` 贴边过滤逻辑冗余且过严

`_passes_geometry_check` 有两层贴边过滤：

1. **第一层**：岛屿中**任何一个像素**碰到地图边界就拒绝 → 太严格，一个大物体的边缘刚好在地图边界就被误杀
2. **第二层**：质心距离边界的 margin 检查 → 这个更合理

而且当 `wall_margin_m = 0`（默认值 0.2 > 0 所以默认走不到这个分支，但逻辑上）时第一层检查仍然生效。

**建议**：删掉第一层（任何像素碰边就拒绝），只保留质心 margin 检查。或者改为"超过 N% 的像素碰边才拒绝"。

### 8. 置信度更新不是真正的 EMA

`SemanticMemory.update` 中：

```python
best_match.confidence = (best_match.confidence * best_match.times_seen + snapped.confidence) / (best_match.times_seen + 1)
```

这是**累积平均值**，不是 EMA。随着 `times_seen` 增大，新观测的权重越来越小（第 100 次观测时权重只有 1/101）。

**建议**：如果想保持与位置/尺寸一致的 EMA 策略，改为：

```python
best_match.confidence = (1 - a) * best_match.confidence + a * snapped.confidence
```

或者如果你想要累积平均，至少把变量名和注释改准确。

### 9. Marker ID 基于索引，对象增删时会闪烁

`_publish_memory_cb` 用 `enumerate` 的序号 `i` 作为 Marker ID。当某个对象被 `age()` 清理后，后续对象的 ID 整体前移，RViz 会认为是不同 Marker，造成视觉闪烁。

**建议**：用 `TrackedObject.object_id` 的哈希值（或给每个 `TrackedObject` 分配一个稳定的 int marker_id）作为 Marker ID。同时，对被删除的对象发送 `action=DELETE` 的 Marker 主动清除。

---

## 四、代码质量问题

### 10. `_sample_island_points` 白算 — 无用功

`_sample_island_points` 在每次 `project()` 时都会执行，但：
- `TrackedObject` 不保存 `points_xy`
- `semantic_projection_node` 和 `loop_closure_guard_node` 都不使用这个数据
- 如果岛屿有上千像素，这是不必要的开销

**建议**：要么在 `TrackedObject` / 下游用上 `points_xy`，要么把 `_sample_island_points` 改为可选（默认不计算）。

### 11. 重复导入 `hypot`

`semantic_projection.py` 第 15 行和第 17 行重复导入了 `hypot`：

```python
from math import atan2, cos, hypot, sin   # 第一次
from math import hypot                     # 第二次（冗余）
```

### 12. `frontend_bridge_node` 的深度图同步方式简陋

`frontend_bridge_node.py` 中 `_depth_cb` 只保存最新深度消息，`_json_cb` 到达时直接使用（可能已过期的）缓存深度图。代码里能看到被注释掉的 `ApproximateTimeSynchronizer`。

**问题**：
- 没有时间戳校验，深度图可能和检测框不是同一帧
- 深度图比检测慢时，用了旧深度，投影坐标就偏了
- `_latest_depth_msg` 永远不清理，即使已经非常陈旧

**建议**：恢复 `ApproximateTimeSynchronizer`，或者至少检查深度消息的时间戳和当前时间差，超过阈值（如 0.5s）就丢弃。

### 13. `package.xml` 中 `description` 和 `license` 还是 TODO

```xml
<description>TODO: Package description</description>
<license>TODO: License declaration</license>
```

发布前建议补全。同样 `setup.py` 的 `description` 和 `license` 也是 TODO。

### 14. 未使用的 import

- `semantic_projection.py` 第 16 行 `from typing import Literal, Sequence` — `Sequence` 只在模块级函数签名里用
- `yolo_seed_projection.py` `from typing import Sequence` 和 `import numpy as np` — `Sequence` 完全未使用

---

## 五、优先级排序

| 优先级 | 编号 | 问题 | 影响 |
|--------|------|------|------|
| P0 紧急 | #1 | `YoloDepthSeedProjector` 导入错误 | 节点无法启动 |
| P0 紧急 | #2 | 测试文件与代码不匹配 | CI 全挂 |
| P1 重要 | #3 | SnappedObject/ProjectedSemanticObject 类型混乱 | 维护困难，后续易出 bug |
| P1 重要 | #4 | TrackedObject 丢失 yaw 等几何信息 | Marker 显示不准确 |
| P1 重要 | #12 | 深度图同步方式简陋 | 投影精度差 |
| P2 改进 | #5 | 代码重复（loop_closure vs semantic_projection） | 维护成本 |
| P2 改进 | #6 | 洪水填充无上限 | 性能风险 |
| P2 改进 | #7 | 贴边过滤过严 | 误杀大物体 |
| P2 改进 | #9 | Marker ID 不稳定 | 视觉闪烁 |
| P3 清洁 | #8,#10,#11,#13,#14 | 代码质量 | 专业性、可维护性 |

---

# 第二部分：工程级补充建议

## 一、部署运行（缺了就不能方便地上线）

### 1. 缺 Launch 文件

包里没有 `launch/` 目录。目前用户必须手动依次 `ros2 run` 三四个节点，参数全靠命令行敲。

**需要**：一个 `launch/semantic_pipeline.launch.py`，统一管理：
- 启动 `frontend_bridge_node`（或 `yolo_seed_projection_node`，通过 launch argument 切换）
- 启动 `semantic_projection_node`（或 `loop_closure_guard_node`）
- 启动 `raw_seed_visualizer_node`（可选，通过 `enable_debug` 参数控制）
- 所有参数集中透传

### 2. 缺参数 YAML 文件

所有参数散落在各节点的 `declare_parameter` 默认值里。调参全靠记忆和命令行。

**需要**：`config/semantic_params.yaml`，例如：

```yaml
semantic_projection_node:
  ros__parameters:
    search_radius_m: 0.5
    occupied_threshold: 65
    match_distance: 1.0
    smoothing_alpha: 0.3
    memory_timeout: 10.0

frontend_bridge_node:
  ros__parameters:
    camera_fx: 320.0
    camera_fy: 320.0
    depth_scale: 1.0
```

launch 文件里 `include` 这个 YAML，实现"改配置不改代码"。

### 3. 缺 rviz 配置文件

每次开 rviz 都要手动添加 Marker 话题、调显示样式。

**需要**：`rviz/semantic_view.rviz`，预配置好 `/map`、`~/markers`、TF 等显示项。

---

## 二、鲁棒性（缺了就在现场出问题）

### 4. 节点崩溃会导致整个管道断掉

当前任何一个回调里抛异常，整个节点就挂了。特别是：

- `frontend_bridge_node.py` 的 `_synced_cb`：JSON 解析、CV Bridge、TF 变换中任何一个异常如果没被 catch 到就会崩
- `semantic_projection_node.py` 的 `_seed_cb`：`project()` 内部的洪水填充如果出现意外也可能崩

**需要**：每个回调入口加 `try/except` 兜底，`warn` 日志记录异常后 `return`，不让单个坏消息杀死节点。

### 5. 深度图陈旧检测缺失

`frontend_bridge_node.py` 中 `_latest_depth_msg` 只存不用时间戳校验。深度相机掉了或卡住时，节点会用几秒前的旧深度做投影，输出大量错误坐标。

**需要**：

```python
# _json_cb 中检查深度消息是否过期
depth_age = (json_msg.header.stamp - depth_msg.header.stamp)
if abs(depth_age_sec) > 0.5:
    self.get_logger().warn("Depth image too stale, skipping")
    return
```

### 6. TF 失败后缺少降级策略

TF 查不到时当前只是 `debug` 日志 + 丢弃。如果 TF 链长时间不可用（比如 Cartographer 还没初始化完），节点会静默丢弃所有种子，用户完全不知道发生了什么。

**需要**：
- 连续 N 次 TF 失败时升级为 `warn` 日志
- 发布一个诊断状态（见下文第 8 条）

### 7. `memory_timeout = inf` 的内存泄漏风险

`loop_closure_guard_node.py` 默认 `memory_timeout=np.inf`，对象永远不会被清理。长时间运行后 `self.objects` 无限增长。

**需要**：设一个合理的默认值（如 60 秒），或者增加 `max_objects` 上限，超出时淘汰最老/最低置信度的。

---

## 三、可观测性（缺了就调不动、排不了障）

### 8. 缺运行状态统计 / 诊断话题

目前只能从日志零散地看到一些信息。没有结构化的运行状态输出。

**需要**：发布一个诊断话题（如 `~/diagnostics`，`std_msgs/String` 或自定义消息），定期输出：

```json
{
  "map_received": true,
  "seeds_received": 142,
  "seeds_projected": 98,
  "seeds_rejected": 44,
  "rejection_reasons": {
    "low_confidence": 5,
    "no_occupied_nearby": 20,
    "geometry_fail": 19
  },
  "tracked_objects": 7,
  "tf_failures": 3,
  "loop_closures_detected": 1
}
```

这对现场调试极其重要。

### 9. 缺被拒绝种子的可视化

目前只有成功吸附的对象会显示为 Marker。被拒绝的种子（置信度低、找不到占用栅格、几何不通过）**完全不可见**，调试时根本不知道发生了什么。

**需要**：在 `semantic_projection_node` 中增加一个可选的 `~/rejected_markers` 话题，用红色 × 或半透明球体显示被拒绝的种子位置，按拒绝原因着色。

### 10. 缺结构化日志标签

当前日志全靠 `self.get_logger().info(f"...")`，无法按级别或模块过滤。

**需要**：利用 ROS 2 的 logger 子节点：

```python
self._snap_logger = self.get_logger().get_child("snap")
self._memory_logger = self.get_logger().get_child("memory")
self._lc_logger = self.get_logger().get_child("loop_closure")
```

这样可以用 `ros2 run --ros-args --log-level snap:=debug` 精确控制。

---

## 四、质量保障（缺了就不敢发布）

### 11. 单元测试严重过时

`test_yolo_seed_projection.py` 测试的是已删除的旧 API，全部会失败。`test_single_point_projection.py` 覆盖了基本场景但缺少：
- `SemanticMemory` 的匹配/平滑/老化测试
- 边界情况：种子正好在地图边缘、极大岛屿、超多像素
- `_estimate_yaw` 的 PCA 方向估计准确性

### 12. 缺集成测试 / 端到端测试

没有从"输入 JSON → 输出 Marker"的端到端测试。

**需要**：用 `launch_testing` 写一个集成测试：
1. 发布一个假地图（OccupancyGrid）
2. 发布一个假种子（String JSON）
3. 监听 `~/markers` 输出
4. 验证有正确 Marker 产生

### 13. 缺 `package.xml` 运行时依赖声明

`cv_bridge`、`message_filters` 在代码里 import 了但 `package.xml` 没声明：

```xml
<!-- 缺少 -->
<depend>cv_bridge</depend>
<depend>message_filters</depend>
```

在另一台机器上 `rosdep install` 时就会漏装。

---

## 五、性能（缺了在大场景下会卡）

### 14. 洪水填充的内存与计算开销

`_flood_fill_island` 用 `set` 做 `visited`，对大地图（如 4000×4000 像素）开销大。

**需要**：
- 增加 `max_island_pixels` 参数（上限保护）
- 考虑用 `numpy` 数组做 visited 标记（O(1) 查找 vs set 的 hash 开销）

### 15. 每次收到地图都完整重建 `OccupancyGridMap`

`_map_cb` 每次收到 `/map` 消息（Cartographer 可能 1Hz 发布整张地图）都会重建整个对象，包括拷贝全部 `data`。对大地图（几十万个像素）有开销。

**需要**：对比新旧地图的 `header.stamp`，如果时间戳没变就跳过。或用增量更新策略。

### 16. `_sample_island_points` 算了但没人用

每次 `project()` 都调用 `_sample_island_points` 计算岛屿点坐标，但 `TrackedObject` 不保存这个字段，下游也不消费。**白白浪费计算**。

**需要**：要么加个参数默认不计算，要么在下游真正用到时才计算。

---

## 六、总结优先级矩阵

### 第一部分（代码问题）

| 优先级 | 编号 | 问题 | 影响 |
|--------|------|------|------|
| P0 紧急 | #1 | `YoloDepthSeedProjector` 导入错误 | 节点无法启动 |
| P0 紧急 | #2 | 测试文件与代码不匹配 | CI 全挂 |
| P1 重要 | #3 | SnappedObject/ProjectedSemanticObject 类型混乱 | 维护困难 |
| P1 重要 | #4 | TrackedObject 丢失 yaw 等几何信息 | Marker 不准确 |
| P1 重要 | #12 | 深度图同步方式简陋 | 投影精度差 |
| P2 改进 | #5 | 代码重复 | 维护成本 |
| P2 改进 | #6 | 洪水填充无上限 | 性能风险 |
| P2 改进 | #7 | 贴边过滤过严 | 误杀大物体 |
| P2 改进 | #9 | Marker ID 不稳定 | 视觉闪烁 |
| P3 清洁 | #8,#10,#11,#13,#14 | 代码质量 | 可维护性 |

### 第二部分（工程补充）

| 优先级 | 项目 | 编号 |
|--------|------|------|
| **P0 — 必须有** | Launch 文件 | 工程 #1 |
| | 参数 YAML | 工程 #2 |
| | 修复测试 | 工程 #11 |
| | 补 `package.xml` 依赖 | 工程 #13 |
| | 节点异常兜底 | 工程 #4 |
| **P1 — 上线前要** | 诊断话题 | 工程 #8 |
| | 被拒绝种子可视化 | 工程 #9 |
| | 深度图陈旧检测 | 工程 #5 |
| | TF 失败降级 | 工程 #6 |
| | 内存泄漏保护 | 工程 #7 |
| | Rviz 配置文件 | 工程 #3 |
| **P2 — 锦上添花** | 集成测试 | 工程 #12 |
| | 日志子模块 | 工程 #10 |
| | 性能优化 | 工程 #14 #15 #16 |
