# semantic_map_pkg 实现状态（2026-05-15）

## 当前已实现（可用于阶段验收）

1. 语义投影主链路已打通：`2D bbox + depth + intrinsics + camera_to_map -> map 语义对象`。  
2. 支持两种尺寸模式：
- `prior_size`：使用类别先验尺寸。
- `observed_size`：从 bbox 内深度点估计 `center/size/yaw`。
3. 多帧融合已具备：
- 同类近邻关联。
- `center/size/yaw` 平滑更新。
- 生命周期管理（过期清理）。
4. 轻量点云融合已加入：
- 每个对象维护局部 `points_xy`（map 平面点）。
- 融合时基于累计点重估几何。
- 点数上限控制（默认每对象 1200 点）用于部署。
5. ROS2 最小可视化链路可运行：
- 地图 `/map`（map_server）已验证。
- `/semantic_map/markers` demo 发布可用于 RViz 联调。

## “完全体”判断

结论：**还不是完全体**，但已经达到“可演示、可联调、可部署优化”的阶段。

## 仍需补齐的能力（建议顺序）

1. 真实 ROS2 输入节点化（替换 demo）：
- 订阅 `Image/CameraInfo/TF/Detection2DArray`。
- 在线调用 `SemanticProjector + SemanticObjectTracker` 发布对象与 Marker。
2. 观测质量增强：
- 点云离群点过滤（统计滤波或半径滤波）。
- 遮挡/误检场景下的几何鲁棒性策略。
3. 语义查询接口：
- 按类别查询目标位置（如 `toilet`）。
- 输出导航候选点（避障约束）。
4. 可选 Nav2 代价地图叠加：
- 将语义对象栅格化到语义层（`/semantic_map/grid` 或 costmap layer）。

## 部署建议（Orange Pi 5 Plus）

1. 初始参数建议：
- `max_points_per_object=600~1200`
- 目标更新频率 `3~5Hz`（检测 10Hz 时可降采样更新）
2. 若 CPU 偏高，优先降：
- bbox 采样密度（增大 stride）。
- 每对象点数上限。
- 几何重估频率（非每帧重估）。

