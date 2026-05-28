# semantic_map

该仓库提供一个语义地图模块的最小可运行实现：把视觉检测结果（类别、2D bbox、深度）投影到 Cartographer `/map` 坐标系，并融合成带矩形尺寸和标签的二维语义对象。

## 当前代码覆盖的链路

1. 读取相机内参 `CameraInfo.k`。
2. 从检测框中心附近取深度中位数。
3. 将像素点反投影到相机三维坐标。
4. 使用相机到 `map` 的 TF 变换得到地图坐标。
5. 按类别先验生成二维矩形框尺寸。
6. 对多帧结果做同类近邻融合，稳定对象 ID、中心点、置信度和生命周期。
7. 生成可用于 RViz `MarkerArray` 的二维矩形角点和颜色信息。

## 运行 JSON 示例

```bash
python -m semantic_map.json_demo examples/sample_semantic_projection.json
# 或安装后运行：semantic_map_json_demo examples/sample_semantic_projection.json
```

输出中的 `center_x`、`center_y` 就是语义目标在 `map` 坐标系下的位置，`size_x`、`size_y` 是可在 RViz 或语义图层中绘制的矩形框尺寸。

## 接入 ROS 2 的建议

- 上游检测节点发布 `label`、`confidence`、bbox 中心和尺寸。
- 语义投影节点订阅 RGB-D、CameraInfo、TF 和检测结果。
- 将 `SemanticProjector.project_detections()` 的结果交给 `SemanticObjectTracker.update()`。
- 使用 `object_to_rectangle()` 生成矩形角点，再发布为 `/semantic_map/markers` 的 `MarkerArray`，在 RViz 中画矩形框和文字标签。
