"""Visualization helpers for drawing semantic rectangles on a 2D map."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin

from semantic_map_pkg.config import DEFAULT_SEMANTIC_CLASSES
from semantic_map_pkg.fusion import SemanticObject


@dataclass(frozen=True)
class Rectangle2D:
    """A closed 2D rectangle polyline in map coordinates."""

    label: str
    points: tuple[tuple[float, float], ...]
    color_rgba: tuple[float, float, float, float]

#从中心/尺寸/朝向生成世界角点
def rectangle_corners(
    center_x: float,
    center_y: float,
    size_x: float,
    size_y: float,
    yaw: float = 0.0,
) -> tuple[tuple[float, float], ...]:
    """Return four rectangle corners plus the first corner again to close the line."""

    half_x = size_x / 2.0
    half_y = size_y / 2.0
    local_corners = (
        (-half_x, -half_y),
        (half_x, -half_y),
        (half_x, half_y),
        (-half_x, half_y),
    )
    cos_yaw = cos(yaw)
    sin_yaw = sin(yaw)
    world_corners = tuple(
        (
            center_x + local_x * cos_yaw - local_y * sin_yaw,
            center_y + local_x * sin_yaw + local_y * cos_yaw,
        )
        for local_x, local_y in local_corners
    )
    return world_corners + (world_corners[0],)

#将语义对象转换为矩形数据结构，包含标签、角点坐标和颜色信息，供可视化使用
def object_to_rectangle(semantic_object: SemanticObject) -> Rectangle2D:
    """Convert a fused semantic object into a rectangle polyline for map overlays."""

    semantic_class = DEFAULT_SEMANTIC_CLASSES.get(semantic_object.label)
    color = semantic_class.color_rgba if semantic_class else (1.0, 1.0, 1.0, 0.85)
    return Rectangle2D(
        label=semantic_object.display_label,
        points=rectangle_corners(
            center_x=semantic_object.center_x,
            center_y=semantic_object.center_y,
            size_x=semantic_object.size_x,
            size_y=semantic_object.size_y,
            yaw=semantic_object.yaw,
        ),
        color_rgba=color,
    )
