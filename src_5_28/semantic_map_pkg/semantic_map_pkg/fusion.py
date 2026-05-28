"""Multi-frame semantic object fusion and lifecycle management."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import atan2, cos, hypot, sin
from time import time
from typing import Iterable
from uuid import uuid4

from semantic_map_pkg.projection import ProjectedObject


@dataclass(frozen=True)
class SemanticObject:
    """A stable semantic map object after multi-frame fusion.数据容器"""

    id: str #唯一标识符，可以是跟踪ID或随机生成的ID
    label: str #语义类别标签，例如"chair"、"pedestrian"
    display_label: str #用于可视化的标签，可能包含别名或更友好的名称
    confidence: float #置信度分数，范围[0.0, 1.0]
    center_x: float #对象在地图坐标系中的中心点x坐标，单位米
    center_y: float # 对象在地图坐标系中的中心点y坐标，单位米
    yaw: float #对象的朝向，单位弧度，范围[-pi, pi]
    size_x: float #对象沿x轴的尺寸，单位米
    size_y: float #对象沿y轴的尺寸，单位米
    source_frame: str #对象最初被观测到时的坐标系，例如"camera_link"
    points_xy: tuple[tuple[float, float], ...]
    first_seen: float #对象第一次被观测到的时间戳，单位秒
    last_seen: float #对象最后一次被观测到的时间戳，单位秒
    observations: int #对象被观测到的次数，初始值为1，每次匹配到新的投影对象时递增

    def distance_to(self, projected: ProjectedObject) -> float:
        #计算当前语义对象与一个投影对象之间的欧氏距离，单位米
        return hypot(self.center_x - projected.center_x, self.center_y - projected.center_y)


class SemanticObjectTracker:
    """Fuse projected objects into stable map-frame semantic objects."""

    def __init__(
        self,
        association_distance_m: float = 0.8,#关联距离，单位米，超过这个距离的投影对象将不会与现有语义对象匹配
        smoothing_alpha: float = 0.35,
        max_age_s: float = 30.0,
        max_points_per_object: int = 1200,#cpu偏高调到600
    ) -> None:
        if association_distance_m <= 0.0:
            raise ValueError("association_distance_m must be positive")
        if not 0.0 < smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in (0, 1]")
        if max_age_s <= 0.0:
            raise ValueError("max_age_s must be positive")
        if max_points_per_object < 16:
            raise ValueError("max_points_per_object must be at least 16")
        self.association_distance_m = association_distance_m
        self.smoothing_alpha = smoothing_alpha
        self.max_age_s = max_age_s
        self.max_points_per_object = max_points_per_object
        self._objects: dict[str, SemanticObject] = {}#物体字典，键为对象ID，值为SemanticObject实例

    @property #当前跟踪的语义对象列表，按最后一次观测时间排序，最近的在前面
    def objects(self) -> tuple[SemanticObject, ...]:
        """Current tracked objects sorted by last update time."""

        return tuple(sorted(self._objects.values(), key=lambda item: item.last_seen, reverse=True))

    def update(
        self,
        projected_objects: Iterable[ProjectedObject],
        now: float | None = None,
    ) -> tuple[SemanticObject, ...]:
        """Associate a batch of projected detections and return live objects."""

        timestamp = time() if now is None else float(now)
        for projected in projected_objects:
            match = self._find_match(projected) #在当前语义对象中寻找与投影对象匹配的对象，返回匹配的SemanticObject实例或None
            if match is None:
                self._add(projected, timestamp)
            else:
                self._merge(match, projected, timestamp)#融合
        self.prune(now=timestamp)#移除过期对象
        return self.objects

    def prune(self, now: float | None = None) -> None:
        """Remove objects that have not been observed within max_age_s."""
        #移除长时间未被观测到的对象，过期时间由max_age_s参数控制，默认30秒
        timestamp = time() if now is None else float(now)
        expired = [
            object_id
            for object_id, semantic_object in self._objects.items()
            if timestamp - semantic_object.last_seen > self.max_age_s
        ]
        for object_id in expired:
            del self._objects[object_id]
        
    def query(self, label_or_alias: str) -> tuple[SemanticObject, ...]:
        """Return objects whose canonical or display label matches the query text."""
        #查询函数，先保留
        text = label_or_alias.strip().lower()
        return tuple(
            item
            for item in self.objects
            if item.label.lower() == text or item.display_label.lower() == text
        )

    def _find_match(self, projected: ProjectedObject) -> SemanticObject | None:
        candidates = [
            item
            for item in self._objects.values()
            if item.label == projected.label and item.distance_to(projected) <= self.association_distance_m
        ]#寻找匹配的候选对象，要求语义标签相同且距离在关联距离范围内
        if not candidates:
            return None
        return min(candidates, key=lambda item: item.distance_to(projected))#从候选对象中选择距离投影对象最近的作为匹配对象

    def _add(self, projected: ProjectedObject, timestamp: float) -> None:
        #为一个新的投影对象创建一个新的语义对象，并添加到跟踪列表中，生成唯一ID，初始观测次数为1
        #优先使用追踪id，如果没有追踪id，则使用标签加随机字符串的方式生成一个唯一ID
        object_id = projected.track_id or f"{projected.label}-{uuid4().hex[:8]}"
        self._objects[object_id] = SemanticObject(
            id=object_id,
            label=projected.label,
            display_label=projected.display_label,
            confidence=projected.confidence,
            center_x=projected.center_x,
            center_y=projected.center_y,
            yaw=projected.yaw,
            size_x=projected.size_x,
            size_y=projected.size_y,
            source_frame=projected.source_frame,
            points_xy=self._tail_points(projected.points_xy),
            first_seen=timestamp,
            last_seen=timestamp,
            observations=1,
        )

    def _merge(self, match: SemanticObject, projected: ProjectedObject, timestamp: float) -> None:
        # Merge both state and local point samples.
        alpha = self.smoothing_alpha
        beta = 1.0 - alpha
        merged_points = self._merge_points(match.points_xy, projected.points_xy)
        observed_geometry = self._estimate_geometry_from_points(merged_points)

        if observed_geometry is not None:
            # Prefer geometry re-estimated from merged points, then smooth.
            observed_center_x, observed_center_y, observed_size_x, observed_size_y, observed_yaw = observed_geometry
            center_x = beta * match.center_x + alpha * observed_center_x
            center_y = beta * match.center_y + alpha * observed_center_y
            yaw = _blend_angle(match.yaw, observed_yaw, alpha)
            size_x = beta * match.size_x + alpha * observed_size_x
            size_y = beta * match.size_y + alpha * observed_size_y
        else:
            # If point-based geometry is unavailable, fallback to projected state.
            center_x = beta * match.center_x + alpha * projected.center_x
            center_y = beta * match.center_y + alpha * projected.center_y
            yaw = _blend_angle(match.yaw, projected.yaw, alpha)
            size_x = beta * match.size_x + alpha * projected.size_x
            size_y = beta * match.size_y + alpha * projected.size_y

        self._objects[match.id] = replace(
            match,
            confidence=max(match.confidence, projected.confidence),#取较高的置信度
            center_x=center_x,
            center_y=center_y,
            yaw=yaw,
            size_x=size_x,#尺寸也进行平滑融合，避免因为单帧检测的尺寸误差导致语义对象尺寸的剧烈变化
            size_y=size_y,#尺寸也进行平滑融合，避免因为单帧检测的尺寸误差导致语义对象尺寸的剧烈变化
            source_frame=projected.source_frame,
            points_xy=merged_points,
            last_seen=timestamp,
            observations=match.observations + 1,
        )
    #
    def _tail_points(self, points: tuple[tuple[float, float], ...]) -> tuple[tuple[float, float], ...]:
        """Return a uniformly sampled version of points, capped at max_points_per_object."""
        n = len(points)
        max_pts = self.max_points_per_object
        if n <= max_pts:
            return points

        # 均匀采样：计算步长，选取索引
        step = n / max_pts
        indices = [int(step * i) for i in range(max_pts)]
        # 确保最后一个索引不越界（理论不会，因为 step*(max_pts-1) <= n-1）
        return tuple(points[i] for i in indices)

    def _merge_points(
        self,
        old_points: tuple[tuple[float, float], ...],
        new_points: tuple[tuple[float, float], ...],
    ) -> tuple[tuple[float, float], ...]:
        if not new_points:
            return old_points
        return self._tail_points(old_points + new_points)

    #计算几何信息
    def _estimate_geometry_from_points(
        self,
        points_xy: tuple[tuple[float, float], ...],
    ) -> tuple[float, float, float, float, float] | None:
        if len(points_xy) < 3:
            return None

        mean_x = sum(p[0] for p in points_xy) / len(points_xy)
        mean_y = sum(p[1] for p in points_xy) / len(points_xy)
        centered = [(x - mean_x, y - mean_y) for x, y in points_xy]

        cov_xx = sum(x * x for x, _ in centered) / len(centered)
        cov_yy = sum(y * y for _, y in centered) / len(centered)
        cov_xy = sum(x * y for x, y in centered) / len(centered)
        if cov_xx == 0.0 and cov_yy == 0.0:
            return None

        yaw = 0.5 * atan2(2.0 * cov_xy, cov_xx - cov_yy)
        axis_x = (cos(yaw), sin(yaw))
        axis_y = (-sin(yaw), cos(yaw))
        proj_x = [x * axis_x[0] + y * axis_x[1] for x, y in centered]
        proj_y = [x * axis_y[0] + y * axis_y[1] for x, y in centered]
        size_x = max(proj_x) - min(proj_x)
        size_y = max(proj_y) - min(proj_y)
        size_x = max(0.2, min(4.0, size_x))
        size_y = max(0.2, min(4.0, size_y))
        return mean_x, mean_y, size_x, size_y, yaw

#对两个角度进行平滑混合（指数移动平均），同时避免角度在 ±π 边界处的跳变问题
def _blend_angle(base: float, target: float, alpha: float) -> float:
    # Blend on the unit circle to avoid wrap-around jumps at +/-pi.
    sin_mix = (1.0 - alpha) * sin(base) + alpha * sin(target)
    cos_mix = (1.0 - alpha) * cos(base) + alpha * cos(target)
    return atan2(sin_mix, cos_mix)
