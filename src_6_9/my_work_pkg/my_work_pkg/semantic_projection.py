"""Single-point semantic projection against an occupancy grid map.

This module deliberately keeps the projection algorithm independent from ROS.
The ROS node only adapts messages into these classes; all important logic lives
here so it is easy to test and explain.
gx,gy为/【map】坐标系下的语义种子位置，单位为米
逻辑是：收到种子 -> 找栅格吸附 -> 扔进记忆池（匹配、平滑或新建）；
同时后台开一个定时器，周期性地把记忆池里存活的对象作为 Marker 发布出来。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import atan2, cos, hypot, sin
from typing import Literal, Sequence
from math import hypot
import numpy as np
import math
@dataclass(frozen=True)
class SnappedObject:
    """
       在测试节点中,从SemanticSeed通过project方法得到ProjectedSemanticObject类型
       直接通过后续的追踪进行update,update方法使用的是此数据结构，但在节点中使用的是
       ProjectedSemanticObject结构，字段兼容，
       可能后续维护会出现bug,即后续的追踪使用的是SnappedObject
       和TrackedObject类型，保存和后续的marker数据发布使用的也是TrackedObject
       这两种都没有追踪id，后续如需要可增加

    """
    label: str
    confidence: float
    center_x: float
    center_y: float
    size_x: float
    size_y: float

@dataclass(frozen=True)
class SemanticSeed:#从视觉系统来的语义种子，坐标是map下的坐标
    """A visual semantic seed already expressed in the map frame.

    The visual system provides a rough point: "near (gx, gy), I saw a bench".
    The projector will not trust this point as the final object center.  It uses
    it only as a seed to search the lidar-built occupancy grid.
    """

    label: str
    confidence: float
    gx: float
    gy: float
    track_id: str | None = None


@dataclass(frozen=True)
class OccupancyGridMap:
    """A small ROS-like occupancy grid wrapper.

    ``data`` follows the usual ROS OccupancyGrid layout: row-major values where
    ``-1`` is unknown, ``0`` is free, and high positive values are occupied.
    """

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: Sequence[int]
    occupied_threshold: int = 50

    def __post_init__(self) -> None:
        """Validate the grid shape once, before projection starts."""
        if self.width <= 0 or self.height <= 0:
            raise ValueError("width and height must be positive")
        if self.resolution <= 0.0:
            raise ValueError("resolution must be positive")
        if len(self.data) != self.width * self.height:
            raise ValueError("data length must equal width * height")

    def world_to_grid(self, wx: float, wy: float) -> tuple[int, int] | None:
        """Convert map-frame meters into integer grid coordinates."""
        gx = math.floor((wx - self.origin_x) / self.resolution)
        gy = math.floor((wy - self.origin_y) / self.resolution)
        if not self.contains(gx, gy):
            return None
        return gx, gy

    def grid_to_world_center(self, gx: int, gy: int) -> tuple[float, float]:
        """Return the map-frame center point of a grid cell."""
        return (
            self.origin_x + (gx + 0.5) * self.resolution,
            self.origin_y + (gy + 0.5) * self.resolution,
        )

    def contains(self, gx: int, gy: int) -> bool:
        """Return whether a grid coordinate lies inside the map."""
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_occupied(self, gx: int, gy: int) -> bool:
        """Return whether a cell should be treated as lidar obstacle mass."""
        if not self.contains(gx, gy):
            return False
        value = int(self.data[gy * self.width + gx])
        return value >= self.occupied_threshold


@dataclass(frozen=True)
class OccupancyIsland:
    """One connected occupied component found by flood-fill."""
    '''洪水填充找到的一个占用岛屿，包含组成岛屿的像素坐标和计算出的几何属性'''
    cells: tuple[tuple[int, int], ...]
    centroid_x: float
    centroid_y: float
    total_pixels: int
    aspect_ratio: float
    size_x: float
    size_y: float
    yaw: float


@dataclass(frozen=True)
class ProjectedSemanticObject:
    """The final snapped semantic object in the map frame."""
    '''最终的投影结果：一个成功吸附到占用岛屿并计算出几何属性的语义对象,在project方法中
       把OccupancyIsland转换到了ProjectedSemanticObject
    
    '''
    label: str
    confidence: float
    center_x: float#岛屿质心的坐标 map下
    center_y: float
    size_x: float#岛屿尺寸
    size_y: float
    yaw: float#岛屿主轴方向，弧度制，0表示长边朝向x轴，正值逆时针
    total_pixels: int#占据岛屿的像素数量，反映了物体的大小和稠密程度
    aspect_ratio: float#占据岛屿的长宽比，定义为较长边除以较短边，反映了物体的形状特征
    source_frame: str = "map"
    track_id: str | None = None
    points_xy: tuple[tuple[float, float], ...] = ()#占据岛屿内的点坐标列表，单位为米，坐标系同map。为了可视化和后续融合，投影结果会附带一些占据岛屿的点坐标，但为了避免过大消息体，这个字段会限制最大点数并均匀采样。


class SinglePointSemanticProjector:
    """Project one semantic seed by snapping it to an occupancy island."""

    def __init__(
        self,
        search_radius_m: float = 0.8,#在种子周围搜索最近占用像素的半径（米）。种子可能不精确，在此半径内找到最近的障碍物栅格作为洪水填充起点。
        min_confidence: float = 0.35,#投影的最低置信度。只有当视觉种子的置信度超过此值时才尝试投影，否则直接丢弃。
        min_total_pixels: int = 2,#一个占用岛屿的最小像素数。投影后占用岛屿的像素数必须至少达到此值，否则被认为是噪声而被丢弃。
        max_aspect_ratio: float = 12.0,#一个占用岛屿的最大长宽比。投影后占用岛屿的长宽比（较长边除以较短边）必须不超过此值，否则被认为是过于细长的墙段而被丢弃。
        connectivity: Literal[4, 8] = 4,#洪水填充的连接方式。4-连通只考虑上下左右四个邻居，8-连通还包括四个对角线邻居。8-连通更宽松，可能会将稍微斜着的墙段视为一个岛屿，而4-连通更严格，可能会将它们分成两个岛屿。
        wall_margin_m: float = 0.2,#墙边距（米）。如果大于0，则投影后占用岛屿的平均网格坐标必须距离地图边界至少此距离，否则被认为是贴边的墙段而被丢弃。这个规则模仿了参考项目中对贴边岛屿的排除，但增加了配置灵活性。
        max_points_per_object: int = 1200,#每个投影对象的最大点数。为了可视化和后续融合，投影结果会附带一些占用岛屿的点坐标，但为了避免过大消息体，这个参数限制了每个对象最多包含多少个点。超过时会均匀采样。
    ) -> None:
        """Create the projector.

        ``search_radius_m`` controls how far from the visual seed we search for
        the first occupied lidar cell.  ``min_total_pixels`` and
        ``max_aspect_ratio`` are the basic geometry gate for rejecting tiny noise
        and long wall-like components.
        """
        if search_radius_m <= 0.0:
            raise ValueError("search_radius_m must be positive")
        if min_total_pixels < 1:
            raise ValueError("min_total_pixels must be at least 1")
        if max_aspect_ratio < 1.0:
            raise ValueError("max_aspect_ratio must be >= 1")
        if connectivity not in (4, 8):
            raise ValueError("connectivity must be 4 or 8")
        if wall_margin_m < 0.0:
            raise ValueError("wall_margin_m must be >= 0")
        if max_points_per_object < 1:
            raise ValueError("max_points_per_object must be positive")

        self.search_radius_m = search_radius_m
        self.min_confidence = min_confidence
        self.min_total_pixels = min_total_pixels
        self.max_aspect_ratio = max_aspect_ratio
        self.connectivity = connectivity
        self.wall_margin_m = wall_margin_m
        self.max_points_per_object = max_points_per_object

    def project(
        self,
        seed: SemanticSeed,
        grid: OccupancyGridMap,
    ) -> ProjectedSemanticObject | None:
        """Snap a single visual seed to the nearest valid occupancy island.

        The four steps mirror the intended mechanism:
        1. Treat ``(gx, gy)`` as a rough semantic seed.
        2. Search nearby occupied cells and flood-fill the connected island.
        3. Compute physical geometry from the island pixels.
        4. Replace the rough visual center with the island centroid.
        """
        if seed.confidence < self.min_confidence:#置信度校验
            return None

        start_cell = self._find_nearest_occupied_cell(seed, grid)#找到最近的占用像素
        if start_cell is None:
            return None

        island = self._flood_fill_island(start_cell, grid)#BFS洪水填充找到占用岛屿
        if not self._passes_geometry_check(island, grid):
            return None

        return ProjectedSemanticObject(
            label=_normalize_label(seed.label),
            confidence=float(seed.confidence),
            center_x=island.centroid_x,
            center_y=island.centroid_y,
            size_x=island.size_x,
            size_y=island.size_y,
            yaw=island.yaw,
            total_pixels=island.total_pixels,
            aspect_ratio=island.aspect_ratio,
            track_id=seed.track_id,
            points_xy=self._sample_island_points(island, grid),
        )

    def project_many(
        self,
        seeds: Sequence[SemanticSeed],
        grid: OccupancyGridMap,
    ) -> list[ProjectedSemanticObject]:
        """Project all usable seeds and drop those that fail validation."""
        objects: list[ProjectedSemanticObject] = []
        for seed in seeds:
            projected = self.project(seed, grid)
            if projected is not None:
                objects.append(projected)
        return objects
    #搜索search_radius_m半径下的最近占用像素，欧氏距离
    def _find_nearest_occupied_cell(
        self,
        seed: SemanticSeed,
        grid: OccupancyGridMap,
    ) -> tuple[int, int] | None:
        """Find the nearest occupied lidar cell around the visual seed.

        This is the "drop the seed" step: the visual center may be noisy, so we
        only use it to locate the closest physical obstacle pixel nearby.
        """
        seed_cell = grid.world_to_grid(seed.gx, seed.gy)
        if seed_cell is None:
            return None

        radius_cells = int(self.search_radius_m / grid.resolution) + 1
        seed_gx, seed_gy = seed_cell
        best_cell: tuple[int, int] | None = None
        best_distance = self.search_radius_m

        for gy in range(seed_gy - radius_cells, seed_gy + radius_cells + 1):
            for gx in range(seed_gx - radius_cells, seed_gx + radius_cells + 1):
                if not grid.is_occupied(gx, gy):
                    continue
                wx, wy = grid.grid_to_world_center(gx, gy)
                distance = hypot(wx - seed.gx, wy - seed.gy)
                if distance <= best_distance:
                    best_cell = (gx, gy)
                    best_distance = distance
        return best_cell
    #BFS洪水填充找到所有联通的占用像素，_build_island(cells, grid)计算属性转换格式
    def _flood_fill_island(
        self,
        start_cell: tuple[int, int],
        grid: OccupancyGridMap,
    ) -> OccupancyIsland:
        """Flood-fill every occupied cell connected to ``start_cell``.

        The output island is the lidar-observed physical entity.  Its pixel
        count, bounding box, aspect ratio, and centroid are the geometric facts
        used to correct the original visual seed.
        """
        visited: set[tuple[int, int]] = set()
        queue: deque[tuple[int, int]] = deque([start_cell])
        cells: list[tuple[int, int]] = []

        while queue:
            gx, gy = queue.popleft()
            if (gx, gy) in visited:
                continue
            visited.add((gx, gy))
            if not grid.is_occupied(gx, gy):
                continue
            cells.append((gx, gy))

            for next_cell in self._neighbors(gx, gy):
                if next_cell not in visited:
                    queue.append(next_cell)

        return self._build_island(cells, grid)

    def _neighbors(self, gx: int, gy: int) -> tuple[tuple[int, int], ...]:
        """Return 4- or 8-connected flood-fill neighbours."""
        cells = (
            (gx - 1, gy),
            (gx + 1, gy),
            (gx, gy - 1),
            (gx, gy + 1),
        )
        if self.connectivity == 4:
            return cells
        return cells + (
            (gx - 1, gy - 1),
            (gx - 1, gy + 1),
            (gx + 1, gy - 1),
            (gx + 1, gy + 1),
        )

    def _build_island(
        self,
        cells: list[tuple[int, int]],
        grid: OccupancyGridMap,
    ) -> OccupancyIsland:
        """Convert raw island cells into metric geometry."""
        if not cells:
            raise ValueError("cannot build an island from zero cells")

        points = [grid.grid_to_world_center(gx, gy) for gx, gy in cells]
        centroid_x = sum(x for x, _ in points) / len(points)
        centroid_y = sum(y for _, y in points) / len(points)

        xs = [gx for gx, _ in cells]
        ys = [gy for _, gy in cells]
        width_cells = max(xs) - min(xs) + 1
        height_cells = max(ys) - min(ys) + 1
        short_side = max(1, min(width_cells, height_cells))
        long_side = max(width_cells, height_cells)
        aspect_ratio = long_side / short_side

        yaw = _estimate_yaw(points, centroid_x, centroid_y)
        return OccupancyIsland(
            cells=tuple(cells),
            centroid_x=centroid_x,
            centroid_y=centroid_y,
            total_pixels=len(cells),
            aspect_ratio=aspect_ratio,
            size_x=width_cells * grid.resolution,
            size_y=height_cells * grid.resolution,
            yaw=yaw,
        )

    def _passes_geometry_check(
        self,
        island: OccupancyIsland,
        grid: OccupancyGridMap,
    ) -> bool:
        """Reject noise, extreme wall-like shapes, and optional map-border hits."""

        '''过滤器：
            若 total_pixels < min_total_pixels,拒绝（噪声）。
            若 aspect_ratio > max_aspect_ratio,拒绝（过于细长，可能是墙壁）。
            若 wall_margin_m > 0:
            计算岛屿的平均栅格坐标 avg_gx, avg_gy。
            要求 avg_gx 和 avg_gy 距离地图边界至少 margin_cells(max(1, int(wall_margin_m / resolution))）个像素。
            这避免了将紧贴地图边界的墙壁误认为是可移动物体。
        
        '''
        if island.total_pixels < self.min_total_pixels:
            return False
        if island.aspect_ratio > self.max_aspect_ratio:
            return False
        if self.wall_margin_m <= 0.0:
            return True

        # A centroid too close to the global map border is often a wall segment
        # rather than a movable object.  This mirrors the reference project's
        # wall-island rejection while keeping the rule configurable.
        margin_cells = max(1, int(self.wall_margin_m / grid.resolution))
        avg_gx = sum(gx for gx, _ in island.cells) / island.total_pixels
        avg_gy = sum(gy for _, gy in island.cells) / island.total_pixels
        return (
            margin_cells <= avg_gx < grid.width - margin_cells
            and margin_cells <= avg_gy < grid.height - margin_cells
        )

    def _sample_island_points(
        self,
        island: OccupancyIsland,
        grid: OccupancyGridMap,
    ) -> tuple[tuple[float, float], ...]:
        """Return island points for visualization/fusion without huge payloads.
           岛屿像素点下采样
        """
        cells = island.cells
        if len(cells) > self.max_points_per_object:
            step = len(cells) / self.max_points_per_object
            cells = tuple(cells[int(step * i)] for i in range(self.max_points_per_object))
        return tuple(grid.grid_to_world_center(gx, gy) for gx, gy in cells)


def _estimate_yaw(
    points: Sequence[tuple[float, float]],
    centroid_x: float,
    centroid_y: float,
) -> float:
    """Estimate island orientation with a simple 2D PCA principal axis."""
    if len(points) < 3:
        return 0.0

    centered = [(x - centroid_x, y - centroid_y) for x, y in points]
    cov_xx = sum(x * x for x, _ in centered) / len(centered)
    cov_yy = sum(y * y for _, y in centered) / len(centered)
    cov_xy = sum(x * y for x, y in centered) / len(centered)
    if cov_xx == 0.0 and cov_yy == 0.0:
        return 0.0

    yaw = 0.5 * atan2(2.0 * cov_xy, cov_xx - cov_yy)
    if _projected_extent(points, centroid_x, centroid_y, yaw) < _projected_extent(
        points,
        centroid_x,
        centroid_y,
        yaw + 1.5707963267948966,
    ):
        yaw += 1.5707963267948966
    return yaw


def _projected_extent(
    points: Sequence[tuple[float, float]],
    centroid_x: float,
    centroid_y: float,
    yaw: float,
) -> float:
    """Measure point spread along one candidate yaw axis."""
    axis = (cos(yaw), sin(yaw))
    projections = [
        (x - centroid_x) * axis[0] + (y - centroid_y) * axis[1]
        for x, y in points
    ]
    return max(projections) - min(projections)


def _normalize_label(label: str) -> str:
    """Normalize detector labels into stable semantic keys."""
    return (label.strip() or "unknown").lower().replace(" ", "_")#将标签转为小写、去空格、下划线替换空格，便于后续匹配



@dataclass
class TrackedObject:
    """在内存中持续跟踪的语义对象"""
    object_id: str#label + 序号，唯一标识一个对象，如 bench_0, bench_1
    label: str
    x: float
    y: float
    size_x: float
    size_y: float
    confidence: float
    times_seen: int
    last_seen: float

class SemanticMemory:
    """语义短时记忆：负责对象的匹配、EMA平滑与遗忘老化"""
    def __init__(self, match_distance: float = 1.0, alpha: float = 0.3, timeout: float = 10.0) -> None:
        self.match_distance = match_distance  # 匹配的欧氏距离阈值 (米)
        self.alpha = alpha                    # EMA 平滑系数 (0~1，越小越平滑但延迟越高)
        self.timeout = timeout                # 遗忘时间 (秒)
        self.objects: dict[str, TrackedObject] = {}
        self._next_id: dict[str, int] = {}    # 用于生成 ID，如 bench_0, bench_1

    def update(self, snapped: SnappedObject, current_time: float) -> TrackedObject:
        """接收一个新吸附的对象，尝试匹配并平滑，或者创建新对象"""
        best_match = None
        best_dist = float('inf')

        # 1. 遍历寻找同类别且距离最近的对象
        for obj in self.objects.values():
            if obj.label != snapped.label:
                continue
            d = hypot(obj.x - snapped.center_x, obj.y - snapped.center_y)
            if d < self.match_distance and d < best_dist:
                best_match = obj
                best_dist = d

        # 2. 如果找到匹配项，执行 EMA 平滑更新
        if best_match is not None:
            a = self.alpha
            # 平滑中心坐标
            best_match.x = a * snapped.center_x + (1.0 - a) * best_match.x
            best_match.y = a * snapped.center_y + (1.0 - a) * best_match.y
            # 平滑包围盒尺寸
            best_match.size_x = a * snapped.size_x + (1.0 - a) * best_match.size_x
            best_match.size_y = a * snapped.size_y + (1.0 - a) * best_match.size_y
            
            # 更新平均置信度与观测次数
            best_match.confidence = (best_match.confidence * best_match.times_seen + snapped.confidence) / (best_match.times_seen + 1)
            best_match.times_seen += 1
            best_match.last_seen = current_time
            return best_match

        # 3. 如果没找到，创建新对象
        else:
            seq = self._next_id.get(snapped.label, 0)# 为每个类别维护一个递增的序列号，生成唯一 ID
            self._next_id[snapped.label] = seq + 1
            oid = f"{snapped.label}_{seq}"
            
            new_obj = TrackedObject(
                object_id=oid,
                label=snapped.label,
                x=snapped.center_x,
                y=snapped.center_y,
                size_x=snapped.size_x,
                size_y=snapped.size_y,
                confidence=snapped.confidence,
                times_seen=1,
                last_seen=current_time
            )
            self.objects[oid] = new_obj
            return new_obj

    def age(self, current_time: float) -> None:
        """清理长时间未被观测到的对象"""
        stale_ids = [oid for oid, obj in self.objects.items() if current_time - obj.last_seen > self.timeout]
        for oid in stale_ids:
            del self.objects[oid]

    def get_active_objects(self) -> list[TrackedObject]:
        """获取当前存活的所有对象"""
        return list(self.objects.values())