"""Project 2D detector outputs into map-frame semantic rectangles."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import atan2, cos, hypot, sin
from statistics import median
from typing import Literal, Sequence

from semantic_map_pkg.geometry import CameraIntrinsics, Transform3D, pixel_to_camera


DepthImage = Sequence[Sequence[float | int | None]]

#检测框2D检测结果的数据容器，包含标签、置信度、中心点像素坐标、宽高等信息
@dataclass(frozen=True)
class Detection2D:
    """A detector result in image coordinates."""

    label: str
    confidence: float
    center_u: float
    center_v: float
    width: float
    height: float
    track_id: str | None = None


@dataclass(frozen=True)
class SemanticMapSeed:
    """A visual semantic hint that is already expressed near the map frame."""

    label: str
    confidence: float
    gx: float
    gy: float
    track_id: str | None = None


@dataclass(frozen=True)
class OccupancyGrid2D:
    """A ROS-like 2D occupancy grid without depending on ROS message classes."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: Sequence[int]
    occupied_threshold: int = 50

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("width and height must be positive")
        if self.resolution <= 0.0:
            raise ValueError("resolution must be positive")
        if len(self.data) != self.width * self.height:
            raise ValueError("data length must equal width * height")

    def world_to_grid(self, x: float, y: float) -> tuple[int, int] | None:
        col = int((x - self.origin_x) // self.resolution)
        row = int((y - self.origin_y) // self.resolution)
        if not self.contains(col, row):
            return None
        return col, row

    def grid_to_world_center(self, col: int, row: int) -> tuple[float, float]:
        return (
            self.origin_x + (col + 0.5) * self.resolution,
            self.origin_y + (row + 0.5) * self.resolution,
        )

    def contains(self, col: int, row: int) -> bool:
        return 0 <= col < self.width and 0 <= row < self.height

    def is_occupied(self, col: int, row: int) -> bool:
        if not self.contains(col, row):
            return False
        value = int(self.data[row * self.width + col])
        return value >= self.occupied_threshold


@dataclass(frozen=True)
class OccupancyIsland:
    """A connected occupied component extracted around a semantic seed."""

    cells: tuple[tuple[int, int], ...]
    centroid_x: float
    centroid_y: float
    total_pixels: int
    aspect_ratio: float
    size_x: float
    size_y: float


#语义矩形投影后的数据容器，包含标签、置信度、中心点地图坐标、尺寸等信息
#size_x和size_y分别表示矩形沿x轴和y轴的尺寸，单位米,先拿先验信息测试
#新增points_xy字段，包含投影过程中使用的所有有效深度点在地图坐标系中的位置，
# 可以用于后续的融合和跟踪算法
@dataclass(frozen=True)
class ProjectedObject:
    """A semantic object rectangle already expressed in the map frame."""

    label: str
    display_label: str
    confidence: float
    center_x: float
    center_y: float
    yaw: float
    size_x: float
    size_y: float
    source_frame: str
    points_xy: tuple[tuple[float, float], ...] = ()
    track_id: str | None = None
    total_pixels: int | None = None
    aspect_ratio: float | None = None


class SinglePointOccupancyProjector:
    """Snap a semantic map seed to the lidar occupancy island nearest to it."""

    def __init__(
        self,
        search_radius_m: float = 0.8,
        min_confidence: float = 0.35,
        min_total_pixels: int = 1,
        max_aspect_ratio: float = 12.0,
        connectivity: Literal[4, 8] = 8,
        max_points_per_island: int = 1200,
    ) -> None:
        if search_radius_m <= 0.0:
            raise ValueError("search_radius_m must be positive")
        if min_total_pixels < 1:
            raise ValueError("min_total_pixels must be at least 1")
        if max_aspect_ratio < 1.0:
            raise ValueError("max_aspect_ratio must be >= 1")
        if connectivity not in (4, 8):
            raise ValueError("connectivity must be 4 or 8")
        if max_points_per_island < 1:
            raise ValueError("max_points_per_island must be positive")
        self.search_radius_m = search_radius_m
        self.min_confidence = min_confidence
        self.min_total_pixels = min_total_pixels
        self.max_aspect_ratio = max_aspect_ratio
        self.connectivity = connectivity
        self.max_points_per_island = max_points_per_island

    def project_seed(
        self,
        seed: SemanticMapSeed,
        occupancy_grid: OccupancyGrid2D,
    ) -> ProjectedObject | None:
        """Project one map-frame semantic seed by snapping it to an island."""

        if seed.confidence < self.min_confidence:
            return None

        start_cell = self._find_nearest_occupied_cell(seed, occupancy_grid)
        if start_cell is None:
            return None

        island = self._flood_fill_island(start_cell, occupancy_grid)
        if not self._passes_geometry_check(island):
            return None

        canonical_label, display_label = _normalize_label(seed.label)
        return ProjectedObject(
            label=canonical_label,
            display_label=display_label,
            confidence=float(seed.confidence),
            center_x=island.centroid_x,
            center_y=island.centroid_y,
            yaw=0.0,
            size_x=island.size_x,
            size_y=island.size_y,
            source_frame="map",
            points_xy=self._island_points_xy(island, occupancy_grid),
            track_id=seed.track_id,
            total_pixels=island.total_pixels,
            aspect_ratio=island.aspect_ratio,
        )

    def project_seeds(
        self,
        seeds: Sequence[SemanticMapSeed],
        occupancy_grid: OccupancyGrid2D,
    ) -> list[ProjectedObject]:
        """Project all usable semantic seeds into snapped map objects."""

        projected: list[ProjectedObject] = []
        for seed in seeds:
            item = self.project_seed(seed, occupancy_grid)
            if item is not None:
                projected.append(item)
        return projected

    def _find_nearest_occupied_cell(
        self,
        seed: SemanticMapSeed,
        occupancy_grid: OccupancyGrid2D,
    ) -> tuple[int, int] | None:
        seed_cell = occupancy_grid.world_to_grid(seed.gx, seed.gy)
        if seed_cell is None:
            return None

        radius_cells = int(self.search_radius_m / occupancy_grid.resolution) + 1
        seed_col, seed_row = seed_cell
        best_cell: tuple[int, int] | None = None
        best_distance = self.search_radius_m

        for row in range(seed_row - radius_cells, seed_row + radius_cells + 1):
            for col in range(seed_col - radius_cells, seed_col + radius_cells + 1):
                if not occupancy_grid.is_occupied(col, row):
                    continue
                cell_x, cell_y = occupancy_grid.grid_to_world_center(col, row)
                distance = hypot(cell_x - seed.gx, cell_y - seed.gy)
                if distance <= best_distance:
                    best_cell = (col, row)
                    best_distance = distance
        return best_cell

    def _flood_fill_island(
        self,
        start_cell: tuple[int, int],
        occupancy_grid: OccupancyGrid2D,
    ) -> OccupancyIsland:
        visited: set[tuple[int, int]] = set()
        queue: deque[tuple[int, int]] = deque([start_cell])
        cells: list[tuple[int, int]] = []

        while queue:
            col, row = queue.popleft()
            if (col, row) in visited or not occupancy_grid.is_occupied(col, row):
                continue
            visited.add((col, row))
            cells.append((col, row))
            for next_cell in self._neighbors(col, row):
                if next_cell not in visited:
                    queue.append(next_cell)

        return self._build_island(cells, occupancy_grid)

    def _neighbors(self, col: int, row: int) -> tuple[tuple[int, int], ...]:
        cells = (
            (col - 1, row),
            (col + 1, row),
            (col, row - 1),
            (col, row + 1),
        )
        if self.connectivity == 4:
            return cells
        return cells + (
            (col - 1, row - 1),
            (col - 1, row + 1),
            (col + 1, row - 1),
            (col + 1, row + 1),
        )

    def _build_island(
        self,
        cells: list[tuple[int, int]],
        occupancy_grid: OccupancyGrid2D,
    ) -> OccupancyIsland:
        world_points = [occupancy_grid.grid_to_world_center(col, row) for col, row in cells]
        centroid_x = sum(x for x, _ in world_points) / len(world_points)
        centroid_y = sum(y for _, y in world_points) / len(world_points)

        cols = [col for col, _ in cells]
        rows = [row for _, row in cells]
        width_cells = max(cols) - min(cols) + 1
        height_cells = max(rows) - min(rows) + 1
        short_side = max(1, min(width_cells, height_cells))
        long_side = max(width_cells, height_cells)
        aspect_ratio = long_side / short_side

        return OccupancyIsland(
            cells=tuple(cells),
            centroid_x=centroid_x,
            centroid_y=centroid_y,
            total_pixels=len(cells),
            aspect_ratio=aspect_ratio,
            size_x=width_cells * occupancy_grid.resolution,
            size_y=height_cells * occupancy_grid.resolution,
        )

    def _passes_geometry_check(self, island: OccupancyIsland) -> bool:
        if island.total_pixels < self.min_total_pixels:
            return False
        return island.aspect_ratio <= self.max_aspect_ratio

    def _island_points_xy(
        self,
        island: OccupancyIsland,
        occupancy_grid: OccupancyGrid2D,
    ) -> tuple[tuple[float, float], ...]:
        cells = island.cells
        if len(cells) > self.max_points_per_island:
            step = len(cells) / self.max_points_per_island
            cells = tuple(cells[int(step * i)] for i in range(self.max_points_per_island))
        return tuple(occupancy_grid.grid_to_world_center(col, row) for col, row in cells)


#语义投影
class SemanticProjector:
    """Convert RGB-D detections into 2D semantic map objects."""

    def __init__(
        self,
        intrinsics: CameraIntrinsics,#相机参数
        depth_window_px: int = 5,#在计算检测框中心点的深度时，使用一个窗口内的像素值的中位数来提高鲁棒性，窗口大小由depth_window_px参数控制，默认5像素
        min_confidence: float = 0.35,
        size_mode: Literal["prior_size", "observed_size"] = "observed_size",
        min_observed_size_m: float = 0.2,
        max_observed_size_m: float = 4.0,
    ) -> None:
        if depth_window_px < 1:
            raise ValueError("depth_window_px must be at least 1")
        if size_mode not in ("prior_size", "observed_size"):
            raise ValueError("size_mode must be 'prior_size' or 'observed_size'")
        if min_observed_size_m <= 0.0:
            raise ValueError("min_observed_size_m must be positive")
        if max_observed_size_m < min_observed_size_m:
            raise ValueError("max_observed_size_m must be >= min_observed_size_m")
        self.intrinsics = intrinsics
        self.depth_window_px = depth_window_px
        self.min_confidence = min_confidence
        self.size_mode = size_mode
        self.min_observed_size_m = min_observed_size_m
        self.max_observed_size_m = max_observed_size_m

    def project_detection(
        self,
        detection: Detection2D,
        depth_image_m: DepthImage,#深度图
        camera_to_map: Transform3D,#变换矩阵
    ) -> ProjectedObject | None:
        """Project a single detection into the map frame.

        Returns None when the class is unsupported, confidence is too low, or no
        valid depth exists near the detection center.
        """

        if detection.confidence < self.min_confidence:
            return None

        canonical_label, display_label = _normalize_label(detection.label)

        # Main path: sample valid depth points inside the bbox and project all of
        # them to map frame. This gives us a robust observed center/size/yaw.
        points_xy = tuple(self._collect_bbox_points_xy(detection, depth_image_m, camera_to_map))
        observed = self._estimate_observed_bbox_and_yaw(points_xy)

        if self.size_mode == "observed_size" and observed is not None:
            map_x, map_y, size_x, size_y, yaw = observed
        else:
            # Fallback path: when bbox depth is too sparse, keep the target alive
            # by using center-pixel median depth projection.
            depth = median_depth_around(
                depth_image_m,
                center_u=detection.center_u,
                center_v=detection.center_v,
                window_px=self.depth_window_px,
            )
            if depth is None:
                return None
            camera_point = pixel_to_camera(
                detection.center_u,
                detection.center_v,
                depth,
                self.intrinsics,
            )
            map_x, map_y, _ = camera_to_map.apply(camera_point)
            size_x, size_y = self._estimate_fallback_size(detection, depth)
            yaw = 0.0

        return ProjectedObject(
            label=canonical_label,
            display_label=display_label,
            confidence=float(detection.confidence),
            center_x=map_x,
            center_y=map_y,
            yaw=yaw,
            size_x=size_x,
            size_y=size_y,
            source_frame=camera_to_map.source_frame,
            points_xy=points_xy,
            track_id=detection.track_id,
        )#返回语义矩阵投影结果，包含标签、置信度、中心点地图坐标、尺寸等信息

    def project_detections(
        self,
        detections: Sequence[Detection2D],
        depth_image_m: DepthImage,
        camera_to_map: Transform3D,
    ) -> list[ProjectedObject]:
        """Project all usable detections into the map frame."""

        projected: list[ProjectedObject] = []
        for detection in detections:
            item = self.project_detection(detection, depth_image_m, camera_to_map)
            if item is not None:
                projected.append(item)
        return projected

    def _estimate_fallback_size(
        self,
        detection: Detection2D,
        depth_m: float,
    ) -> tuple[float, float]:
        observed_x = abs(float(detection.width)) * depth_m / self.intrinsics.fx
        observed_y = abs(float(detection.height)) * depth_m / self.intrinsics.fy

        observed_x = min(self.max_observed_size_m, max(self.min_observed_size_m, observed_x))
        observed_y = min(self.max_observed_size_m, max(self.min_observed_size_m, observed_y))
        return observed_x, observed_y
    #估计物体的中心、尺寸和朝向
    
    def _estimate_observed_bbox_and_yaw(
        self,
        points_xy: tuple[tuple[float, float], ...],
    ) -> tuple[float, float, float, float, float] | None:
        # Need at least 3 points for stable covariance-based orientation.
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

        # 2D PCA principal axis is used as object yaw.
        yaw = 0.5 * atan2(2.0 * cov_xy, cov_xx - cov_yy)
        axis_x = (cos(yaw), sin(yaw))
        axis_y = (-sin(yaw), cos(yaw))

        proj_x = [x * axis_x[0] + y * axis_x[1] for x, y in centered]
        proj_y = [x * axis_y[0] + y * axis_y[1] for x, y in centered]
        size_x = max(proj_x) - min(proj_x)
        size_y = max(proj_y) - min(proj_y)
        size_x = min(self.max_observed_size_m, max(self.min_observed_size_m, size_x))
        size_y = min(self.max_observed_size_m, max(self.min_observed_size_m, size_y))
        return mean_x, mean_y, size_x, size_y, yaw

    def _collect_bbox_points_xy(
        self,
        detection: Detection2D,
        depth_image_m: DepthImage,
        camera_to_map: Transform3D,
    ) -> list[tuple[float, float]]:
        if not depth_image_m:
            return []
        height = len(depth_image_m)
        width = len(depth_image_m[0]) if height else 0
        if width == 0:
            return []

        u0 = max(0, int(round(detection.center_u - detection.width / 2.0)))
        u1 = min(width - 1, int(round(detection.center_u + detection.width / 2.0)))
        v0 = max(0, int(round(detection.center_v - detection.height / 2.0)))
        v1 = min(height - 1, int(round(detection.center_v + detection.height / 2.0)))
        if u1 < u0 or v1 < v0:
            return []

        # Adaptive sub-sampling keeps runtime bounded on large boxes.
        area = max(1, (u1 - u0 + 1) * (v1 - v0 + 1))
        stride = max(1, int((area / 400) ** 0.5))
        points_xy: list[tuple[float, float]] = []
        for v in range(v0, v1 + 1, stride):
            row = depth_image_m[v]
            for u in range(u0, u1 + 1, stride):
                value = row[u]
                if value is None:
                    continue
                depth = float(value)
                if depth <= 0.0:
                    continue
                cam = pixel_to_camera(float(u), float(v), depth, self.intrinsics)
                map_x, map_y, _ = camera_to_map.apply(cam)
                points_xy.append((map_x, map_y))
        return points_xy

#收集窗口内所有有效的正深度值 返回这些值的中值
#observe出问题使用
def median_depth_around(
    depth_image_m: DepthImage,
    center_u: float,
    center_v: float,
    window_px: int,
) -> float | None:
    """Return median positive depth around a pixel center."""

    if not depth_image_m:
        return None

    height = len(depth_image_m)
    width = len(depth_image_m[0]) if height else 0
    if width == 0:
        return None

    half = window_px // 2
    center_u_px = min(width - 1, max(0, int(round(center_u))))
    center_v_px = min(height - 1, max(0, int(round(center_v))))
    u0 = max(0, center_u_px - half)
    u1 = min(width - 1, center_u_px + half)
    v0 = max(0, center_v_px - half)
    v1 = min(height - 1, center_v_px + half)

    values: list[float] = []
    for v in range(v0, v1 + 1):
        row = depth_image_m[v]
        for u in range(u0, u1 + 1):
            value = row[u]
            if value is None:
                continue
            depth = float(value)
            if depth > 0.0:
                values.append(depth)

    if not values:
        return None
    return float(median(values))


def _normalize_label(raw_label: str) -> tuple[str, str]:
    display_label = raw_label.strip() or "unknown"
    canonical_label = display_label.lower().replace(" ", "_")
    return canonical_label, display_label
