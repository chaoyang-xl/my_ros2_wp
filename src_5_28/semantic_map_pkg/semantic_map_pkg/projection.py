"""Project 2D detector outputs into map-frame semantic rectangles."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, sin
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
