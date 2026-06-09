"""Project YOLO 2D detections into rough map-frame semantic seeds.

The occupancy snap stage expects ``gx, gy`` to already be map-frame meters.
This module provides the previous stage: YOLO bbox center -> camera bearing ->
LiDAR range -> planar point in ``base_link``.  A ROS node then transforms that
point into ``map`` and publishes it as a semantic seed.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, isfinite, pi, sin
from typing import Sequence
import numpy as np

@dataclass(frozen=True)
class YoloDetection2D:
    """Minimal YOLO detection data needed for planar projection."""

    label: str
    confidence: float
    center_u: float
    center_v: float
    image_width: int
    image_height: int
    track_id: str | None = None


@dataclass(frozen=True)
class CameraFrameSeed:
    """A semantic seed in ``base_link`` before TF transforms it into ``map``."""
    '''表示在相机光学坐标系 (camera_optical_frame) 下的语义种子点'''
    label: str
    confidence: float
    x: float
    y: float
    z: float
    track_id: str | None = None


class YoloLidarSeedProjector:
    """使用深度图将检测框投影到3d  图像坐标到相机坐标系的投影器"""

    def __init__(
        self,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        window_size: int = 5,
        min_depth_m: float = 0.3,
        max_depth_m: float = 5.0,
    ) -> None:
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.window_half = window_size // 2
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
    def project(
        self,
        det: YoloDetection2D,
        depth_image: np.ndarray
    ) -> CameraFrameSeed | None:

        u = int(det.center_u)
        v = int(det.center_v)

        # 取一个小窗，提取中位数深度
        h,w = depth_image.shape
        u_min = max(0, u - self.window_half)
        u_max = min(w, u + self.window_half + 1)
        v_min = max(0, v - self.window_half)
        v_max = min(h, v + self.window_half + 1)

        depth_roi = depth_image[v_min:v_max, u_min:u_max]


        # 过滤无效值(0,NaN,inf)和超出范围的值
        valid_depths = depth_roi[
            np.isfinite(depth_roi) &
            (depth_roi >= self.min_depth_m) &
            (depth_roi <= self.max_depth_m)
        ]
        if valid_depths.size == 0:
            return None #深度无效
        median_depth = float(np.median(valid_depths))

        # 反投影到相机坐标系
        z3d = median_depth
        x3d = (u - self.cx) * z3d / self.fx
        y3d = (v - self.cy) * z3d / self.fy

        return CameraFrameSeed(
            label=det.label,
            confidence=float(det.confidence),
            x=x3d,
            y=y3d,
            z=z3d,
            track_id=det.track_id,
        )


