"""Small geometry helpers for semantic map projection."""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Sequence


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole camera intrinsics matching ROS CameraInfo's K matrix."""

    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_k_matrix(cls, k: Sequence[float]) -> "CameraIntrinsics":
        """Create intrinsics from the 9-value row-major CameraInfo.k array."""

        if len(k) != 9:
            raise ValueError("CameraInfo K matrix must contain exactly 9 values")
        return cls(fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]))


@dataclass(frozen=True)
class Quaternion:
    """Quaternion in ROS order x, y, z, w."""

    x: float
    y: float
    z: float
    w: float

    def normalized(self) -> "Quaternion":
        norm = sqrt(self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w)
        if norm == 0.0:
            raise ValueError("Quaternion norm must be non-zero")
        return Quaternion(self.x / norm, self.y / norm, self.z / norm, self.w / norm)


@dataclass(frozen=True)
#刚体变换，包含平移和旋转信息，以及源坐标系和目标坐标系的名称
class Transform3D:
    """Rigid transform from a source frame into a target frame."""

    translation: tuple[float, float, float]
    rotation: Quaternion = Quaternion(0.0, 0.0, 0.0, 1.0)
    source_frame: str = "camera_link"
    target_frame: str = "map"
    #将一个3D点从源坐标系变换到目标坐标系，返回变换后的点坐标
    #得到地图坐标
    def apply(self, point: tuple[float, float, float]) -> tuple[float, float, float]:
        """Apply this transform to a 3D point."""

        q = self.rotation.normalized()
        px, py, pz = point

        # Quaternion-vector multiplication optimized as:
        # v' = v + 2*w*(q_vec x v) + 2*(q_vec x (q_vec x v))
        uv = _cross((q.x, q.y, q.z), (px, py, pz))
        uuv = _cross((q.x, q.y, q.z), uv)
        rx = px + 2.0 * (q.w * uv[0] + uuv[0])
        ry = py + 2.0 * (q.w * uv[1] + uuv[1])
        rz = pz + 2.0 * (q.w * uv[2] + uuv[2])

        tx, ty, tz = self.translation
        return rx + tx, ry + ty, rz + tz


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )

#像素到相机坐标系的反投影函数，根据像素坐标、深度值和相机内参计算出相机坐标系中的3D点坐标
def pixel_to_camera(
    u: float,
    v: float,
    depth_m: float,
    intrinsics: CameraIntrinsics,
) -> tuple[float, float, float]:
    """Back-project a pixel and metric depth into the camera optical frame."""

    if depth_m <= 0.0:
        raise ValueError("Depth must be positive")
    x = (u - intrinsics.cx) * depth_m / intrinsics.fx
    y = (v - intrinsics.cy) * depth_m / intrinsics.fy
    z = depth_m
    return x, y, z
