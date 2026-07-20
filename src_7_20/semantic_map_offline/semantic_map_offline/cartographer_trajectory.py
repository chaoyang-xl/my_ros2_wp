"""Final Cartographer trajectory interpolation and static-TF composition."""

from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class PoseSample:
    """A tracking-frame pose in the global map frame."""

    stamp_ns: int
    translation: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class InterpolatedPose:
    """An interpolated transform and the source-node interval."""

    matrix: np.ndarray
    lower_stamp_ns: int
    upper_stamp_ns: int
    alpha: float

    @property
    def interval_ns(self) -> int:
        return self.upper_stamp_ns - self.lower_stamp_ns


def normalize_frame(frame: str) -> str:
    """Normalize ROS frame names for graph lookup."""
    return str(frame).strip().lstrip("/")


def matrix_from_pose(
    translation: Iterable[float],
    quaternion_xyzw: Iterable[float],
) -> np.ndarray:
    """Build T_parent_child from translation and an ROS-order quaternion."""
    quaternion = np.asarray(tuple(quaternion_xyzw), dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if not np.isfinite(norm) or norm <= 1e-12:
        raise ValueError("Quaternion must be finite and non-zero")
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = Rotation.from_quat(quaternion / norm).as_matrix()
    matrix[:3, 3] = np.asarray(tuple(translation), dtype=np.float64)
    return matrix


def slerp_xyzw(
    first: Iterable[float],
    second: Iterable[float],
    alpha: float,
) -> np.ndarray:
    """Spherically interpolate two quaternions in ROS xyzw order."""
    q0 = np.asarray(tuple(first), dtype=np.float64)
    q1 = np.asarray(tuple(second), dtype=np.float64)
    q0 /= np.linalg.norm(q0)
    q1 /= np.linalg.norm(q1)

    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))

    if dot > 0.9995:
        result = q0 + float(alpha) * (q1 - q0)
        return result / np.linalg.norm(result)

    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    weight0 = math.sin((1.0 - float(alpha)) * theta) / sin_theta
    weight1 = math.sin(float(alpha) * theta) / sin_theta
    return weight0 * q0 + weight1 * q1


class TrajectoryInterpolator:
    """Interpolate final optimized tracking poses at arbitrary sensor stamps."""

    def __init__(self, samples: Iterable[PoseSample]) -> None:
        ordered = sorted(samples, key=lambda item: item.stamp_ns)
        if not ordered:
            raise ValueError("Trajectory has no pose samples")
        unique: list[PoseSample] = []
        for sample in ordered:
            if unique and sample.stamp_ns == unique[-1].stamp_ns:
                unique[-1] = sample
            else:
                unique.append(sample)
        self.samples = unique
        self.stamps_ns = [sample.stamp_ns for sample in unique]

    def interpolate(
        self,
        stamp_ns: int,
        *,
        max_interval_ns: int | None = None,
    ) -> InterpolatedPose | None:
        """Return an exact/interpolated pose, or None outside accepted bounds."""
        stamp_ns = int(stamp_ns)
        index = bisect_left(self.stamps_ns, stamp_ns)
        if index < len(self.samples) and self.stamps_ns[index] == stamp_ns:
            sample = self.samples[index]
            matrix = matrix_from_pose(sample.translation, sample.quaternion_xyzw)
            return InterpolatedPose(matrix, stamp_ns, stamp_ns, 0.0)
        if index == 0 or index == len(self.samples):
            return None

        lower = self.samples[index - 1]
        upper = self.samples[index]
        interval_ns = upper.stamp_ns - lower.stamp_ns
        if interval_ns <= 0:
            return None
        if max_interval_ns is not None and interval_ns > int(max_interval_ns):
            return None

        alpha = (stamp_ns - lower.stamp_ns) / interval_ns
        translation = (
            (1.0 - alpha) * np.asarray(lower.translation, dtype=np.float64)
            + alpha * np.asarray(upper.translation, dtype=np.float64)
        )
        quaternion = slerp_xyzw(
            lower.quaternion_xyzw,
            upper.quaternion_xyzw,
            alpha,
        )
        matrix = matrix_from_pose(translation, quaternion)
        return InterpolatedPose(
            matrix,
            lower.stamp_ns,
            upper.stamp_ns,
            float(alpha),
        )


class StaticTransformGraph:
    """Resolve rigid transforms from a set of ROS static transforms."""

    def __init__(self) -> None:
        self._edges: dict[str, dict[str, np.ndarray]] = {}

    def add(
        self,
        parent_frame: str,
        child_frame: str,
        transform_parent_child: np.ndarray,
    ) -> None:
        """Add T_parent_child and its inverse."""
        parent = normalize_frame(parent_frame)
        child = normalize_frame(child_frame)
        if not parent or not child:
            raise ValueError("Static transform frames cannot be empty")
        transform = np.asarray(transform_parent_child, dtype=np.float64)
        if transform.shape != (4, 4):
            raise ValueError("Static transform must be 4x4")
        self._edges.setdefault(parent, {})[child] = transform
        self._edges.setdefault(child, {})[parent] = np.linalg.inv(transform)

    @property
    def frames(self) -> tuple[str, ...]:
        return tuple(sorted(self._edges))

    def lookup(self, parent_frame: str, child_frame: str) -> np.ndarray:
        """Return T_parent_child by composing the shortest available TF path."""
        parent = normalize_frame(parent_frame)
        child = normalize_frame(child_frame)
        if parent == child:
            return np.eye(4, dtype=np.float64)
        if parent not in self._edges or child not in self._edges:
            raise KeyError(
                f"Unknown static TF frame: {parent!r} or {child!r}; "
                f"available={self.frames}"
            )

        queue: list[tuple[str, np.ndarray]] = [
            (parent, np.eye(4, dtype=np.float64))
        ]
        visited = {parent}
        for current, transform_parent_current in queue:
            for neighbor, transform_current_neighbor in self._edges[current].items():
                if neighbor in visited:
                    continue
                transform_parent_neighbor = (
                    transform_parent_current @ transform_current_neighbor
                )
                if neighbor == child:
                    return transform_parent_neighbor
                visited.add(neighbor)
                queue.append((neighbor, transform_parent_neighbor))
        raise KeyError(f"No static TF path from {parent!r} to {child!r}")


def nearest_index(
    sorted_stamps_ns: list[int],
    query_stamp_ns: int,
    *,
    max_delta_ns: int | None = None,
) -> tuple[int | None, int | None]:
    """Return nearest index and signed query-to-sample delta in nanoseconds."""
    if not sorted_stamps_ns:
        return None, None
    query = int(query_stamp_ns)
    index = bisect_left(sorted_stamps_ns, query)
    candidates = []
    if index < len(sorted_stamps_ns):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    best = min(
        candidates,
        key=lambda candidate: (
            abs(sorted_stamps_ns[candidate] - query),
            sorted_stamps_ns[candidate],
        ),
    )
    delta_ns = sorted_stamps_ns[best] - query
    if max_delta_ns is not None and abs(delta_ns) > int(max_delta_ns):
        return None, delta_ns
    return best, delta_ns
