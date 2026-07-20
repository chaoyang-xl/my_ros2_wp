"""ROS-independent semantic-map parsing and approach-pose selection."""

from __future__ import annotations

from dataclasses import dataclass
import json
from math import atan2, ceil, cos, floor, hypot, pi, sin
from pathlib import Path
from typing import Sequence


DEFAULT_DYNAMIC_LABELS = frozenset({"person"})


@dataclass(frozen=True)
class SemanticObject:
    """One confirmed semantic object expressed in the map frame."""

    object_id: str
    label: str
    x: float
    y: float
    size_x: float
    size_y: float
    confidence: float
    source: str


@dataclass(frozen=True)
class SemanticMap:
    """Validated static subset of a semantic object JSON snapshot."""

    frame_id: str
    objects: tuple[SemanticObject, ...]
    stamp_sec: float | None = None

    def object_by_id(self, object_id: str) -> SemanticObject | None:
        return next((obj for obj in self.objects if obj.object_id == object_id), None)


@dataclass(frozen=True)
class GridMap:
    """Minimal occupancy-grid representation with rotated-origin support."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: Sequence[int]
    origin_yaw: float = 0.0
    occupied_threshold: int = 50
    reject_unknown: bool = True

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("grid width and height must be positive")
        if self.resolution <= 0.0:
            raise ValueError("grid resolution must be positive")
        if len(self.data) != self.width * self.height:
            raise ValueError("grid data length does not match width * height")

    def world_to_grid(self, wx: float, wy: float) -> tuple[int, int] | None:
        dx = wx - self.origin_x
        dy = wy - self.origin_y
        local_x = cos(self.origin_yaw) * dx + sin(self.origin_yaw) * dy
        local_y = -sin(self.origin_yaw) * dx + cos(self.origin_yaw) * dy
        gx = floor(local_x / self.resolution)
        gy = floor(local_y / self.resolution)
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return gx, gy
        return None

    def grid_to_world_center(self, gx: int, gy: int) -> tuple[float, float]:
        local_x = (gx + 0.5) * self.resolution
        local_y = (gy + 0.5) * self.resolution
        return (
            self.origin_x
            + cos(self.origin_yaw) * local_x
            - sin(self.origin_yaw) * local_y,
            self.origin_y
            + sin(self.origin_yaw) * local_x
            + cos(self.origin_yaw) * local_y,
        )

    def is_free_with_clearance(self, wx: float, wy: float, clearance_m: float) -> bool:
        center = self.world_to_grid(wx, wy)
        if center is None:
            return False
        radius_cells = max(0, ceil(clearance_m / self.resolution))
        cgx, cgy = center
        for gy in range(cgy - radius_cells, cgy + radius_cells + 1):
            for gx in range(cgx - radius_cells, cgx + radius_cells + 1):
                if not (0 <= gx < self.width and 0 <= gy < self.height):
                    return False
                cx, cy = self.grid_to_world_center(gx, gy)
                if hypot(cx - wx, cy - wy) > clearance_m:
                    continue
                value = int(self.data[gy * self.width + gx])
                if value >= self.occupied_threshold:
                    return False
                if value < 0 and self.reject_unknown:
                    return False
        return True


@dataclass(frozen=True)
class ApproachPose:
    """A free map-frame pose facing a semantic object."""

    x: float
    y: float
    yaw: float
    object_id: str


def load_semantic_map(
    path: str | Path,
    excluded_labels: Sequence[str] = tuple(DEFAULT_DYNAMIC_LABELS),
) -> SemanticMap:
    """Load the existing snapshot format and retain confirmed static objects."""
    input_path = Path(path).expanduser()
    payload = json.loads(input_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("semantic map root must be a JSON object")

    frame_id = str(payload.get("frame_id", "map")).strip()
    if not frame_id:
        raise ValueError("semantic map frame_id must not be empty")
    raw_objects = payload.get("objects")
    if not isinstance(raw_objects, list):
        raise ValueError("semantic map objects must be a JSON array")

    excluded = {_normalise_label(label) for label in excluded_labels}
    objects: list[SemanticObject] = []
    seen_ids: set[str] = set()
    for index, raw in enumerate(raw_objects):
        if not isinstance(raw, dict):
            raise ValueError(f"objects[{index}] must be a JSON object")
        if raw.get("state", "confirmed") != "confirmed":
            continue
        label = _normalise_label(_required_string(raw, "label", index))
        if label in excluded:
            continue
        object_id = _required_string(raw, "id", index)
        if object_id in seen_ids:
            raise ValueError(f"duplicate semantic object id: {object_id}")
        seen_ids.add(object_id)
        objects.append(
            SemanticObject(
                object_id=object_id,
                label=label,
                x=_required_float(raw, "x", index),
                y=_required_float(raw, "y", index),
                size_x=max(0.0, _optional_float(raw, "size_x", 0.0, index)),
                size_y=max(0.0, _optional_float(raw, "size_y", 0.0, index)),
                confidence=_optional_float(raw, "confidence", 0.0, index),
                source=str(raw.get("source", "unknown")),
            )
        )

    stamp = payload.get("stamp_sec")
    return SemanticMap(
        frame_id=frame_id,
        objects=tuple(objects),
        stamp_sec=float(stamp) if stamp is not None else None,
    )


def semantic_map_snapshot(semantic_map: SemanticMap) -> dict:
    """Return a compact JSON-compatible static semantic-map snapshot."""
    snapshot = {
        "frame_id": semantic_map.frame_id,
        "count": len(semantic_map.objects),
        "objects": [
            {
                "id": obj.object_id,
                "label": obj.label,
                "state": "confirmed",
                "source": obj.source,
                "x": obj.x,
                "y": obj.y,
                "size_x": obj.size_x,
                "size_y": obj.size_y,
                "confidence": obj.confidence,
            }
            for obj in semantic_map.objects
        ],
    }
    if semantic_map.stamp_sec is not None:
        snapshot["source_stamp_sec"] = semantic_map.stamp_sec
    return snapshot


def choose_approach_pose(
    grid: GridMap,
    obj: SemanticObject,
    robot_x: float,
    robot_y: float,
    standoff_m: float = 0.7,
    clearance_m: float = 0.35,
    sample_count: int = 32,
) -> ApproachPose | None:
    """Choose the nearest collision-free sampled pose around an object."""
    if standoff_m < 0.0 or clearance_m < 0.0:
        raise ValueError("standoff and clearance must be non-negative")
    if sample_count < 4:
        raise ValueError("sample_count must be at least 4")

    object_radius = 0.5 * hypot(obj.size_x, obj.size_y)
    approach_radius = max(clearance_m, object_radius + standoff_m)
    candidates: list[tuple[float, ApproachPose]] = []
    for index in range(sample_count):
        angle = 2.0 * pi * index / sample_count
        x = obj.x + approach_radius * cos(angle)
        y = obj.y + approach_radius * sin(angle)
        if not grid.is_free_with_clearance(x, y, clearance_m):
            continue
        pose = ApproachPose(
            x=x,
            y=y,
            yaw=atan2(obj.y - y, obj.x - x),
            object_id=obj.object_id,
        )
        candidates.append((hypot(x - robot_x, y - robot_y), pose))

    return min(candidates, key=lambda item: item[0])[1] if candidates else None


def _normalise_label(label: str) -> str:
    return (label.strip() or "unknown").lower().replace(" ", "_")


def _required_string(raw: dict, key: str, index: int) -> str:
    value = raw.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"objects[{index}].{key} must be a non-empty string")
    return value.strip()


def _required_float(raw: dict, key: str, index: int) -> float:
    if key not in raw:
        raise ValueError(f"objects[{index}].{key} is required")
    return _as_float(raw[key], f"objects[{index}].{key}")


def _optional_float(raw: dict, key: str, default: float, index: int) -> float:
    if key not in raw:
        return default
    return _as_float(raw[key], f"objects[{index}].{key}")


def _as_float(value: object, field_name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{field_name} must be numeric")
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} must be numeric") from exc

