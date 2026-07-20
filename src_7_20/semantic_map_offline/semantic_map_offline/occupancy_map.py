"""ROS occupancy-grid metadata and map-to-image coordinate conversion."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class OccupancyMapMetadata:
    """Metadata stored in a nav_msgs map YAML file."""

    yaml_path: Path
    image_path: Path
    resolution: float
    origin: np.ndarray
    negate: int
    occupied_thresh: float
    free_thresh: float


def load_occupancy_map_metadata(path: str | Path) -> OccupancyMapMetadata:
    """Load a ROS map YAML and resolve its image relative to the YAML file."""
    yaml_path = Path(path).expanduser().resolve()
    if not yaml_path.is_file():
        raise FileNotFoundError(f"SLAM map YAML not found: {yaml_path}")
    document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"Invalid SLAM map YAML: {yaml_path}")

    origin = np.asarray(document.get("origin", []), dtype=np.float64)
    if origin.shape != (3,):
        raise ValueError("Map origin must contain [x, y, yaw]")
    resolution = float(document.get("resolution", 0.0))
    if resolution <= 0.0:
        raise ValueError("Map resolution must be positive")
    image_value = Path(str(document.get("image", ""))).expanduser()
    image_path = image_value if image_value.is_absolute() else yaml_path.parent / image_value
    image_path = image_path.resolve()
    if not image_path.is_file():
        raise FileNotFoundError(f"SLAM map image not found: {image_path}")

    return OccupancyMapMetadata(
        yaml_path=yaml_path,
        image_path=image_path,
        resolution=resolution,
        origin=origin,
        negate=int(document.get("negate", 0)),
        occupied_thresh=float(document.get("occupied_thresh", 0.65)),
        free_thresh=float(document.get("free_thresh", 0.25)),
    )


def world_to_map_pixels(
    xy: np.ndarray,
    *,
    image_width: int,
    image_height: int,
    resolution: float,
    origin: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Convert map-frame XY points to top-left-origin occupancy image pixels."""
    points = np.asarray(xy, dtype=np.float64)
    map_origin = np.asarray(origin, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("xy must have shape (N, 2)")
    if map_origin.shape != (3,):
        raise ValueError("origin must have shape (3,)")
    if image_width <= 0 or image_height <= 0 or resolution <= 0.0:
        raise ValueError("image dimensions and resolution must be positive")

    delta = points - map_origin[:2]
    cosine = np.cos(map_origin[2])
    sine = np.sin(map_origin[2])
    local_x = cosine * delta[:, 0] + sine * delta[:, 1]
    local_y = -sine * delta[:, 0] + cosine * delta[:, 1]
    columns = np.floor(local_x / resolution).astype(np.int64)
    rows_from_bottom = np.floor(local_y / resolution).astype(np.int64)
    rows = image_height - 1 - rows_from_bottom
    pixels = np.column_stack((columns, rows)).astype(np.int32)
    finite = np.all(np.isfinite(points), axis=1)
    valid = (
        finite
        & (columns >= 0)
        & (columns < image_width)
        & (rows >= 0)
        & (rows < image_height)
    )
    return pixels, valid
