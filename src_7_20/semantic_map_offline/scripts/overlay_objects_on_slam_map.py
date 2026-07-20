#!/usr/bin/env python3
"""Overlay fused semantic object XY point clouds on a saved ROS SLAM map."""

from __future__ import annotations

import argparse
import colorsys
import json
from pathlib import Path
import sys

import cv2
import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from semantic_map_offline.occupancy_map import (  # noqa: E402
    load_occupancy_map_metadata,
    world_to_map_pixels,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Draw fused object point clouds and class labels on a ROS map."
    )
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--objects-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--json-output", type=Path, default=None)
    parser.add_argument("--min-observations", type=int, default=5)
    parser.add_argument("--point-radius", type=int, default=1)
    parser.add_argument("--point-opacity", type=float, default=0.82)
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def object_color(track_id: int) -> tuple[int, int, int]:
    hue = (track_id * 0.61803398875) % 1.0
    rgb = colorsys.hsv_to_rgb(hue, 0.82, 0.92)
    return tuple(int(round(channel * 255)) for channel in rgb[::-1])


def load_objects(objects_dir: Path, min_observations: int) -> list[dict]:
    objects = []
    for path in sorted(objects_dir.glob("object_*.npz")):
        with np.load(path, allow_pickle=False) as data:
            observations = int(data["observation_count"])
            status = str(data["status"]) if "status" in data.files else "confirmed"
            if observations < min_observations or status != "confirmed":
                continue
            points = np.asarray(data["points_map"], dtype=np.float32)
            points = points[np.all(np.isfinite(points), axis=1)]
            if points.shape[0] == 0:
                continue
            objects.append({
                "track_id": int(data["track_id"]),
                "class_id": int(data["class_id"]),
                "class_name": str(data["class_name"]),
                "confidence": float(data["confidence"]),
                "observation_count": observations,
                "points": points,
                "source": path,
            })
    return objects


def draw_label(image: np.ndarray, text: str, center: tuple[int, int], color) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.48
    (width, height), baseline = cv2.getTextSize(text, font, scale, 1)
    left = int(np.clip(center[0] - width / 2 - 5, 1, max(1, image.shape[1] - width - 11)))
    top = int(np.clip(center[1] - height / 2 - 5, 1, max(1, image.shape[0] - height - baseline - 11)))
    right = left + width + 10
    bottom = top + height + baseline + 10
    cv2.rectangle(image, (left, top), (right, bottom), (250, 250, 250), cv2.FILLED)
    cv2.rectangle(image, (left, top), (right, bottom), color, 1, cv2.LINE_AA)
    cv2.putText(image, text, (left + 5, top + height + 4), font, scale, (20, 20, 20), 1, cv2.LINE_AA)


def render(args: argparse.Namespace) -> tuple[Path, Path]:
    metadata = load_occupancy_map_metadata(args.map_yaml)
    grayscale = cv2.imread(str(metadata.image_path), cv2.IMREAD_GRAYSCALE)
    if grayscale is None:
        raise RuntimeError(f"Failed to read map image: {metadata.image_path}")
    image = cv2.cvtColor(grayscale, cv2.COLOR_GRAY2BGR)
    overlay = image.copy()
    objects = load_objects(args.objects_dir, args.min_observations)
    records = []

    for item in objects:
        pixels, valid = world_to_map_pixels(
            item["points"][:, :2],
            image_width=image.shape[1],
            image_height=image.shape[0],
            resolution=metadata.resolution,
            origin=metadata.origin,
        )
        visible = pixels[valid]
        if visible.shape[0] == 0:
            continue
        color = object_color(item["track_id"])
        radius = max(0, args.point_radius)
        for x, y in visible:
            cv2.circle(overlay, (int(x), int(y)), radius, color, cv2.FILLED)

        left, top = np.min(visible, axis=0).tolist()
        right, bottom = np.max(visible, axis=0).tolist()
        item["visible"] = visible
        item["rectangle"] = (int(left), int(top), int(right), int(bottom))
        records.append({
            "track_id": item["track_id"],
            "class_id": item["class_id"],
            "class_name": item["class_name"],
            "confidence": item["confidence"],
            "observation_count": item["observation_count"],
            "point_count": int(item["points"].shape[0]),
            "visible_point_count": int(visible.shape[0]),
            "bounds_map_xy": {
                "minimum": np.min(item["points"][:, :2], axis=0).tolist(),
                "maximum": np.max(item["points"][:, :2], axis=0).tolist(),
            },
            "rectangle_pixels": {
                "left": int(left), "top": int(top),
                "right": int(right), "bottom": int(bottom),
            },
            "source_npz": str(item["source"]),
        })

    opacity = float(np.clip(args.point_opacity, 0.0, 1.0))
    image = cv2.addWeighted(overlay, opacity, image, 1.0 - opacity, 0.0)
    for item in objects:
        if "rectangle" not in item:
            continue
        left, top, right, bottom = item["rectangle"]
        color = object_color(item["track_id"])
        cv2.rectangle(image, (left, top), (right, bottom), color, 2, cv2.LINE_AA)
        draw_label(image, item["class_name"], ((left + right) // 2, (top + bottom) // 2), color)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), image):
        raise RuntimeError(f"Failed to write overlay: {args.output}")
    json_output = args.json_output or args.output.with_suffix(".json")
    document = {
        "schema_version": 1,
        "frame_id": "map",
        "map_yaml": str(metadata.yaml_path),
        "map_image": str(metadata.image_path),
        "output_image": str(args.output),
        "resolution": metadata.resolution,
        "origin": metadata.origin.tolist(),
        "image_size": {"width": image.shape[1], "height": image.shape[0]},
        "object_count": len(records),
        "objects": records,
    }
    json_output.write_text(json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8")
    if args.show:
        cv2.imshow("Semantic objects on SLAM map", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return args.output, json_output


def main() -> None:
    args = parse_args()
    if not args.objects_dir.is_dir():
        raise SystemExit(f"Objects directory does not exist: {args.objects_dir}")
    output, json_output = render(args)
    print(f"overlay: {output}")
    print(f"metadata: {json_output}")


if __name__ == "__main__":
    main()
