"""Command-line demo for projection and fusion using a JSON fixture.

This entry point is useful before the ROS 2 wrapper is connected: it accepts the
same logical inputs the ROS node needs (CameraInfo K, depth image, camera->map
transform, detections) and prints stable semantic map objects as JSON.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

from semantic_map_pkg.fusion import SemanticObjectTracker
from semantic_map_pkg.geometry import CameraIntrinsics, Quaternion, Transform3D
from semantic_map_pkg.projection import Detection2D, SemanticProjector


def main() -> None:
    parser = argparse.ArgumentParser(description="Project RGB-D detections into a 2D semantic map")
    parser.add_argument("input", type=Path, help="JSON file containing intrinsics, transform, depth, detections")
    parser.add_argument(
        "--size-mode",
        choices=("prior_size", "observed_size"),
        default="prior_size",
        help="How rectangle size is produced: prior class defaults or observed from bbox+depth.",
    )
    args = parser.parse_args()

    payload = json.loads(args.input.read_text(encoding="utf-8"))
    intrinsics = CameraIntrinsics.from_k_matrix(payload["camera_info_k"])
    transform_payload = payload["camera_to_map"]
    transform = Transform3D(
        translation=tuple(transform_payload["translation"]),
        rotation=Quaternion(*transform_payload.get("rotation_xyzw", [0.0, 0.0, 0.0, 1.0])),
        source_frame=transform_payload.get("source_frame", "camera_link"),
        target_frame=transform_payload.get("target_frame", "map"),
    )
    detections = [Detection2D(**item) for item in payload["detections"]]

    projector = SemanticProjector(intrinsics=intrinsics, size_mode=args.size_mode)
    projected = projector.project_detections(detections, payload["depth_image_m"], transform)
    tracker = SemanticObjectTracker()
    objects = tracker.update(projected, now=float(payload.get("timestamp", 0.0)))

    print(json.dumps([asdict(item) for item in objects], ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
