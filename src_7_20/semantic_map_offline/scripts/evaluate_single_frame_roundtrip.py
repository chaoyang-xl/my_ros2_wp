#!/usr/bin/env python3
"""Evaluate independent per-frame projection and reprojection accuracy."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import cv2
import numpy as np
from ultralytics import YOLO


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from semantic_map_offline.bbox_projection import CameraIntrinsics, project_bbox_depth  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--start", type=int, default=800)
    parser.add_argument("--frames", type=int, default=15)
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument("--pixel-stride", type=int, default=2)
    parser.add_argument("--visibility-tolerance-m", type=float, default=0.10)
    return parser.parse_args()


def camera_to_world(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
    """将当前检测的相机点云正投影到 map/world。"""
    return (points.astype(np.float64) @ pose[:3, :3].T + pose[:3, 3]).astype(np.float32)


def world_to_image(
    points_world: np.ndarray,
    pose: np.ndarray,
    intrinsics: CameraIntrinsics,
) -> tuple[np.ndarray, np.ndarray]:
    """将 map/world 点云变回当前相机并反投影到图像。"""
    inverse_pose = np.linalg.inv(pose)
    camera = points_world.astype(np.float64) @ inverse_pose[:3, :3].T + inverse_pose[:3, 3]
    valid = camera[:, 2] > 1e-6
    uv = np.full((camera.shape[0], 2), np.nan, dtype=np.float64)
    uv[valid, 0] = intrinsics.fx * camera[valid, 0] / camera[valid, 2] + intrinsics.cx
    uv[valid, 1] = intrinsics.fy * camera[valid, 1] / camera[valid, 2] + intrinsics.cy
    return uv.astype(np.float32), camera[:, 2].astype(np.float32)


def visible_points(
    uv: np.ndarray,
    z: np.ndarray,
    depth_m: np.ndarray,
    tolerance_m: float,
) -> np.ndarray:
    """保留位于图像内且与当前深度表面一致的反投影点。"""
    ui = np.rint(uv[:, 0]).astype(np.int32)
    vi = np.rint(uv[:, 1]).astype(np.int32)
    height, width = depth_m.shape
    finite = np.all(np.isfinite(uv), axis=1)
    inside = finite & (ui >= 0) & (ui < width) & (vi >= 0) & (vi < height)
    indices = np.flatnonzero(inside)
    if indices.size == 0:
        return np.empty((0, 2), dtype=np.float32)
    measured = depth_m[vi[indices], ui[indices]]
    visible = np.isfinite(measured) & (measured > 0.0) & (np.abs(z[indices] - measured) <= tolerance_m)
    return uv[indices[visible]]


def projected_bbox(points_uv: np.ndarray, width: int, height: int) -> list[float] | None:
    '''计算反投影点的边界框。矩形框的坐标为 [x_min, y_min, x_max, y_max]，如果没有足够的点则返回 None。'''
    if points_uv.shape[0] < 3:
        return None
    low = np.min(points_uv, axis=0)
    high = np.max(points_uv, axis=0)
    return [
        float(np.clip(low[0], 0, width - 1)), float(np.clip(low[1], 0, height - 1)),
        float(np.clip(high[0], 0, width - 1)), float(np.clip(high[1], 0, height - 1)),
    ]


def bbox_iou(first: list[float], second: list[float]) -> float:
    '''计算两个边界框的交并比（IoU）。'''
    left, top = max(first[0], second[0]), max(first[1], second[1])
    right, bottom = min(first[2], second[2]), min(first[3], second[3])
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    second_area = max(0.0, second[2] - second[0]) * max(0.0, second[3] - second[1])
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def draw_box(image: np.ndarray, box: list[float], color, text: str) -> None:
    '''在图像上绘制边界框和标签。'''
    x1, y1, x2, y2 = (int(round(value)) for value in box)
    cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
    cv2.putText(image, text, (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)


def save_binary_ply(path: Path, points: np.ndarray, rgb: np.ndarray) -> None:
    """保存可由 CloudCompare/Open3D 直接读取的二进制 XYZRGB PLY。"""
    vertices = np.empty(points.shape[0], dtype=[
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
        ("red", "u1"), ("green", "u1"), ("blue", "u1"),
    ])
    vertices["x"], vertices["y"], vertices["z"] = points.T
    vertices["red"], vertices["green"], vertices["blue"] = rgb.T
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {points.shape[0]}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
    ).encode("ascii")
    with path.open("wb") as stream:
        stream.write(header)
        stream.write(vertices.tobytes())


def main() -> None:
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    object_directory = args.output / "objects"
    object_directory.mkdir(parents=True, exist_ok=True)
    camera = json.loads((args.data_root / "cam_params.json").read_text(encoding="utf-8"))["camera"]
    poses = np.loadtxt(args.data_root / "traj.txt", dtype=np.float64).reshape(-1, 4, 4)
    # 标准Replica traj.txt可直接作为OpenCV深度点云的camera-to-world矩阵。
    intrinsics = CameraIntrinsics(camera["fx"], camera["fy"], camera["cx"], camera["cy"])
    frame_ids = list(range(args.start, args.start + args.frames))
    image_paths = [args.data_root / "results" / f"frame{index:06d}.jpg" for index in frame_ids]
    depth_paths = [args.data_root / "results" / f"depth{index:06d}.png" for index in frame_ids]
    predictions = YOLO(str(args.model)).predict(
        [str(path) for path in image_paths], imgsz=640, conf=args.confidence, verbose=False
    )

    metrics = []
    for frame_id, image_path, depth_path, pose, result in zip(
        frame_ids, image_paths, depth_paths, poses[frame_ids], predictions
    ):
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        depth_raw = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        depth_m = depth_raw.astype(np.float32) / float(camera["scale"])
        overlay = image.copy()
        if result.boxes is not None:
            boxes = result.boxes
            for detection_id, (box, confidence, class_id) in enumerate(zip(
                boxes.xyxy.cpu().numpy(), boxes.conf.cpu().numpy(), boxes.cls.cpu().numpy().astype(int)
            )):
                projected = project_bbox_depth(
                    depth_m, box.tolist(), (image.shape[1], image.shape[0]), intrinsics,
                    pixel_stride=args.pixel_stride, min_depth_m=0.1, max_depth_m=10.0,
                )
                if projected is None:
                    continue
                points_world = camera_to_world(projected.points_camera, pose)
                reprojected_uv, reprojected_z = world_to_image(points_world, pose, intrinsics)
                # 与正投影时保存的深度像素比较，直接衡量闭环的数值误差。
                pixel_errors = np.linalg.norm(reprojected_uv - projected.depth_uv, axis=1)
                visible_uv = visible_points(
                    reprojected_uv, reprojected_z, depth_m, args.visibility_tolerance_m
                )
                reverse_box = projected_bbox(visible_uv, image.shape[1], image.shape[0])
                iou = bbox_iou(box.tolist(), reverse_box) if reverse_box else 0.0
                class_name = str(result.names[int(class_id)])
                safe_class_name = "".join(
                    char if char.isalnum() or char in ("-", "_") else "_"
                    for char in class_name
                )
                object_stem = f"frame{frame_id:06d}_det{detection_id:02d}_{safe_class_name}"
                u = np.clip(np.rint(projected.image_uv[:, 0]).astype(np.int32), 0, image.shape[1] - 1)
                v = np.clip(np.rint(projected.image_uv[:, 1]).astype(np.int32), 0, image.shape[0] - 1)
                rgb = image[v, u, ::-1].copy()
                npz_path = object_directory / f"{object_stem}.npz"
                ply_path = object_directory / f"{object_stem}.ply"
                np.savez_compressed(
                    npz_path,
                    points_map=points_world,
                    points_camera=projected.points_camera,
                    rgb=rgb,
                    depth_uv=projected.depth_uv,
                    image_uv=projected.image_uv,
                    detection_xyxy=np.asarray(box, dtype=np.float32),
                    class_id=np.asarray(int(class_id), dtype=np.int32),
                    class_name=np.asarray(class_name),
                    confidence=np.asarray(float(confidence), dtype=np.float32),
                )
                save_binary_ply(ply_path, points_world, rgb)
                metrics.append({
                    "frame_id": frame_id,
                    "detection_id": detection_id,
                    "class_id": int(class_id),
                    "class_name": class_name,
                    "confidence": float(confidence),
                    "detection_xyxy": box.tolist(),
                    "reprojection_xyxy": reverse_box,
                    "point_count": int(projected.points_camera.shape[0]),
                    "visible_point_count": int(visible_uv.shape[0]),
                    "mean_pixel_error": float(np.mean(pixel_errors)),
                    "max_pixel_error": float(np.max(pixel_errors)),
                    "bbox_iou": iou,
                    "object_npz": str(npz_path.relative_to(args.output)),
                    "object_ply": str(ply_path.relative_to(args.output)),
                })
                draw_box(overlay, box.tolist(), (0, 0, 255), f"YOLO {class_name}")
                if reverse_box:
                    draw_box(overlay, reverse_box, (0, 255, 0), f"single 3D IoU={iou:.2f}")
                    sample = visible_uv[::max(1, visible_uv.shape[0] // 1000)].astype(np.int32)
                    for u, v in sample:
                        cv2.circle(overlay, (int(u), int(v)), 1, (255, 255, 0), -1)
        cv2.imwrite(str(args.output / f"frame{frame_id:06d}_overlay.jpg"), overlay)

    if metrics:
        with (args.output / "metrics.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(metrics[0]))
            writer.writeheader()
            writer.writerows(metrics)
    (args.output / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    summary = {
        "mode": "single_frame_no_tracking_no_fusion",
        "frame_start": args.start,
        "frame_count": args.frames,
        "detections": len(metrics),
        "valid_reprojections": sum(item["reprojection_xyxy"] is not None for item in metrics),
        "object_point_clouds": len(metrics),
        "mean_pixel_error": float(np.mean([item["mean_pixel_error"] for item in metrics])) if metrics else 0.0,
        "max_pixel_error": float(np.max([item["max_pixel_error"] for item in metrics])) if metrics else 0.0,
        "mean_bbox_iou": float(np.mean([item["bbox_iou"] for item in metrics])) if metrics else 0.0,
        "median_bbox_iou": float(np.median([item["bbox_iou"] for item in metrics])) if metrics else 0.0,
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
