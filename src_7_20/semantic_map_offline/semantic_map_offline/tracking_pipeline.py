#!/usr/bin/env python3
"""Evaluate multi-frame object projection, association, and fusion."""
# 仅用于replica测试
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np


from semantic_map_offline.bbox_projection import CameraIntrinsics, project_bbox_depth  # noqa: E402
from semantic_map_offline.mask_projection import project_mask_depth  # noqa: E402
from semantic_map_offline.object_tracker import (
    ObjectObservation,
    ObjectTracker,
)
from semantic_map_offline.object_map_io import object_record, write_semantic_object_map  # noqa: E402
from semantic_map_offline.point_cloud_io import save_object_ply
from semantic_map_offline.world_detection import load_class_prompts, load_yolo_world


def build_common_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--clip-model", type=Path)
    parser.add_argument("--classes-path", type=Path)
    parser.add_argument(
        "--excluded-labels",
        default="person,wall,floor,ceiling,unknown",
        help="Comma-separated YOLO-World prompts to exclude",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--start", type=int, default=800)
    parser.add_argument(
        "--frames", type=int, default=15,
        help="Number of frames to process; 0 uses all contiguous frames from --start",
    )
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument("--device", default="", help="Ultralytics device, e.g. cpu, 0; empty uses auto")
    parser.add_argument("--pixel-stride", type=int, default=2)
    parser.add_argument("--pose-convention", choices=("replica", "opencv", "replica_opengl_legacy"), default="replica")
    # 与ROS object_fusion_node使用同名参数和相同默认值。
    parser.add_argument("--voxel-size", type=float, default=0.02)
    parser.add_argument("--overlap-radius", type=float, default=0.04)
    parser.add_argument("--max-centroid-distance-m", type=float, default=1.0)
    parser.add_argument("--min-geometric-overlap", type=float, default=0.05)
    parser.add_argument("--association-threshold", type=float, default=0.45)
    parser.add_argument("--geometry-weight", type=float, default=0.7)
    parser.add_argument("--semantic-weight", type=float, default=0.3)
    parser.add_argument("--observation-cluster-eps", type=float, default=0.10)
    parser.add_argument("--observation-cluster-min-points", type=int, default=10)
    parser.add_argument("--denoise-interval", type=int, default=20)
    parser.add_argument("--min-bbox-overlap", type=float, default=0.0)
    parser.add_argument("--max-extent-growth", type=float, default=2.0)
    parser.add_argument("--map-merge-interval", type=int, default=20)
    parser.add_argument("--map-merge-overlap", type=float, default=0.80)
    parser.add_argument("--min-confirmed-observations", type=int, default=3)
    parser.add_argument("--candidate-max-missed-frames", type=int, default=30)
    parser.add_argument(
        "--progress-every", type=int, default=10,
        help="Print progress every N frames; 0 only prints the final frame",
    )
    return parser


def load_dataset(args: argparse.Namespace):
    params = json.loads((args.data_root / "cam_params.json").read_text(encoding="utf-8"))["camera"]
    poses = np.loadtxt(args.data_root / "traj.txt", dtype=np.float64).reshape(-1, 4, 4)
    if args.pose_convention == "replica_opengl_legacy":
        # 旧实验轴翻转，仅用于复现；标准Replica traj.txt应直接作为OpenCV c2w使用。
        axis_conversion = np.diag([1.0, -1.0, -1.0, 1.0])
        poses = poses @ axis_conversion
    intrinsics = CameraIntrinsics(params["fx"], params["fy"], params["cx"], params["cy"])
    return params, poses, intrinsics


def transform_camera_to_world(points: np.ndarray, camera_to_world: np.ndarray) -> np.ndarray:
    return (points.astype(np.float64) @ camera_to_world[:3, :3].T + camera_to_world[:3, 3]).astype(np.float32)


def load_exported_detections(path: Path, min_confidence: float) -> list[dict]:
    """Load one exporter detection record into the tracking schema."""
    payload = json.loads(path.read_text(encoding="utf-8"))
    detections = payload.get("detections", [])
    if not isinstance(detections, list):
        raise ValueError(f"Invalid detections list: {path}")

    normalized = []
    for index, item in enumerate(detections):
        try:
            box = [float(value) for value in item["xyxy"]]
            confidence = float(item["confidence"])
            class_id = int(item["class_id"])
            class_name = str(item["class_name"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"Invalid detection {index} in {path}: {exc}") from exc
        if len(box) != 4:
            raise ValueError(f"Detection {index} in {path} has invalid xyxy")
        if confidence < min_confidence or class_name.casefold() == "person":
            continue
        normalized.append({
            "class_id": class_id, "class_name": class_name,
            "confidence": confidence, "xyxy": box,
        })
    return normalized


def run_tracking(args: argparse.Namespace, sam_config: dict | None = None) -> dict:
    args.output.mkdir(parents=True, exist_ok=True)
    params, poses, intrinsics = load_dataset(args)
    if args.start < 0 or args.frames < 0:
        raise ValueError("--start and --frames must be non-negative")

    available_frames = 0
    for index in range(args.start, len(poses)):
        image_path = args.data_root / "results" / f"frame{index:06d}.jpg"
        depth_path = args.data_root / "results" / f"depth{index:06d}.png"
        if not image_path.is_file() or not depth_path.is_file():
            break
        available_frames += 1

    frame_count = available_frames if args.frames == 0 else args.frames
    if frame_count == 0 or frame_count > available_frames:
        raise FileNotFoundError(
            "Requested frame range is incomplete: "
            f"start={args.start}, requested={args.frames}, "
            f"available_contiguous={available_frames}, poses={len(poses)}. "
            "Use --frames 0 to process all available frames."
        )

    frame_ids = list(range(args.start, args.start + frame_count))
    image_paths = [args.data_root / "results" / f"frame{index:06d}.jpg" for index in frame_ids]
    depth_paths = [args.data_root / "results" / f"depth{index:06d}.png" for index in frame_ids]

    # Exported bag datasets reuse their timestamp-matched YOLO-World records.
    # Replica-style datasets without those records fall back to inference.
    detection_paths = [
        args.data_root / "detections" / f"{index:06d}.json"
        for index in frame_ids
    ]
    use_exported_detections = all(path.is_file() for path in detection_paths)
    model = None
    if use_exported_detections:
        detection_source = "exported_json"
    else:
        missing_arguments = [
            name for name, value in (
                ("--model", args.model), ("--clip-model", args.clip_model),
                ("--classes-path", args.classes_path),
            ) if value is None
        ]
        if missing_arguments:
            raise ValueError(
                "Dataset detections are incomplete and YOLO-World fallback requires: "
                + ", ".join(missing_arguments)
            )
        excluded_labels = [name.strip() for name in args.excluded_labels.split(",")]
        class_prompts = load_class_prompts(args.classes_path, excluded_labels)
        model = load_yolo_world(args.model, class_prompts, args.clip_model)
        detection_source = "yolo_world_inference"

    projection_mode = "mobile_sam" if sam_config is not None else "bbox"
    print(
        f"Tracking {frame_count} frames from {args.start}; "
        f"detections={detection_source}; projection={projection_mode}",
        flush=True,
    )
    sam_predictor = None
    sam_device = None
    if sam_config is not None:
        import torch
        import sys
        sys.path.insert(0, str(sam_config["source"]))
        from mobile_sam import SamPredictor, sam_model_registry
        sam_device = sam_config["device"] or ("cuda" if torch.cuda.is_available() else "cpu")
        mobile_sam = sam_model_registry["vit_t"](checkpoint=str(sam_config["checkpoint"]))
        mobile_sam.to(device=sam_device)
        mobile_sam.eval()
        sam_predictor = SamPredictor(mobile_sam)
        print(f"MobileSAM enabled: device={sam_device}, mask_erode_px={sam_config['mask_erode_px']}")
    fusion = ObjectTracker(
        voxel_size=args.voxel_size,
        overlap_radius=args.overlap_radius,
        max_centroid_distance_m=args.max_centroid_distance_m,
        min_geometric_overlap=args.min_geometric_overlap,
        min_bbox_overlap=args.min_bbox_overlap,
        association_threshold=args.association_threshold,
        geometry_weight=args.geometry_weight,
        semantic_weight=args.semantic_weight,
        observation_cluster_eps=args.observation_cluster_eps,
        observation_cluster_min_points=args.observation_cluster_min_points,
        denoise_interval=args.denoise_interval,
        max_extent_growth=args.max_extent_growth,
        map_merge_interval=args.map_merge_interval,
        map_merge_overlap=args.map_merge_overlap,
        min_confirmed_observations=args.min_confirmed_observations,
        candidate_max_missed_frames=args.candidate_max_missed_frames,
    )
    detections_json = []

    started_at = time.perf_counter()
    input_detection_count = 0
    progress_every = max(0, int(args.progress_every))
    progress_width = len(str(frame_count))
    for processed_count, (frame_id, image_path, depth_path, detection_path, pose) in enumerate(
        zip(frame_ids, image_paths, depth_paths, detection_paths, poses[frame_ids]),
        start=1,
    ):
        frame_started_at = time.perf_counter()
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        depth_raw = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        if image is None or depth_raw is None:
            raise FileNotFoundError(f"Failed to read RGB-D frame {frame_id}")

        if use_exported_detections:
            detections = load_exported_detections(detection_path, args.confidence)
        else:
            # Replica-style datasets without exported JSON run YOLO-World here.
            result = model.predict(
                str(image_path), imgsz=640, conf=args.confidence,
                device=args.device or None, verbose=False,
            )[0]
            detections = []
            if result.boxes is not None:
                for box, confidence, class_id in zip(
                    result.boxes.xyxy.cpu().numpy(),
                    result.boxes.conf.cpu().numpy(),
                    result.boxes.cls.cpu().numpy().astype(int),
                ):
                    class_name = str(result.names[class_id])
                    if class_name.casefold() == "person":
                        continue
                    detections.append({
                        "class_id": int(class_id), "class_name": class_name,
                        "confidence": float(confidence), "xyxy": box.tolist(),
                    })
        input_detection_count += len(detections)
        depth_m = depth_raw.astype(np.float32) / float(params["scale"])
        sam_masks = None
        sam_scores = None
        if sam_predictor is not None and detections:
            import torch
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            sam_predictor.set_image(rgb_image)
            boxes_for_sam = torch.as_tensor(
                [item["xyxy"] for item in detections],
                dtype=torch.float32, device=sam_device,
            )
            transformed_boxes = sam_predictor.transform.apply_boxes_torch(
                boxes_for_sam, rgb_image.shape[:2]
            )
            with torch.inference_mode():
                masks_tensor, scores_tensor, _ = sam_predictor.predict_torch(
                    point_coords=None, point_labels=None, boxes=transformed_boxes,
                    multimask_output=False,
                )
            sam_masks = masks_tensor[:, 0].detach().cpu().numpy().astype(bool)
            sam_scores = scores_tensor[:, 0].detach().cpu().numpy()
            if sam_config["mask_erode_px"] > 0:
                kernel_size = sam_config["mask_erode_px"] * 2 + 1
                kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
                sam_masks = np.stack([
                    cv2.erode(mask.astype(np.uint8), kernel, iterations=1).astype(bool)
                    for mask in sam_masks
                ])
        observations = []
        records = []
        for detection_id, detection in enumerate(detections):
            box = np.asarray(detection["xyxy"], dtype=np.float32)
            confidence = float(detection["confidence"])
            class_id = int(detection["class_id"])
            class_name = str(detection["class_name"])
            if sam_masks is not None:
                projected = project_mask_depth(
                    depth_m, sam_masks[detection_id], intrinsics,
                    pixel_stride=args.pixel_stride, min_depth_m=0.3, max_depth_m=5.0,
                )
            else:
                projected = project_bbox_depth(
                    depth_m, box.tolist(), (image.shape[1], image.shape[0]), intrinsics,
                    pixel_stride=args.pixel_stride, min_depth_m=0.3, max_depth_m=5.0,
                )
            if projected is None:
                continue
            points_world = transform_camera_to_world(projected.points_camera, pose)
            image_uv = np.rint(projected.image_uv).astype(np.int64)
            image_uv[:, 0] = np.clip(image_uv[:, 0], 0, image.shape[1] - 1)
            image_uv[:, 1] = np.clip(image_uv[:, 1], 0, image.shape[0] - 1)
            colors = image[
                image_uv[:, 1], image_uv[:, 0], ::-1
            ].copy()
            observations.append(ObjectObservation(
                detection_id, int(class_id), class_name, float(confidence),
                float(frame_id), points_world, class_name != "person",
                colors,
            ))
            records.append({
                "frame_id": frame_id, "detection_id": detection_id,
                "class_id": int(class_id), "class_name": class_name,
                "confidence": float(confidence), "xyxy": box.tolist(),
                "forward_point_count": int(points_world.shape[0]),
                "projection_mode": "mobile_sam" if sam_masks is not None else "bbox",
                "sam_score": float(sam_scores[detection_id]) if sam_scores is not None else None,
                "mask_area_pixels": int(np.count_nonzero(sam_masks[detection_id])) if sam_masks is not None else None,
                "observation_centroid_map": np.mean(points_world, axis=0).tolist(),
                "observation_extent_map": np.ptp(points_world, axis=0).tolist(),
            })
        associations = fusion.update(observations)
        for association in associations:
            record = records[association.observation_index]
            record["track_id"] = association.track_id
            record["association_score"] = association.score
            record["geometric_overlap"] = association.geometric_overlap
            record["semantic_similarity"] = association.semantic_similarity
            record["bbox_overlap"] = association.bbox_overlap
            record["is_new_track"] = association.is_new
        detections_json.extend(records)

        elapsed_s = time.perf_counter() - started_at
        frame_s = time.perf_counter() - frame_started_at
        average_s = elapsed_s / processed_count
        remaining_s = average_s * (frame_count - processed_count)
        show_progress = (
            processed_count == 1 or processed_count == frame_count
            or (progress_every > 0 and processed_count % progress_every == 0)
        )
        if show_progress:
            eta = time.strftime("%H:%M:%S", time.gmtime(max(0.0, remaining_s)))
            print(
                f"[{processed_count:>{progress_width}}/{frame_count} "
                f"{processed_count / frame_count * 100:5.1f}%] "
                f"frame={frame_id} det={len(detections)} projected={len(records)} "
                f"tracks={len(fusion.tracks)} frame_s={frame_s:.3f} "
                f"avg_s={average_s:.3f} ETA={eta}",
                flush=True,
            )

    fusion.finalize()
    # 关联完成后，每个持久对象保存一份NPZ和PLY。
    objects_directory = args.output / "objects"
    semantic_records = []
    objects_directory.mkdir(parents=True, exist_ok=True)
    for track in fusion.tracks.values():
        safe_name = "".join(
            char if char.isalnum() or char in ("-", "_") else "_"
            for char in track.class_name
        )
        stem = f"object_{track.track_id:04d}_{safe_name}"
        object_arrays = {
            "points_map": track.points,
            "track_id": np.asarray(track.track_id, dtype=np.int32),
            "class_id": np.asarray(track.class_id, dtype=np.int32),
            "class_name": np.asarray(track.class_name),
            "confidence": np.asarray(track.confidence, dtype=np.float32),
            "observation_count": np.asarray(
                track.observation_count, dtype=np.int32
            ),
            "first_seen": np.asarray(track.first_seen, dtype=np.float64),
            "last_seen": np.asarray(track.last_seen, dtype=np.float64),
            "status": np.asarray(track.status),
            "missed_frames": np.asarray(track.missed_frames, dtype=np.int32),
            "semantic_scores": np.asarray(
                json.dumps(track.semantic_scores, sort_keys=True)
            ),
        }
        if track.colors is not None:
            object_arrays["rgb"] = track.colors
        np.savez_compressed(objects_directory / f"{stem}.npz", **object_arrays)
        save_object_ply(
            objects_directory / f"{stem}.ply", track.points, track.colors
        )
        semantic_records.append(object_record(
            track_id=track.track_id, class_id=track.class_id,
            class_name=track.class_name, confidence=track.confidence,
            observation_count=track.observation_count, first_seen=track.first_seen,
            last_seen=track.last_seen, points=track.points,
            ply_path=f"objects/{stem}.ply", npz_path=f"objects/{stem}.npz",
            source=f"{'mobile_sam' if sam_config is not None else 'bbox'}_object_tracking",
            semantic_scores=track.semantic_scores,
            status=track.status,
            missed_frames=track.missed_frames,
        ))

    semantic_map_path = args.output / "semantic_objects.json"
    write_semantic_object_map(
        semantic_map_path, semantic_records, frame_id="map",
        source="semantic_map_offline",
        metadata={"frame_start": args.start, "frame_count": frame_count},
    )
    (args.output / "associations.json").write_text(
        json.dumps(detections_json, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    summary = {
        "frame_start": args.start,
        "frame_count": frame_count,
        "detection_source": detection_source,
        "input_detections": input_detection_count,
        "detections": len(detections_json),
        "objects": len(fusion.tracks),
        "object_point_clouds": len(fusion.tracks),
        "semantic_objects_json": "semantic_objects.json",
        "fusion_parameters": {
            "voxel_size": args.voxel_size,
            "overlap_radius": args.overlap_radius,
            "max_centroid_distance_m": args.max_centroid_distance_m,
            "min_geometric_overlap": args.min_geometric_overlap,
            "association_threshold": args.association_threshold,
            "geometry_weight": args.geometry_weight,
            "semantic_weight": args.semantic_weight,
        },
        "pose_convention": args.pose_convention,
        "projection_mode": "mobile_sam" if sam_config is not None else "bbox",
        "fusion_mode": "object_tracker",
        "depth_scale_divisor": params["scale"],
        "elapsed_seconds": round(time.perf_counter() - started_at, 3),
    }
    if sam_config is not None:
        summary.update({
            "sam_checkpoint": str(sam_config["checkpoint"]),
            "mask_erode_px": sam_config["mask_erode_px"],
        })
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return summary
