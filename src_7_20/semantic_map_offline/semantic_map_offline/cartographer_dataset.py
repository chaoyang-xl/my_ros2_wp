"""Export rosbag2 RGB-D data using a final optimized Cartographer trajectory."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import json
import os
import shutil
from typing import Any

import cv2
from cv_bridge import CvBridge
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml

from semantic_map_offline.cartographer_trajectory import (
    InterpolatedPose,
    StaticTransformGraph,
    TrajectoryInterpolator,
    matrix_from_pose,
    nearest_index,
)
from semantic_map_offline.compressed_rgbd import (
    decode_16uc1_depth,
    decode_color,
)


@dataclass(frozen=True)
class MessageRef:
    topic: str
    bag_stamp_ns: int
    header_stamp_ns: int
    type_name: str


@dataclass(frozen=True)
class FramePlan:
    index: int
    color: MessageRef
    depth: MessageRef
    pose_map_camera: np.ndarray
    trajectory_pose: InterpolatedPose
    rgb_depth_delta_ns: int
    detection: dict[str, Any]
    detection_delta_ns: int | None


@dataclass(frozen=True)
class ExportConfig:
    bag_uri: Path
    output_directory: Path
    color_topic: str = "/camera/color/image_raw"
    depth_topic: str = "/camera/depth/image_raw"
    camera_info_topic: str = "/camera/color/camera_info"
    static_tf_topic: str = "/tf_static"
    tracking_frame: str = "camera_gyro_frame"
    camera_frame: str = "camera_color_optical_frame"
    jsonl_path: Path | None = None
    max_rgb_depth_delta_ns: int = 50_000_000
    max_detection_delta_ns: int = 150_000_000
    max_trajectory_interval_ns: int = 10_000_000_000
    input_depth_scale: float = 0.001
    start_frame: int = 0
    max_frames: int = 0
    jpeg_quality: int = 95
    overwrite: bool = False
    allow_resolution_mismatch: bool = False


@dataclass
class BagIndex:
    topic_types: dict[str, str]
    color: list[MessageRef]
    depth: list[MessageRef]
    camera_info: list[tuple[int, Any]]
    static_graph: StaticTransformGraph


def stamp_to_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _storage_id(bag_uri: Path) -> str:
    metadata_path = bag_uri / "metadata.yaml"
    if metadata_path.exists():
        payload = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        information = payload.get("rosbag2_bagfile_information", payload)
        storage_id = str(information.get("storage_identifier", "")).strip()
        if storage_id:
            return storage_id
    if bag_uri.suffix == ".mcap":
        return "mcap"
    return "sqlite3"


def _open_reader(bag_uri: Path, topics: list[str]) -> tuple[Any, dict[str, str]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_uri),
            storage_id=_storage_id(bag_uri),
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    missing = [topic for topic in topics if topic not in topic_types]
    if missing:
        raise ValueError(f"Bag is missing required topics: {missing}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader, topic_types


def _deserialize(
    data: bytes,
    type_name: str,
    type_cache: dict[str, type],
) -> Any:
    message_type = type_cache.get(type_name)
    if message_type is None:
        message_type = get_message(type_name)
        type_cache[type_name] = message_type
    return deserialize_message(data, message_type)


def _message_ref(
    topic: str,
    bag_stamp_ns: int,
    type_name: str,
    message: Any,
) -> MessageRef:
    if not hasattr(message, "header"):
        raise ValueError(f"{topic} message has no header")
    return MessageRef(
        topic=topic,
        bag_stamp_ns=int(bag_stamp_ns),
        header_stamp_ns=stamp_to_ns(message.header.stamp),
        type_name=type_name,
    )


def build_bag_index(config: ExportConfig) -> BagIndex:
    """Read only headers/calibration/static TF needed to plan an export."""
    topics = [
        config.color_topic,
        config.depth_topic,
        config.camera_info_topic,
        config.static_tf_topic,
    ]
    reader, topic_types = _open_reader(config.bag_uri, topics)
    type_cache: dict[str, type] = {}
    color: list[MessageRef] = []
    depth: list[MessageRef] = []
    camera_info: list[tuple[int, Any]] = []
    static_graph = StaticTransformGraph()

    while reader.has_next():
        topic, data, bag_stamp_ns = reader.read_next()
        message = _deserialize(data, topic_types[topic], type_cache)
        if topic == config.color_topic:
            color.append(
                _message_ref(topic, bag_stamp_ns, topic_types[topic], message)
            )
        elif topic == config.depth_topic:
            depth.append(
                _message_ref(topic, bag_stamp_ns, topic_types[topic], message)
            )
        elif topic == config.camera_info_topic:
            camera_info.append((stamp_to_ns(message.header.stamp), message))
        elif topic == config.static_tf_topic:
            for transform in message.transforms:
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                static_graph.add(
                    transform.header.frame_id,
                    transform.child_frame_id,
                    matrix_from_pose(
                        (translation.x, translation.y, translation.z),
                        (rotation.x, rotation.y, rotation.z, rotation.w),
                    ),
                )

    color.sort(key=lambda item: item.header_stamp_ns)
    depth.sort(key=lambda item: item.header_stamp_ns)
    camera_info.sort(key=lambda item: item[0])
    if not color or not depth or not camera_info:
        raise ValueError(
            "Bag index is incomplete: "
            f"color={len(color)}, depth={len(depth)}, "
            f"camera_info={len(camera_info)}"
        )
    return BagIndex(topic_types, color, depth, camera_info, static_graph)


def load_detection_jsonl(
    path: Path | None,
) -> tuple[list[int], list[dict[str, Any]]]:
    if path is None:
        return [], []
    if not path.is_file():
        raise FileNotFoundError(f"Detection JSONL does not exist: {path}")
    records: list[tuple[int, dict[str, Any]]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                payload = json.loads(line)
                stamp_ns = (
                    int(payload["stamp_sec"]) * 1_000_000_000
                    + int(payload["stamp_nanosec"])
                )
            except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"Invalid detection JSONL line {line_number}: {exc}"
                ) from exc
            records.append((stamp_ns, payload))
    records.sort(key=lambda item: item[0])
    return [item[0] for item in records], [item[1] for item in records]


def _empty_detection(color_stamp_ns: int) -> dict[str, Any]:
    return {
        "stamp_sec": color_stamp_ns // 1_000_000_000,
        "stamp_nanosec": color_stamp_ns % 1_000_000_000,
        "detections": [],
    }


def create_frame_plans(
    config: ExportConfig,
    index: BagIndex,
    trajectory: TrajectoryInterpolator,
    detection_stamps_ns: list[int],
    detections: list[dict[str, Any]],
) -> tuple[list[FramePlan], dict[str, int]]:
    """Pair frames and compose final T_map_camera transforms."""
    transform_tracking_camera = index.static_graph.lookup(
        config.tracking_frame,
        config.camera_frame,
    )
    color_stamps = [item.header_stamp_ns for item in index.color]
    candidates = index.depth[max(0, config.start_frame):]

    plans: list[FramePlan] = []
    rejected = {
        "depth_candidates": 0,
        "rejected_rgb_sync": 0,
        "rejected_trajectory": 0,
    }
    for depth_ref in candidates:
        if config.max_frames > 0 and len(plans) >= config.max_frames:
            break
        rejected["depth_candidates"] += 1
        color_index, rgb_delta_ns = nearest_index(
            color_stamps,
            depth_ref.header_stamp_ns,
            max_delta_ns=config.max_rgb_depth_delta_ns,
        )
        if color_index is None or rgb_delta_ns is None:
            rejected["rejected_rgb_sync"] += 1
            continue

        trajectory_pose = trajectory.interpolate(
            depth_ref.header_stamp_ns,
            max_interval_ns=config.max_trajectory_interval_ns,
        )
        if trajectory_pose is None:
            rejected["rejected_trajectory"] += 1
            continue

        color_ref = index.color[color_index]
        detection_payload = _empty_detection(color_ref.header_stamp_ns)
        detection_delta_ns: int | None = None
        if detection_stamps_ns:
            detection_index, detection_delta_ns = nearest_index(
                detection_stamps_ns,
                color_ref.header_stamp_ns,
                max_delta_ns=config.max_detection_delta_ns,
            )
            if detection_index is not None:
                detection_payload = dict(detections[detection_index])
            else:
                detection_delta_ns = None

        plans.append(
            FramePlan(
                index=len(plans),
                color=color_ref,
                depth=depth_ref,
                pose_map_camera=(
                    trajectory_pose.matrix @ transform_tracking_camera
                ),
                trajectory_pose=trajectory_pose,
                rgb_depth_delta_ns=int(rgb_delta_ns),
                detection=detection_payload,
                detection_delta_ns=detection_delta_ns,
            )
        )
    rejected["accepted_frames"] = len(plans)
    return plans, rejected


def _decode_color(message: Any, type_name: str, bridge: CvBridge) -> np.ndarray:
    if type_name == "sensor_msgs/msg/CompressedImage":
        return decode_color(message.data)
    if type_name != "sensor_msgs/msg/Image":
        raise ValueError(f"Unsupported color message type: {type_name}")
    image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
    return np.asarray(image, dtype=np.uint8)


def _decode_depth_mm(
    message: Any,
    type_name: str,
    bridge: CvBridge,
    input_depth_scale: float,
) -> np.ndarray:
    if type_name == "sensor_msgs/msg/CompressedImage":
        depth = decode_16uc1_depth(message.data, message.format)
    elif type_name == "sensor_msgs/msg/Image":
        depth = np.asarray(
            bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        )
    else:
        raise ValueError(f"Unsupported depth message type: {type_name}")
    if depth.ndim != 2:
        raise ValueError(f"Depth image must be 2D, got {depth.shape}")
    depth_m = depth.astype(np.float64) * float(input_depth_scale)
    valid = np.isfinite(depth_m) & (depth_m > 0.0)
    depth_mm = np.zeros(depth.shape, dtype=np.uint16)
    depth_mm[valid] = np.clip(
        np.rint(depth_m[valid] * 1000.0),
        1,
        np.iinfo(np.uint16).max,
    ).astype(np.uint16)
    return depth_mm


def _prepare_output(path: Path, overwrite: bool) -> tuple[Path, Path]:
    target = path.expanduser().resolve()
    partial = target.with_name(target.name + ".partial")
    for candidate in (target, partial):
        if candidate.exists() and any(candidate.iterdir()):
            if not overwrite:
                raise FileExistsError(
                    f"Output is not empty: {candidate}; set overwrite:=true"
                )
            shutil.rmtree(candidate)
        elif candidate.exists():
            candidate.rmdir()
    partial.mkdir(parents=True)
    for name in ("results", "pose", "detections"):
        (partial / name).mkdir()
    return target, partial


def _nearest_camera_info(
    camera_info: list[tuple[int, Any]],
    stamp_ns: int,
) -> Any:
    stamps = [item[0] for item in camera_info]
    index, _ = nearest_index(stamps, stamp_ns)
    if index is None:
        raise ValueError("No CameraInfo is available")
    return camera_info[index][1]


def _write_json(path: Path, payload: Any) -> None:
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def export_dataset(
    config: ExportConfig,
    trajectory: TrajectoryInterpolator,
    *,
    trajectory_id: int,
    map_frame: str,
) -> dict[str, Any]:
    """Export a Replica-compatible dataset and return its report."""
    index = build_bag_index(config)
    detection_stamps, detections = load_detection_jsonl(config.jsonl_path)
    plans, planning_stats = create_frame_plans(
        config,
        index,
        trajectory,
        detection_stamps,
        detections,
    )
    if not plans:
        raise RuntimeError(f"No frames passed export planning: {planning_stats}")

    target, partial = _prepare_output(
        config.output_directory,
        config.overwrite,
    )
    bridge = CvBridge()
    color_targets: dict[int, list[FramePlan]] = {}
    depth_targets: dict[int, list[FramePlan]] = {}
    for plan in plans:
        color_targets.setdefault(plan.color.bag_stamp_ns, []).append(plan)
        depth_targets.setdefault(plan.depth.bag_stamp_ns, []).append(plan)

    topics = [config.color_topic, config.depth_topic]
    reader, topic_types = _open_reader(config.bag_uri, topics)
    type_cache: dict[str, type] = {}
    written_color: set[int] = set()
    written_depth: set[int] = set()
    color_shapes: dict[int, tuple[int, int]] = {}
    depth_shapes: dict[int, tuple[int, int]] = {}

    try:
        while reader.has_next():
            topic, data, bag_stamp_ns = reader.read_next()
            matching = (
                color_targets.get(bag_stamp_ns)
                if topic == config.color_topic
                else depth_targets.get(bag_stamp_ns)
            )
            if not matching:
                continue
            message = _deserialize(data, topic_types[topic], type_cache)
            if topic == config.color_topic:
                image = _decode_color(message, topic_types[topic], bridge)
                for plan in matching:
                    output_path = (
                        partial / "results" / f"frame{plan.index:06d}.jpg"
                    )
                    if not cv2.imwrite(
                        str(output_path),
                        image,
                        [cv2.IMWRITE_JPEG_QUALITY, config.jpeg_quality],
                    ):
                        raise OSError(f"Could not write {output_path}")
                    color_shapes[plan.index] = (image.shape[1], image.shape[0])
                    written_color.add(plan.index)
            else:
                depth_mm = _decode_depth_mm(
                    message,
                    topic_types[topic],
                    bridge,
                    config.input_depth_scale,
                )
                for plan in matching:
                    output_path = (
                        partial / "results" / f"depth{plan.index:06d}.png"
                    )
                    if not cv2.imwrite(str(output_path), depth_mm):
                        raise OSError(f"Could not write {output_path}")
                    depth_shapes[plan.index] = (
                        depth_mm.shape[1],
                        depth_mm.shape[0],
                    )
                    written_depth.add(plan.index)

        expected = set(range(len(plans)))
        if written_color != expected or written_depth != expected:
            raise RuntimeError(
                "Second bag pass missed selected messages: "
                f"color={len(written_color)}/{len(plans)}, "
                f"depth={len(written_depth)}/{len(plans)}"
            )

        mismatched = [
            frame_id
            for frame_id in range(len(plans))
            if color_shapes[frame_id] != depth_shapes[frame_id]
        ]
        if mismatched and not config.allow_resolution_mismatch:
            raise ValueError(
                f"RGB/depth resolution mismatch in {len(mismatched)} frames; "
                "registered projection requires equal sizes. "
                "Set allow_resolution_mismatch:=true only for diagnostics."
            )

        first_info = _nearest_camera_info(
            index.camera_info,
            plans[0].color.header_stamp_ns,
        )
        camera_width, camera_height = color_shapes[0]
        camera_payload = {
            "camera": {
                "w": camera_width,
                "h": camera_height,
                "fx": float(first_info.k[0]),
                "fy": float(first_info.k[4]),
                "cx": float(first_info.k[2]),
                "cy": float(first_info.k[5]),
                "scale": 1000.0,
            }
        }
        _write_json(partial / "cam_params.json", camera_payload)

        pose_rows = []
        timestamps = []
        for plan in plans:
            pose_rows.append(plan.pose_map_camera)
            np.savetxt(
                partial / "pose" / f"{plan.index:06d}.txt",
                plan.pose_map_camera,
                fmt="%.10f",
            )
            detection_payload = dict(plan.detection)
            detection_payload["export_frame_index"] = plan.index
            _write_json(
                partial / "detections" / f"{plan.index:06d}.json",
                detection_payload,
            )
            timestamps.append({
                "frame_index": plan.index,
                "color_stamp_ns": plan.color.header_stamp_ns,
                "depth_stamp_ns": plan.depth.header_stamp_ns,
                "rgb_depth_delta_ns": plan.rgb_depth_delta_ns,
                "detection_delta_ns": plan.detection_delta_ns,
                "trajectory_lower_stamp_ns": (
                    plan.trajectory_pose.lower_stamp_ns
                ),
                "trajectory_upper_stamp_ns": (
                    plan.trajectory_pose.upper_stamp_ns
                ),
                "trajectory_alpha": plan.trajectory_pose.alpha,
            })
        np.savetxt(
            partial / "traj.txt",
            np.vstack(pose_rows),
            fmt="%.10f",
        )
        _write_json(partial / "timestamps.json", timestamps)

        intervals = np.asarray(
            [plan.trajectory_pose.interval_ns for plan in plans],
            dtype=np.float64,
        )
        rgb_deltas = np.asarray(
            [abs(plan.rgb_depth_delta_ns) for plan in plans],
            dtype=np.float64,
        )
        detection_matches = sum(
            plan.detection_delta_ns is not None for plan in plans
        )
        report = {
            "schema_version": 1,
            "status": "complete",
            "bag_uri": str(config.bag_uri),
            "output_directory": str(target),
            "trajectory_id": int(trajectory_id),
            "map_frame": map_frame,
            "tracking_frame": config.tracking_frame,
            "camera_frame": config.camera_frame,
            "pose_convention": "T_map_camera OpenCV optical camera-to-world",
            "topics": {
                "color": config.color_topic,
                "depth": config.depth_topic,
                "camera_info": config.camera_info_topic,
                "static_tf": config.static_tf_topic,
            },
            "source_counts": {
                "color": len(index.color),
                "depth": len(index.depth),
                "camera_info": len(index.camera_info),
                "detections": len(detections),
            },
            "planning": planning_stats,
            "exported_frames": len(plans),
            "detection_matches": detection_matches,
            "rgb_depth_delta_ms": {
                "median": float(np.median(rgb_deltas) / 1e6),
                "max": float(np.max(rgb_deltas) / 1e6),
            },
            "trajectory_interval_ms": {
                "median": float(np.median(intervals) / 1e6),
                "max": float(np.max(intervals) / 1e6),
            },
            "resolution": {
                "color": list(color_shapes[0]),
                "depth": list(depth_shapes[0]),
                "mismatched_frames": len(mismatched),
            },
            "camera": camera_payload["camera"],
            "static_tf_frames": list(index.static_graph.frames),
        }
        metadata = {
            "schema_version": 1,
            "dataset_type": "cartographer_final_trajectory_rgbd",
            "frame_count": len(plans),
            "pose_file": "traj.txt",
            "camera_file": "cam_params.json",
            "image_pattern": "results/frame%06d.jpg",
            "depth_pattern": "results/depth%06d.png",
            "detection_pattern": "detections/%06d.json",
            "individual_pose_pattern": "pose/%06d.txt",
            "depth_unit": "millimeter",
            "coordinate_convention": {
                "pose": "T_map_camera",
                "camera": "ROS optical/OpenCV x-right y-down z-forward",
                "map": map_frame,
            },
        }
        _write_json(partial / "metadata.json", metadata)
        _write_json(partial / "export_report.json", report)

        if target.exists():
            target.rmdir()
        os.replace(partial, target)
        return report
    except Exception:
        # Keep partial output for diagnosis; a future run requires overwrite.
        raise
