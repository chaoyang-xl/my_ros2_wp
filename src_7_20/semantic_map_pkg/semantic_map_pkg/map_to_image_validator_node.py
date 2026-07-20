#!/usr/bin/env python3
"""Overlay map-frame semantic points back onto camera images.

This is an offline-friendly validator: play a rosbag, run the semantic mapping
pipeline, then run this node to save RGB images with projected map points.
Because the semantic output stores only map x/y, not height, the validator
projects a short vertical column at each map point.  If that column cuts through
the detected object in the image, the map projection is geometrically plausible.
"""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  Required for tf2 PointStamped support.
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


@dataclass
class MapPoint:
    source: str
    label: str
    x: float
    y: float
    confidence: float | None = None
    object_id: str | None = None
    state: str | None = None


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass
class DetectionBox:
    label: str
    xyxy: tuple[float, float, float, float]
    confidence: float | None = None
    track_id: str | None = None


class MapToImageValidatorNode(Node):
    """Project /semantic_seed and /semantic_objects map points onto RGB frames."""

    def __init__(self) -> None:
        super().__init__("map_to_image_validator_node")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("detection_topic", "/yolo/results_json")
        self.declare_parameter("semantic_seed_topic", "/semantic_seed")
        self.declare_parameter("semantic_objects_topic", "/semantic_objects")
        self.declare_parameter("target_camera_frame", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("output_dir", "/tmp/map_to_image_validation")
        self.declare_parameter("save_every_n", 2)
        self.declare_parameter("draw_seed", True)
        self.declare_parameter("draw_objects", True)
        self.declare_parameter("draw_detections", True)
        self.declare_parameter("point_ttl_s", 2.0)
        self.declare_parameter("detection_ttl_s", 2.0)
        self.declare_parameter("tf_timeout_s", 0.2)
        self.declare_parameter("use_latest_tf", True)
        self.declare_parameter("camera_fx", 0.0)
        self.declare_parameter("camera_fy", 0.0)
        self.declare_parameter("camera_cx", 0.0)
        self.declare_parameter("camera_cy", 0.0)
        self.declare_parameter("z_min_m", 0.0)
        self.declare_parameter("z_max_m", 1.4)
        self.declare_parameter("z_step_m", 0.2)

        self._bridge = CvBridge()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._intrinsics: Intrinsics | None = self._intrinsics_from_params()
        self._latest_seed_points: list[MapPoint] = []
        self._latest_object_points: list[MapPoint] = []
        self._latest_detections: list[DetectionBox] = []
        self._last_seed_sec = 0.0
        self._last_object_sec = 0.0
        self._last_detection_sec = 0.0
        self._image_count = 0
        self._saved_count = 0

        self._output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._csv_path = self._output_dir / "projection_report.csv"
        self._csv_file = self._csv_path.open("w", newline="", encoding="utf-8")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(
            [
                "image_file",
                "stamp_sec",
                "source",
                "id",
                "label",
                "map_x",
                "map_y",
                "projected_pixels",
                "status",
            ]
        )

        image_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        info_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        string_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._image_cb,
            image_qos,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._camera_info_cb,
            info_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("detection_topic").value),
            self._detection_cb,
            string_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("semantic_seed_topic").value),
            self._seed_cb,
            string_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("semantic_objects_topic").value),
            self._objects_cb,
            string_qos,
        )

        self.get_logger().info(
            f"Map-to-image validator saving overlays to {self._output_dir}"
        )

    def destroy_node(self) -> bool:
        try:
            self._csv_file.close()
        finally:
            return super().destroy_node()

    def _intrinsics_from_params(self) -> Intrinsics | None:
        fx = float(self.get_parameter("camera_fx").value)
        fy = float(self.get_parameter("camera_fy").value)
        cx = float(self.get_parameter("camera_cx").value)
        cy = float(self.get_parameter("camera_cy").value)
        if fx > 0.0 and fy > 0.0:
            return Intrinsics(fx=fx, fy=fy, cx=cx, cy=cy)
        return None

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self._intrinsics = Intrinsics(
                fx=float(msg.k[0]),
                fy=float(msg.k[4]),
                cx=float(msg.k[2]),
                cy=float(msg.k[5]),
            )

    def _seed_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._latest_seed_points = [
                MapPoint(
                    source="seed",
                    label=str(data.get("label", "unknown")),
                    x=float(data["gx"]),
                    y=float(data["gy"]),
                    confidence=(
                        float(data["confidence"])
                        if data.get("confidence") is not None
                        else None
                    ),
                )
            ]
            self._last_seed_sec = self._now_sec()
        except Exception as exc:
            self.get_logger().warn(f"Invalid semantic seed JSON: {exc}")

    def _objects_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            objects = data.get("objects", [])
            points: list[MapPoint] = []
            for obj in objects:
                points.append(
                    MapPoint(
                        source="object",
                        label=str(obj.get("label", "unknown")),
                        x=float(obj["x"]),
                        y=float(obj["y"]),
                        confidence=(
                            float(obj["confidence"])
                            if obj.get("confidence") is not None
                            else None
                        ),
                        object_id=str(obj.get("id", "")),
                        state=str(obj.get("state", "")),
                    )
                )
            self._latest_object_points = points
            self._last_object_sec = self._now_sec()
        except Exception as exc:
            self.get_logger().warn(f"Invalid semantic objects JSON: {exc}")

    def _detection_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            detections = data.get("detections", [])
            boxes: list[DetectionBox] = []
            for det in detections:
                xyxy = self._extract_xyxy(det)
                if xyxy is None:
                    continue
                boxes.append(
                    DetectionBox(
                        label=str(det.get("class_name", det.get("label", "unknown"))),
                        xyxy=xyxy,
                        confidence=(
                            float(det["confidence"])
                            if det.get("confidence") is not None
                            else None
                        ),
                        track_id=(
                            str(det.get("track_id"))
                            if det.get("track_id") is not None
                            else None
                        ),
                    )
                )
            self._latest_detections = boxes
            self._last_detection_sec = self._now_sec()
        except Exception as exc:
            self.get_logger().warn(f"Invalid detection JSON: {exc}")

    def _image_cb(self, msg: Image) -> None:
        self._image_count += 1
        save_every_n = max(1, int(self.get_parameter("save_every_n").value))
        if self._image_count % save_every_n != 0:
            return
        if self._intrinsics is None:
            self.get_logger().warn(
                "No CameraInfo yet. Provide camera_info or camera_fx/fy/cx/cy."
            )
            return

        points = self._active_points()
        detections = self._active_detections()
        if not points and not detections:
            return

        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert image: {exc}")
            return

        stamp_sec = self._stamp_sec(msg)
        image_file = self._output_dir / f"overlay_{self._saved_count:06d}_{stamp_sec:.3f}.png"
        camera_frame = (
            str(self.get_parameter("target_camera_frame").value).strip()
            or msg.header.frame_id
        )

        drawn = 0
        for point in points:
            pixels, status = self._project_map_column(point, camera_frame, msg)
            if pixels:
                color = (0, 0, 255) if point.source == "seed" else (0, 220, 0)
                self._draw_projected_column(image, point, pixels, color)
                drawn += 1
            self._csv.writerow(
                [
                    image_file.name,
                    f"{stamp_sec:.9f}",
                    point.source,
                    point.object_id or "",
                    point.label,
                    f"{point.x:.4f}",
                    f"{point.y:.4f}",
                    len(pixels),
                    status,
                ]
            )

        boxes_drawn = 0
        if bool(self.get_parameter("draw_detections").value):
            for det in detections:
                if self._draw_detection_box(image, det):
                    boxes_drawn += 1

        if drawn == 0 and boxes_drawn == 0:
            return

        cv2.putText(
            image,
            "cyan=detection box  red=/semantic_seed  green=/semantic_objects  line=z sweep",
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imwrite(str(image_file), image)
        self._csv_file.flush()
        self._saved_count += 1
        self.get_logger().info(
            f"saved {image_file} with {drawn} projected point(s), "
            f"{boxes_drawn} detection box(es)"
        )

    def _active_points(self) -> list[MapPoint]:
        now_sec = self._now_sec()
        ttl = float(self.get_parameter("point_ttl_s").value)
        points: list[MapPoint] = []
        if bool(self.get_parameter("draw_seed").value):
            if ttl <= 0.0 or now_sec - self._last_seed_sec <= ttl:
                points.extend(self._latest_seed_points)
        if bool(self.get_parameter("draw_objects").value):
            if ttl <= 0.0 or now_sec - self._last_object_sec <= ttl:
                points.extend(self._latest_object_points)
        return points

    def _active_detections(self) -> list[DetectionBox]:
        if not bool(self.get_parameter("draw_detections").value):
            return []
        now_sec = self._now_sec()
        ttl = float(self.get_parameter("detection_ttl_s").value)
        if ttl > 0.0 and now_sec - self._last_detection_sec > ttl:
            return []
        return list(self._latest_detections)

    def _project_map_column(
        self,
        point: MapPoint,
        camera_frame: str,
        image_msg: Image,
    ) -> tuple[list[tuple[int, int]], str]:
        zs = list(self._z_samples())
        pixels: list[tuple[int, int]] = []
        statuses: list[str] = []
        for z in zs:
            pixel, status = self._project_map_point(point.x, point.y, z, camera_frame, image_msg)
            statuses.append(status)
            if pixel is not None:
                u, v = pixel
                if 0 <= u < image_msg.width and 0 <= v < image_msg.height:
                    pixels.append((u, v))
        if pixels:
            return pixels, "ok"
        if statuses:
            return pixels, statuses[-1]
        return pixels, "no_z_samples"

    def _project_map_point(
        self,
        x: float,
        y: float,
        z: float,
        camera_frame: str,
        image_msg: Image,
    ) -> tuple[tuple[int, int] | None, str]:
        point = PointStamped()
        point.header.frame_id = str(self.get_parameter("map_frame").value)
        if bool(self.get_parameter("use_latest_tf").value):
            point.header.stamp = rclpy.time.Time(seconds=0).to_msg()
        else:
            point.header.stamp = image_msg.header.stamp
        point.point.x = float(x)
        point.point.y = float(y)
        point.point.z = float(z)

        try:
            transformed = self._tf_buffer.transform(
                point,
                camera_frame,
                timeout=Duration(seconds=float(self.get_parameter("tf_timeout_s").value)),
            )
        except Exception as exc:
            return None, f"tf_failed:{type(exc).__name__}"

        intr = self._intrinsics
        if intr is None:
            return None, "no_intrinsics"

        z_cam = float(transformed.point.z)
        if z_cam <= 0.05:
            return None, "behind_camera"
        u = intr.fx * float(transformed.point.x) / z_cam + intr.cx
        v = intr.fy * float(transformed.point.y) / z_cam + intr.cy
        if not np.isfinite(u) or not np.isfinite(v):
            return None, "nonfinite_pixel"
        return (int(round(u)), int(round(v))), "ok"

    def _z_samples(self) -> Iterable[float]:
        z_min = float(self.get_parameter("z_min_m").value)
        z_max = float(self.get_parameter("z_max_m").value)
        z_step = max(0.01, float(self.get_parameter("z_step_m").value))
        z = z_min
        while z <= z_max + 1e-9:
            yield z
            z += z_step

    @staticmethod
    def _draw_projected_column(
        image: np.ndarray,
        point: MapPoint,
        pixels: list[tuple[int, int]],
        color: tuple[int, int, int],
    ) -> None:
        if len(pixels) >= 2:
            cv2.polylines(image, [np.array(pixels, dtype=np.int32)], False, color, 2)
        for u, v in pixels:
            cv2.circle(image, (u, v), 4, color, -1)
        u0, v0 = pixels[0]
        name = point.object_id or point.label
        text = f"{point.source}:{name}"
        cv2.putText(
            image,
            text,
            (u0 + 6, max(18, v0 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

    @staticmethod
    def _draw_detection_box(image: np.ndarray, det: DetectionBox) -> bool:
        h, w = image.shape[:2]
        x1, y1, x2, y2 = det.xyxy
        x1_i = int(round(max(0, min(w - 1, x1))))
        y1_i = int(round(max(0, min(h - 1, y1))))
        x2_i = int(round(max(0, min(w - 1, x2))))
        y2_i = int(round(max(0, min(h - 1, y2))))
        if x2_i <= x1_i or y2_i <= y1_i:
            return False

        color = (255, 220, 0)
        cv2.rectangle(image, (x1_i, y1_i), (x2_i, y2_i), color, 2)
        label = det.label
        if det.confidence is not None:
            label += f" {det.confidence:.2f}"
        if det.track_id:
            label += f" #{det.track_id}"
        text_origin = (x1_i, max(18, y1_i - 6))
        cv2.putText(
            image,
            label,
            text_origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
        return True

    @staticmethod
    def _extract_xyxy(det: dict) -> tuple[float, float, float, float] | None:
        xyxy = det.get("xyxy")
        if isinstance(xyxy, list) and len(xyxy) == 4:
            return tuple(float(v) for v in xyxy)  # type: ignore[return-value]

        bbox = det.get("bbox")
        if isinstance(bbox, list) and len(bbox) == 4:
            x, y, w, h = (float(v) for v in bbox)
            return x, y, x + w, y + h

        box = det.get("box")
        if isinstance(box, dict):
            if all(k in box for k in ("x1", "y1", "x2", "y2")):
                return (
                    float(box["x1"]),
                    float(box["y1"]),
                    float(box["x2"]),
                    float(box["y2"]),
                )
            if all(k in box for k in ("x", "y", "w", "h")):
                x = float(box["x"])
                y = float(box["y"])
                return x, y, x + float(box["w"]), y + float(box["h"])

        return None

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_sec(msg: Image) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MapToImageValidatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
