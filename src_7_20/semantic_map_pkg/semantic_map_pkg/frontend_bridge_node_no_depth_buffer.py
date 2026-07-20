#!/usr/bin/env python3
"""ROS 2 node: Front-end detection JSON + Depth Image -> map-frame semantic seed JSON.

This node bridges custom front-end (OrangePi+YOLO) detection JSON to the
existing semantic projection pipeline.  It reads richer structured data
(detection bboxes, pose keypoints, posture, action tags) and produces
seeds compatible with ``semantic_projection_node`` on ``/semantic_seed``.
"""

from __future__ import annotations

import json

import rclpy
import tf2_geometry_msgs  # noqa: F401 Needed so Buffer can transform PointStamped.
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import message_filters
import numpy as np

from semantic_map_pkg.yolo_seed_projection import (
    YoloDetection2D,
    YoloLidarSeedProjector,
)


class FrontendBridgeNode(Node):
    """Parse front-end JSON detections + depth images, publish map-frame seeds."""

    def __init__(self) -> None:
        super().__init__("frontend_bridge_node")

        # Camera intrinsics (override to match your real / simulated camera)
        # self.declare_parameter("camera_fx", 609.818)
        # self.declare_parameter("camera_fy", 609.572)
        # self.declare_parameter("camera_cx", 639.986)
        # self.declare_parameter("camera_cy", 362.435)

        self.declare_parameter("camera_fx", 320.0)#模拟器测试参数，实际使用时请修改为你的相机内参
        self.declare_parameter("camera_fy", 320.0)
        self.declare_parameter("camera_cx", 320.0)
        self.declare_parameter("camera_cy", 240.0)
        # Depth scale: 16UC1 (mm) → 0.001; 32FC1 (m) → 1.0
        #self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("depth_scale", 1.0)#模拟器测试参数，实际使用时请修改为你的相机内参

        self.declare_parameter("target_frame", "map")
        self.declare_parameter("tf_timeout_s", 0.1)
        # Topic remapping
        self.declare_parameter("input_topic", "/yolo/results_json")
        self.declare_parameter("depth_topic", "/camera/depth_image")
        self.declare_parameter("camera_frame", "")  # override depth_msg.header.frame_id
        # Time sync
        self.declare_parameter("slop", 0.1)
        # Pose matching
        self.declare_parameter("pose_iou_threshold", 0.3)
        self.declare_parameter("enable_pose", True)

        self._target_frame = self.get_parameter("target_frame").value
        self._tf_timeout_s = self.get_parameter("tf_timeout_s").value
        self._depth_scale = self.get_parameter("depth_scale").value
        self._pose_iou_threshold = self.get_parameter("pose_iou_threshold").value
        self._enable_pose = self.get_parameter("enable_pose").value

        self.cv_bridge = CvBridge()
        self.projector = YoloLidarSeedProjector(
            fx=self.get_parameter("camera_fx").value,
            fy=self.get_parameter("camera_fy").value,
            cx=self.get_parameter("camera_cx").value,
            cy=self.get_parameter("camera_cy").value,
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # QoS profiles — BEST_EFFORT for sensor data (Gazebo default)
        image_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        string_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        input_topic = self.get_parameter("input_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        slop = self.get_parameter("slop").value
        self._latest_depth_msg = None
        self.create_subscription(
                    Image,
                    depth_topic,
                    self._depth_cb,
                    image_qos,
                )
        self.create_subscription(
                    String,
                    input_topic,
                    self._json_cb,
                    string_qos,
                )
        # self._json_sub = message_filters.Subscriber(
        #     self, String, input_topic, qos_profile=string_qos
        # )
        # self._depth_sub = message_filters.Subscriber(
        #     self, Image, depth_topic, qos_profile=image_qos
        # )

        # self._sync = message_filters.ApproximateTimeSynchronizer(
        #     [self._json_sub, self._depth_sub],
        #     queue_size=10,
        #     slop=slop,
        #     allow_headerless=True,
        # )
        #self._sync.registerCallback(self._synced_cb)

        self._pub = self.create_publisher(String, "/semantic_seed", 10)

        self.get_logger().info(
            f"Frontend Bridge Node Ready.  Listening on '{input_topic}' + '{depth_topic}'"
        )

    # ------------------------------------------------------------------
    # Callback
    # ------------------------------------------------------------------
    def _depth_cb(self, msg: Image):

        self._latest_depth_msg = msg

    def _json_cb(self, json_msg: String):

        if self._latest_depth_msg is None:
            self.get_logger().info("Received JSON but no depth image yet")
            return

        self._synced_cb(
            json_msg,
            self._latest_depth_msg
        )

    def _synced_cb(self, json_msg: String, depth_msg: Image) -> None:
        """Sync callback: parse JSON, project detections, publish seeds."""
        #self.get_logger().info("SYNC CALLBACK TRIGGERED")
        # 1. Parse JSON -------------------------------------------------------
        try:
            payload = json.loads(json_msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Invalid JSON from front-end: {exc}")
            return

        detections: list[dict] = payload.get("detections", [])
        poses: list[dict] = payload.get("poses", [])

        if not detections:

            #self.get_logger().info("No detections in JSON payload")
            return

        # 2. Convert depth image to metres ------------------------------------
        try:
            cv_depth = self.cv_bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
            )
        except Exception as exc:
            self.get_logger().error(f"CV Bridge Error: {exc}")
            return

        depth_image_meters = cv_depth * self._depth_scale
        camera_frame = (
            self.get_parameter("camera_frame").value
            or depth_msg.header.frame_id
        )

        # 3. Build pose matching pool (one entry per person pose box) ---------
        pose_pool: list[tuple[list[float], dict]] = []
        if self._enable_pose and poses:
            for pose in poses:
                xyxy = pose.get("xyxy")
                if xyxy and len(xyxy) == 4:
                    pose_pool.append((xyxy, pose))

        # 4. Process each detection -------------------------------------------
        for det in detections:
            xyxy = det.get("xyxy")
            if not xyxy or len(xyxy) != 4:
                continue

            class_name = det.get("class_name", "unknown")
            confidence = float(det.get("confidence", 0.0))
            class_id = det.get("class_id", -1)

            # Compute bbox centre from xyxy
            center_u = (xyxy[0] + xyxy[2]) / 2.0
            center_v = (xyxy[1] + xyxy[3]) / 2.0

            det2d = YoloDetection2D(
                label=class_name,
                confidence=confidence,
                center_u=center_u,
                center_v=center_v,
                image_width=depth_msg.width,
                image_height=depth_msg.height,
                track_id=None,  # front-end does not provide track IDs
            )

            # 4a. Project 2D → 3D (camera optical frame) -------------------
            seed_camera = self.projector.project(det2d, depth_image_meters)
            if seed_camera is None:
                self.get_logger().warn(
                    f"Depth projection failed for '{class_name}' at ({center_u:.0f}, {center_v:.0f})"
                )
                continue

            # 4b. Transform camera → map via TF ----------------------------
            map_point = self._transform_to_map(
                seed_camera.x,
                seed_camera.y,
                seed_camera.z,
                camera_frame,
                depth_msg.header.stamp,
            )
            if map_point is None:
                self.get_logger().warn(
                    f"TF failed for '{class_name}': {camera_frame} -> {self._target_frame}"
                )
                continue

            gx, gy = map_point

            # 4c. Match pose (greedy IoU for person detections) ------------
            pose_info: dict | None = None
            if class_id == 0 and pose_pool:
                best_idx: int | None = None
                best_iou: float = 0.0
                for idx, (pose_xyxy, _pose_data) in enumerate(pose_pool):
                    iou = self._compute_iou(xyxy, pose_xyxy)
                    if iou > best_iou:
                        best_iou = iou
                        best_idx = idx
                if best_idx is not None and best_iou >= self._pose_iou_threshold:
                    _, matched_pose = pose_pool.pop(best_idx)
                    pose_info = self._extract_pose_summary(matched_pose)

            # 4d. Build and publish seed JSON ------------------------------
            seed = {
                "label": class_name,
                "confidence": confidence,
                "gx": gx,
                "gy": gy,
                "class_id": class_id,
            }
            # 空摘要不算有效姿态，避免下游错误刷新 pose_last_seen。
            if pose_info:
                seed["pose"] = pose_info

            self._pub.publish(String(data=json.dumps(seed)))
            self.get_logger().info(
                f"Published seed: '{class_name}' at ({gx:.2f}, {gy:.2f}) with confidence {confidence:.2f}"
            )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _transform_to_map(
        self, x: float, y: float, z: float, source_frame: str , depth_msg_header_stamp
    ) -> tuple[float, float] | None:
        """Transform a point from the camera optical frame into the map frame."""
        point = PointStamped()
        point.header.stamp = rclpy.time.Time(seconds=0).to_msg()
        #point.header.stamp = depth_msg_header_stamp #使用深度图像的时间戳，确保TF变换的时间一致性
        point.header.frame_id = source_frame
        point.point.x = x
        point.point.y = y
        point.point.z = z

        try:
            transformed = self._tf_buffer.transform(
                point,
                self._target_frame,
                timeout=Duration(seconds=self._tf_timeout_s),
            )
        except Exception as exc:
            self.get_logger().debug(
                f"TF {source_frame} -> {self._target_frame} failed: {exc}"
            )
            return None

        return float(transformed.point.x), float(transformed.point.y)

    @staticmethod
    def _compute_iou(box1: list[float], box2: list[float]) -> float:
        """Intersection-over-Union for two axis-aligned boxes in xyxy format."""
        x_left = max(box1[0], box2[0])
        y_top = max(box1[1], box2[1])
        x_right = min(box1[2], box2[2])
        y_bottom = min(box1[3], box2[3])

        if x_right <= x_left or y_bottom <= y_top:
            return 0.0

        inter_area = (x_right - x_left) * (y_bottom - y_top)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union_area = area1 + area2 - inter_area

        return inter_area / union_area if union_area > 0.0 else 0.0

    @staticmethod
    def _extract_pose_summary(pose_data: dict) -> dict:
        """Extract a compact pose summary for inclusion in the seed JSON."""
        summary: dict = {}

        posture = pose_data.get("posture")
        if posture:
            summary["posture"] = posture
            summary["posture_confidence"] = pose_data.get("posture_confidence", 0.0)

        action_tags = pose_data.get("action_tags", [])
        if action_tags:
            summary["action_tags"] = action_tags

        # Filter keypoints to only include those with meaningful confidence
        keypoints = pose_data.get("keypoints", [])
        if keypoints:
            filtered = [
                {
                    "name": kp["name"],
                    "x": kp["x"],
                    "y": kp["y"],
                    "confidence": kp.get("confidence", 0.0),
                }
                for kp in keypoints
                if kp.get("confidence", 0.0) > 0.1
            ]
            if filtered:
                summary["keypoints"] = filtered

        return summary


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FrontendBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
