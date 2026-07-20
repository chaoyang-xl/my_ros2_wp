#!/usr/bin/env python3
"""ROS 2 MobileSAM instance-mask depth projection node."""

from __future__ import annotations

from pathlib import Path
import sys

import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from semantic_map_offline.mask_projection import ProjectedMask, project_mask_depth
from semantic_map_offline.offline_projector_node import OfflineProjectorNode


class SamOfflineProjectorNode(OfflineProjectorNode):
    """Project MobileSAM masks; bbox ROS projector remains dependency-free."""

    def __init__(self) -> None:
        super().__init__(node_name="semantic_map_offline_sam_projector")
        for name, default in (
            ("sam_checkpoint", ""),
            ("sam_source", ""),
            ("sam_device", ""),
            ("mask_erode_px", 2),
            ("publish_debug_image", True),
            ("debug_image_topic", "/semantic_offline/sam_debug_image"),
            ("debug_mask_alpha", 0.45),
        ):
            self.declare_parameter(name, default)

        checkpoint = str(self.get_parameter("sam_checkpoint").value).strip()
        if not checkpoint:
            raise ValueError("sam_checkpoint is required for sam_offline_projector_node")
        source = str(self.get_parameter("sam_source").value).strip()
        if source:
            sys.path.insert(0, str(Path(source).expanduser()))

        import torch
        from mobile_sam import SamPredictor, sam_model_registry

        self._sam_device = str(self.get_parameter("sam_device").value).strip()
        if not self._sam_device:
            self._sam_device = "cuda" if torch.cuda.is_available() else "cpu"
        model = sam_model_registry["vit_t"](checkpoint=str(Path(checkpoint).expanduser()))
        model.to(device=self._sam_device)
        model.eval()
        self._sam_predictor = SamPredictor(model)
        self._mask_erode_px = max(0, int(self.get_parameter("mask_erode_px").value))
        self._publish_debug_image = bool(
            self.get_parameter("publish_debug_image").value
        )
        self._debug_mask_alpha = float(
            np.clip(self.get_parameter("debug_mask_alpha").value, 0.0, 1.0)
        )
        self._debug_image_pub = None
        if self._publish_debug_image:
            sensor_qos = QoSProfile(
                depth=2, reliability=ReliabilityPolicy.BEST_EFFORT
            )
            self._debug_image_pub = self.create_publisher(
                Image, str(self.get_parameter("debug_image_topic").value), sensor_qos
            )
        self.get_logger().info(
            f"MobileSAM projector ready: device={self._sam_device}, "
            f"mask_erode_px={self._mask_erode_px}, "
            f"debug_image={self._publish_debug_image}"
        )

    @property
    def projection_mode(self) -> str:
        return "mobile_sam"

    @property
    def requires_rgb(self) -> bool:
        return True

    def _prepare_projection_frame(self, msg, payload, depth_m, source_size):
        context = super()._prepare_projection_frame(
            msg, payload, depth_m, source_size
        )
        rgb = context["rgb_image"]
        if rgb is None:
            raise RuntimeError("No RGB frame available for MobileSAM")
        detections = payload.get("detections", [])
        if not detections:
            return {**context, "masks": [], "scores": []}

        import torch

        source_width, source_height = source_size
        scale_x = rgb.shape[1] / source_width
        scale_y = rgb.shape[0] / source_height
        boxes = np.asarray([item.get("xyxy", []) for item in detections], dtype=np.float32)
        if boxes.ndim != 2 or boxes.shape[1] != 4:
            raise RuntimeError("MobileSAM requires xyxy boxes for every detection")
        boxes[:, (0, 2)] *= scale_x
        boxes[:, (1, 3)] *= scale_y

        self._sam_predictor.set_image(rgb)
        box_tensor = torch.as_tensor(boxes, device=self._sam_device)
        transformed = self._sam_predictor.transform.apply_boxes_torch(
            box_tensor, rgb.shape[:2]
        )
        with torch.inference_mode():
            masks, scores, _ = self._sam_predictor.predict_torch(
                point_coords=None,
                point_labels=None,
                boxes=transformed,
                multimask_output=False,
            )
        masks = masks[:, 0].detach().cpu().numpy().astype(bool)
        scores = scores[:, 0].detach().cpu().numpy()
        if self._mask_erode_px > 0:
            size = self._mask_erode_px * 2 + 1
            kernel = np.ones((size, size), dtype=np.uint8)
            masks = np.stack([
                cv2.erode(mask.astype(np.uint8), kernel, iterations=1).astype(bool)
                for mask in masks
            ])
        # 保留 RGB 分辨率下的 mask，用于直观看出分割是否包含了背景。
        display_masks = masks.copy()
        if masks.shape[1:] != depth_m.shape:
            masks = np.stack([
                cv2.resize(
                    mask.astype(np.uint8),
                    (depth_m.shape[1], depth_m.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                ).astype(bool)
                for mask in masks
            ])
        self._publish_sam_debug_image(msg, rgb, payload, display_masks, scores)
        return {**context, "masks": masks, "scores": scores}

    def _publish_sam_debug_image(
        self, msg, rgb, payload, masks, scores
    ) -> None:
        """发布 mask、检测框和标签叠加图，不参与后续几何计算。"""
        if self._debug_image_pub is None:
            return
        blended = rgb.astype(np.float32)
        detections = payload.get("detections", [])
        for detection, mask in zip(detections, masks):
            confidence = float(detection.get("confidence", 0.0))
            if confidence < self._min_confidence or self._is_excluded_detection(detection):
                continue
            packed = self._class_color(int(detection.get("class_id", -1)))
            color = np.asarray(
                [(packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF],
                dtype=np.float32,
            )
            blended[mask] = (
                (1.0 - self._debug_mask_alpha) * blended[mask]
                + self._debug_mask_alpha * color
            )
        overlay = np.clip(blended, 0, 255).astype(np.uint8)
        for detection_id, detection in enumerate(detections):
            confidence = float(detection.get("confidence", 0.0))
            if confidence < self._min_confidence or self._is_excluded_detection(detection):
                continue
            xyxy = detection.get("xyxy", [])
            if len(xyxy) != 4:
                continue
            packed = self._class_color(int(detection.get("class_id", -1)))
            color = (
                int((packed >> 16) & 0xFF),
                int((packed >> 8) & 0xFF),
                int(packed & 0xFF),
            )
            x1, y1, x2, y2 = (int(round(value)) for value in xyxy)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
            score = float(scores[detection_id]) if detection_id < len(scores) else 0.0
            label = (
                f"{detection.get('class_name', 'unknown')} "
                f"det={confidence:.2f} sam={score:.2f}"
            )
            cv2.putText(
                overlay, label, (max(0, x1), max(16, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA,
            )
        debug_msg = self._bridge.cv2_to_imgmsg(overlay, encoding="rgb8")
        debug_msg.header.stamp = msg.header.stamp
        debug_msg.header.frame_id = msg.header.frame_id
        self._debug_image_pub.publish(debug_msg)

    def _project_detection(self, depth_m, detection, detection_id, source_size, context):
        if detection_id >= len(context["masks"]):
            return None
        mask = context["masks"][detection_id]
        projected = project_mask_depth(
            depth_m,
            mask,
            self._intrinsics,
            pixel_stride=self._pixel_stride,
            min_depth_m=self._min_depth_m,
            max_depth_m=self._max_depth_m,
        )
        if projected is None:
            return None
        source_width, source_height = source_size
        image_uv = projected.image_uv.copy()
        image_uv[:, 0] = (image_uv[:, 0] + 0.5) * source_width / depth_m.shape[1] - 0.5
        image_uv[:, 1] = (image_uv[:, 1] + 0.5) * source_height / depth_m.shape[0] - 0.5
        projected = ProjectedMask(projected.points_camera, image_uv)
        return projected, {
            "sam_score": float(context["scores"][detection_id]),
            "mask_area_pixels": int(np.count_nonzero(mask)),
        }


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SamOfflineProjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
