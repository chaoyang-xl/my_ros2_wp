#!/usr/bin/env python3
"""Decode the Meeting_Room ROS1 compressed RGB-D topics for ROS 2 nodes."""

from __future__ import annotations

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from semantic_map_offline.compressed_rgbd import decode_16uc1_depth, decode_color


class CompressedRgbdDecoderNode(Node):
    """Republish compressed color/depth messages as raw Image messages."""

    def __init__(self) -> None:
        super().__init__("meeting_room_rgbd_decoder")
        defaults = {
            "color_compressed_topic": "/orbbec_camera/color/image_raw/compressed",
            "depth_compressed_topic": (
                "/orbbec_camera/depth/image_raw/compressedDepth"
            ),
            "color_raw_topic": "/camera/color/image_raw",
            "depth_raw_topic": "/camera/depth/image_raw",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        input_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        output_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self._bridge = CvBridge()
        self._color_pub = self.create_publisher(
            Image, str(self.get_parameter("color_raw_topic").value), output_qos
        )
        self._depth_pub = self.create_publisher(
            Image, str(self.get_parameter("depth_raw_topic").value), output_qos
        )
        self.create_subscription(
            CompressedImage,
            str(self.get_parameter("color_compressed_topic").value),
            self._color_cb,
            input_qos,
        )
        self.create_subscription(
            CompressedImage,
            str(self.get_parameter("depth_compressed_topic").value),
            self._depth_cb,
            input_qos,
        )
        self.get_logger().info("Meeting_Room compressed RGB-D decoder ready")

    def _color_cb(self, msg: CompressedImage) -> None:
        try:
            image = decode_color(msg.data)
            output = self._bridge.cv2_to_imgmsg(image, encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(
                f"Color decode failed: {exc}", throttle_duration_sec=5.0
            )
            return
        output.header = msg.header
        self._color_pub.publish(output)
        self.get_logger().info(
            f"Decoded RGB active: {output.width}x{output.height}", once=True
        )

    def _depth_cb(self, msg: CompressedImage) -> None:
        try:
            depth = decode_16uc1_depth(msg.data, msg.format)
            output = self._bridge.cv2_to_imgmsg(depth, encoding="16UC1")
        except Exception as exc:
            self.get_logger().warn(
                f"Depth decode failed: {exc}", throttle_duration_sec=5.0
            )
            return
        output.header = msg.header
        self._depth_pub.publish(output)
        self.get_logger().info(
            f"Decoded depth active: {output.width}x{output.height}", once=True
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CompressedRgbdDecoderNode()
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
