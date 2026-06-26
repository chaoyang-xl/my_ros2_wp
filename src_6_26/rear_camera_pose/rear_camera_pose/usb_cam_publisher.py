#!/usr/bin/env python3
"""
Minimal USB camera publisher — reads V4L2 device, publishes sensor_msgs/Image.

No inference, no processing, no recording.  Just capture + publish.
Designed to feed rear_pose_node (or any other image subscriber).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class UsbCamPublisher(Node):
    def __init__(self):
        super().__init__("usb_cam_publisher")

        self.declare_parameter("camera_device", "/dev/video2")
        self.declare_parameter("image_topic", "/rear_camera/image_raw")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15)

        device = self.get_parameter("camera_device").value
        topic = self.get_parameter("image_topic").value
        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        fps = self.get_parameter("fps").value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, topic, 10)

        self.get_logger().info(f"Opening {device} ({width}x{height} @ {fps} fps)")
        cap = cv2.VideoCapture(device)
        if not cap.isOpened():
            self.get_logger().error(f"Cannot open {device}")
            raise RuntimeError(f"Cannot open V4L2 device: {device}")

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f"Actual: {actual_w}x{actual_h} @ {actual_fps:.1f} fps → publishing on {topic}"
        )

        self._cap = cap
        period = 1.0 / max(1, fps)
        self._timer = self.create_timer(period, self._publish_frame)
        self.get_logger().info(f"Publishing at {fps} Hz (period={period:.3f}s)")

    def _publish_frame(self):
        ret, frame = self._cap.read()
        if not ret or frame is None:
            self.get_logger().warn("V4L2 read failed", throttle_duration_sec=5.0)
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "rear_camera"
        self.pub.publish(msg)

    def destroy_node(self):
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
