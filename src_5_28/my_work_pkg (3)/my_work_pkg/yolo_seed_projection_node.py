#!/usr/bin/env python3
"""ROS 2 node: YOLO detections + Depth Image -> map-frame semantic seed JSON."""

from __future__ import annotations

import json

import rclpy
import tf2_geometry_msgs  # noqa: F401 Needed so Buffer can transform PointStamped.
import tf2_ros
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
import message_filters
from cv_bridge import CvBridge

from my_work_pkg.yolo_seed_projection import (
    YoloDetection2D,
    YoloDepthSeedProjector,
)


class YoloSeedProjectionNode(Node):
    """Turn YOLO 2D detections and Depth images into rough `map` seeds."""

    def __init__(self) -> None:
        super().__init__("yolo_seed_projection_node")

        # 声明相机内参 (请根据你的真实相机或仿真相机修改默认值)
        self.declare_parameter("camera_fx", 380.0)
        self.declare_parameter("camera_fy", 380.0)
        self.declare_parameter("camera_cx", 320.0)
        self.declare_parameter("camera_cy", 240.0)
        # 深度图尺度: 16UC1(毫米)通常需要0.001转为米; 32FC1(米)则为1.0
        self.declare_parameter("depth_scale", 0.001) 
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("tf_timeout_s", 0.1)

        self._target_frame = self.get_parameter("target_frame").value
        self._tf_timeout_s = self.get_parameter("tf_timeout_s").value
        self.depth_scale = self.get_parameter("depth_scale").value

        self.cv_bridge = CvBridge()
        self.projector = YoloDepthSeedProjector(
            fx=self.get_parameter("camera_fx").value,
            fy=self.get_parameter("camera_fy").value,
            cx=self.get_parameter("camera_cx").value,
            cy=self.get_parameter("camera_cy").value,
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # 图像往往很大，QoS需要适配
        image_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        det_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # 使用 message_filters 同步 YOLO 检测框和深度图
        self._yolo_sub = message_filters.Subscriber(self, Detection2DArray, "/yolo/detections", qos_profile=det_qos)
        self._depth_sub = message_filters.Subscriber(self, Image, "/camera/depth/image_rect_raw", qos_profile=image_qos)

        # slop 允许一定的时间误差 (0.1秒)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._yolo_sub, self._depth_sub],
            queue_size=10,
            slop=0.1,
        )
        self._sync.registerCallback(self._synced_cb)

        self._pub = self.create_publisher(String, "/semantic_seed", 10)
        
        self.get_logger().info("YOLO Depth Seed Projector Node Ready.")

    def _synced_cb(self, yolo_msg: Detection2DArray, depth_msg: Image) -> None:
        """Callback for synchronized YOLO bounding boxes and Depth images."""
        
        try:
            # 将 ROS Image 转为 OpenCV numpy 数组 (保留原始编码)
            cv_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        # 统一转为单位 [米] 的 numpy 数组
        depth_image_meters = cv_depth * self.depth_scale
        camera_frame = depth_msg.header.frame_id 

        for det in yolo_msg.detections:
            if not det.results:
                continue

            det2d = YoloDetection2D(
                label=det.results[0].hypothesis.class_id,
                confidence=det.results[0].hypothesis.score,
                center_u=det.bbox.center.position.x,
                center_v=det.bbox.center.position.y,
                image_width=depth_msg.width,
                image_height=depth_msg.height,
                track_id=det.id if det.id else None,
            )

            # 获取局部相机坐标系下的 3D 点
            seed_camera = self.projector.project(det2d, depth_image_meters)
            if not seed_camera:
                continue

            # 通过 TF 变换到 map 坐标系
            map_point = self._transform_to_map(
                seed_camera.x, 
                seed_camera.y, 
                seed_camera.z, 
                camera_frame
            )
            
            if map_point is None:
                continue

            gx, gy = map_point

            # 构建 JSON 发送给后端的 Occupancy Snap 节点
            payload = {
                "label": seed_camera.label,
                "confidence": seed_camera.confidence,
                "gx": gx,
                "gy": gy,
            }
            if seed_camera.track_id:
                payload["track_id"] = seed_camera.track_id

            self._pub.publish(String(data=json.dumps(payload)))
    #camera optical frame到map frame的变换，输入相机坐标系下的点，输出地图坐标系下的点
    def _transform_to_map(
        self, x: float, y: float, z: float, source_frame: str
    ) -> tuple[float, float] | None:
        """Transform a point from the camera optical frame into the map frame."""
        point = PointStamped()
        point.header.stamp = rclpy.time.Time(seconds=0).to_msg()
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
            self.get_logger().debug("TF %s -> %s failed: %s" % (
                source_frame, self._target_frame, exc
            ))
            return None

        return float(transformed.point.x), float(transformed.point.y)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = YoloSeedProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()