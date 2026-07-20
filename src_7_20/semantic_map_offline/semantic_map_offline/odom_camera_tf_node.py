#!/usr/bin/env python3
"""Publish recorded FastLIO odometry and configurable camera extrinsics as TF."""

from __future__ import annotations

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from semantic_map_offline.compressed_rgbd import standard_optical_quaternion


class OdomCameraTfNode(Node):
    """Bridge odom->lidar TF and the missing lidar->camera static transform."""

    def __init__(self) -> None:
        super().__init__("meeting_room_odom_camera_tf")
        optical_q = standard_optical_quaternion()
        defaults = {
            "odom_topic": "/fastlio_odom",
            "odom_frame": "odom",
            "lidar_frame": "lidar_link",
            "camera_frame": "orbbec_camera_color_optical_frame",
            "camera_x": 0.0,
            "camera_y": 0.0,
            "camera_z": 0.0,
            "camera_qx": optical_q[0],
            "camera_qy": optical_q[1],
            "camera_qz": optical_q[2],
            "camera_qw": optical_q[3],
            "path_topic": "/meeting_room/trajectory",
            "path_max_poses": 2000,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._lidar_frame = str(self.get_parameter("lidar_frame").value)
        self._camera_frame = str(self.get_parameter("camera_frame").value)
        self._path_max_poses = max(
            1, int(self.get_parameter("path_max_poses").value)
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._path = Path()
        self._path.header.frame_id = self._odom_frame
        self._path_pub = self.create_publisher(
            Path, str(self.get_parameter("path_topic").value), 10
        )
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_cb,
            qos,
        )
        self._publish_camera_extrinsic()
        self.get_logger().warn(
            "Using configurable lidar->camera extrinsic; default is standard "
            "optical-axis rotation with zero translation"
        )

    def _publish_camera_extrinsic(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._lidar_frame
        transform.child_frame_id = self._camera_frame
        transform.transform.translation.x = float(
            self.get_parameter("camera_x").value
        )
        transform.transform.translation.y = float(
            self.get_parameter("camera_y").value
        )
        transform.transform.translation.z = float(
            self.get_parameter("camera_z").value
        )
        transform.transform.rotation.x = float(
            self.get_parameter("camera_qx").value
        )
        transform.transform.rotation.y = float(
            self.get_parameter("camera_qy").value
        )
        transform.transform.rotation.z = float(
            self.get_parameter("camera_qz").value
        )
        transform.transform.rotation.w = float(
            self.get_parameter("camera_qw").value
        )
        self._static_broadcaster.sendTransform(transform)

    def _odom_cb(self, msg: Odometry) -> None:
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._lidar_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

        pose = PoseStamped()
        pose.header = transform.header
        pose.pose = msg.pose.pose
        self._path.header.stamp = msg.header.stamp
        self._path.poses.append(pose)
        if len(self._path.poses) > self._path_max_poses:
            self._path.poses = self._path.poses[-self._path_max_poses:]
        self._path_pub.publish(self._path)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdomCameraTfNode()
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
