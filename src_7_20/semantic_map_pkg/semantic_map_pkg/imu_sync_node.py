#!/usr/bin/env python3
"""Merge split Orbbec accel/gyro streams and align their device clock to ROS time."""

from __future__ import annotations

from collections import deque
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def stamp_nanoseconds(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def set_stamp_from_nanoseconds(stamp, value: int) -> None:
    stamp.sec = int(value // 1_000_000_000)
    stamp.nanosec = int(value % 1_000_000_000)


class ImuSyncNode(Node):
    """Publish one complete, monotonic IMU sample for each gyro message."""

    def __init__(self) -> None:
        super().__init__("semantic_bag_imu_sync")
        defaults = {
            "accel_topic": "/camera/accel/sample",
            "gyro_topic": "/camera/gyro/sample",
            "output_topic": "/imu",
            "output_frame": "camera_gyro_optical_frame",
            "max_pair_dt_s": 0.02,
            "accel_buffer_size": 100,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._output_frame = str(self.get_parameter("output_frame").value)
        self._max_pair_dt_ns = int(
            float(self.get_parameter("max_pair_dt_s").value) * 1_000_000_000
        )
        buffer_size = max(2, int(self.get_parameter("accel_buffer_size").value))
        self._accel: deque[tuple[int, Imu]] = deque(maxlen=buffer_size)
        self._clock_offset_ns: int | None = None
        self._last_output_ns = -1
        self._published = 0
        self._dropped = 0

        self._publisher = self.create_publisher(
            Imu, str(self.get_parameter("output_topic").value), qos_profile_sensor_data
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("accel_topic").value),
            self._accel_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("gyro_topic").value),
            self._gyro_cb,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "IMU sync ready: split accel/gyro -> /imu with ROS-time alignment"
        )

    def _accel_cb(self, msg: Imu) -> None:
        self._accel.append((stamp_nanoseconds(msg.header.stamp), msg))

    def _gyro_cb(self, gyro: Imu) -> None:
        if not self._accel:
            self._dropped += 1
            return
        gyro_device_ns = stamp_nanoseconds(gyro.header.stamp)
        accel_device_ns, accel = min(
            self._accel, key=lambda item: abs(item[0] - gyro_device_ns)
        )
        pair_dt_ns = abs(accel_device_ns - gyro_device_ns)
        if pair_dt_ns > self._max_pair_dt_ns:
            self._dropped += 1
            return

        ros_now_ns = self.get_clock().now().nanoseconds
        if ros_now_ns <= 0:
            self._dropped += 1
            return
        if self._clock_offset_ns is None:
            self._clock_offset_ns = ros_now_ns - gyro_device_ns
            self.get_logger().info(
                f"IMU clock aligned: offset={self._clock_offset_ns / 1e9:.6f}s"
            )

        output_ns = gyro_device_ns + self._clock_offset_ns
        if output_ns <= self._last_output_ns:
            output_ns = self._last_output_ns + 1

        merged = Imu()
        merged.header = deepcopy(gyro.header)
        merged.header.frame_id = self._output_frame
        set_stamp_from_nanoseconds(merged.header.stamp, output_ns)
        merged.orientation = deepcopy(gyro.orientation)
        merged.orientation_covariance = list(gyro.orientation_covariance)
        merged.angular_velocity = deepcopy(gyro.angular_velocity)
        merged.angular_velocity_covariance = list(gyro.angular_velocity_covariance)
        merged.linear_acceleration = deepcopy(accel.linear_acceleration)
        merged.linear_acceleration_covariance = list(accel.linear_acceleration_covariance)
        self._publisher.publish(merged)
        self._last_output_ns = output_ns
        self._published += 1
        self.get_logger().info(
            f"IMU output active: published={self._published}, dropped={self._dropped}, "
            f"pair_dt={pair_dt_ns / 1e6:.2f}ms",
            throttle_duration_sec=10.0,
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ImuSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
