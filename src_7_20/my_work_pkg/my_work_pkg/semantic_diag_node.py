#!/usr/bin/env python3
"""ROS 2 semantic pipeline diagnostics.

This node watches the main semantic-mapping topics and periodically publishes a
small JSON status report.  It is intentionally lightweight so it can run during
field tests without changing the mapping pipeline itself.
"""

from __future__ import annotations

import json
from collections import deque

import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class _TopicRate:
    """Small sliding-window Hz estimator."""

    def __init__(self, window_s: float = 5.0) -> None:
        self.window_s = window_s
        self.stamps: deque[float] = deque()
        self.total = 0

    def tick(self, now_s: float) -> None:
        self.total += 1
        self.stamps.append(now_s)
        self.prune(now_s)

    def prune(self, now_s: float) -> None:
        while self.stamps and now_s - self.stamps[0] > self.window_s:
            self.stamps.popleft()

    def hz(self, now_s: float) -> float:
        self.prune(now_s)
        if len(self.stamps) < 2:
            return 0.0
        dt = self.stamps[-1] - self.stamps[0]
        if dt <= 0.0:
            return 0.0
        return (len(self.stamps) - 1) / dt

    def age(self, now_s: float) -> float | None:
        if not self.stamps:
            return None
        return now_s - self.stamps[-1]


class SemanticDiagNode(Node):
    """Watch semantic pipeline topic rates and TF availability."""

    def __init__(self) -> None:
        super().__init__("semantic_diag_node")

        self.declare_parameter("json_topic", "/yolo/results_json")
        self.declare_parameter("depth_topic", "/camera/depth_image")
        self.declare_parameter("seed_topic", "/semantic_seed")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("camera_frame", "camera_optical_link")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("publish_period_s", 1.0)
        self.declare_parameter("stale_after_s", 2.0)

        self._rates = {
            "json": _TopicRate(),
            "depth": _TopicRate(),
            "seed": _TopicRate(),
            "map": _TopicRate(),
        }
        self._map_received = False

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(
            String,
            self.get_parameter("json_topic").value,
            lambda _msg: self._tick("json"),
            reliable_qos,
        )
        self.create_subscription(
            Image,
            self.get_parameter("depth_topic").value,
            lambda _msg: self._tick("depth"),
            sensor_qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("seed_topic").value,
            lambda _msg: self._tick("seed"),
            reliable_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self._map_cb,
            map_qos,
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(String, "/semantic_diag", 10)
        self.create_timer(
            float(self.get_parameter("publish_period_s").value),
            self._publish_diag,
        )

        self.get_logger().info("SemanticDiagNode Ready")

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self, key: str) -> None:
        self._rates[key].tick(self._now_s())

    def _map_cb(self, _msg: OccupancyGrid) -> None:
        self._map_received = True
        self._tick("map")

    def _publish_diag(self) -> None:
        now = self._now_s()
        stale_after = float(self.get_parameter("stale_after_s").value)
        camera_frame = str(self.get_parameter("camera_frame").value or "camera_optical_link")
        target_frame = str(self.get_parameter("target_frame").value)

        tf_ok = self._tf_buffer.can_transform(
            target_frame,
            camera_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.02),
        )

        topics = {}
        for key, rate in self._rates.items():
            age = rate.age(now)
            topics[key] = {
                "hz": round(rate.hz(now), 2),
                "total": rate.total,
                "age_s": None if age is None else round(age, 2),
                "stale": age is None or age > stale_after,
            }

        report = {
            "ok": bool(self._map_received and tf_ok and not topics["seed"]["stale"]),
            "map_received": self._map_received,
            "tf": {
                "source": camera_frame,
                "target": target_frame,
                "ok": bool(tf_ok),
            },
            "topics": topics,
        }

        text = json.dumps(report, ensure_ascii=False)
        self._pub.publish(String(data=text))
        self.get_logger().info(text)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SemanticDiagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
