#!/usr/bin/env python3
"""ROS 2 节点：语义种子追踪 + EMA 平滑 + 置信度累积 + 点云可视化。

不依赖 /map，直接订阅 /semantic_seed，用 SeedTracker 做轻量级追踪，
发布彩色球 + 文字标签到 RViz。

用法::
    ros2 run my_work_pkg seed_tracker_node
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from my_work_pkg.seed_tracker import SeedTracker, TrackedSeed

# ── 24 色调色板 ──────────────────────────────────────────────────────────────

_PALETTE: list[tuple[float, float, float]] = [
    (1.0, 0.0, 0.0), (0.0, 0.6, 1.0), (0.0, 0.8, 0.2), (1.0, 0.7, 0.0),
    (0.7, 0.0, 1.0), (0.0, 1.0, 0.8), (1.0, 0.3, 0.6), (0.5, 1.0, 0.0),
    (0.0, 0.4, 1.0), (1.0, 0.5, 0.0), (0.4, 0.0, 0.8), (0.0, 0.8, 0.6),
    (1.0, 0.0, 0.5), (0.6, 0.8, 0.0), (0.0, 0.2, 0.8), (0.8, 0.3, 0.0),
    (0.8, 0.0, 0.8), (0.0, 0.7, 0.3), (0.9, 0.9, 0.0), (0.6, 0.0, 0.3),
    (0.0, 0.5, 0.5), (0.8, 0.6, 0.0), (0.3, 0.3, 1.0), (0.9, 0.4, 0.6),
]


def _color_for_label(label: str) -> tuple[float, float, float]:
    return _PALETTE[hash(label) % len(_PALETTE)]


# ── Node ─────────────────────────────────────────────────────────────────────

class SeedTrackerNode(Node):
    """EMA 平滑 + 置信度累积 + 生命周期管理 + 彩色点可视化。"""

    def __init__(self) -> None:
        super().__init__("seed_tracker_node")

        # ---- tracker 参数 ----
        self.declare_parameter("match_distance", 1.0)
        self.declare_parameter("smoothing_alpha", 0.3)
        self.declare_parameter("memory_timeout", 5.0)

        # ---- 可视化参数 ----
        self.declare_parameter("input_topic", "/semantic_seed")
        self.declare_parameter("point_scale", 0.12)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("min_times_seen", 1)  # 最少观测几次才显示

        # ---- 引擎 ----
        self._tracker = SeedTracker(
            match_distance=self.get_parameter("match_distance").value,
            alpha=self.get_parameter("smoothing_alpha").value,
            timeout=self.get_parameter("memory_timeout").value,
        )
        self._min_times_seen = self.get_parameter("min_times_seen").value
        self._point_scale = self.get_parameter("point_scale").value

        # ---- 订阅 & 发布 ----
        self._sub = self.create_subscription(
            String,
            self.get_parameter("input_topic").value,
            self._seed_cb,
            10,
        )
        self._pub = self.create_publisher(MarkerArray, "~/markers", 10)
        self.create_timer(1.0 / self.get_parameter("publish_hz").value, self._tick)

        self._label_colors: dict[str, tuple[float, float, float]] = {}

        self.get_logger().info("SeedTrackerNode Ready (EMA + lifetime + points)")

    # ------------------------------------------------------------------
    def _seed_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        label = payload.get("label", "unknown")
        confidence = payload.get("confidence", 0.0)
        gx = payload.get("gx")
        gy = payload.get("gy")
        if gx is None or gy is None:
            return

        obj = self._tracker.update(label, confidence, gx, gy)
        self.get_logger().debug(
            f"[Tracker] {obj.object_id} pos=({obj.x:.2f},{obj.y:.2f}) "
            f"conf={obj.confidence:.2f} seen={obj.times_seen}"
        )

    def _tick(self) -> None:
        # 老化
        stale = self._tracker.age()
        for s in stale:
            self.get_logger().debug(f"[Tracker] expired: {s.object_id}")

        active = self._tracker.get_active()
        if not active:
            return

        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        lifetime = rclpy.duration.Duration(
            seconds=self.get_parameter("memory_timeout").value
        ).to_msg()

        for obj in active:
            if obj.times_seen < self._min_times_seen:
                continue

            r, g, b = self._get_color(obj.label)

            # ---- sphere ----
            sphere = Marker()
            sphere.header.frame_id = "map"
            sphere.header.stamp = stamp
            sphere.ns = f"tracked_seed_sphere/{obj.label}"
            sphere.id = hash(obj.object_id) % 100000
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.lifetime = lifetime

            sphere.pose.position.x = obj.x
            sphere.pose.position.y = obj.y
            sphere.pose.position.z = 0.05
            sphere.pose.orientation.w = 1.0

            s = self._point_scale
            sphere.scale.x = s
            sphere.scale.y = s
            sphere.scale.z = s

            sphere.color.r = r
            sphere.color.g = g
            sphere.color.b = b
            sphere.color.a = min(1.0, 0.4 + 0.6 * obj.confidence)

            # ---- text label ----
            text = Marker()
            text.header = sphere.header
            text.ns = f"tracked_seed_text/{obj.label}"
            text.id = sphere.id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.lifetime = lifetime

            text.pose.position.x = obj.x
            text.pose.position.y = obj.y
            text.pose.position.z = 0.25
            text.pose.orientation.w = 1.0

            text.scale.z = 0.10
            text.color.r = r
            text.color.g = g
            text.color.b = b
            text.color.a = 0.95
            text.text = (
                f"{obj.label} {obj.confidence * 100:.0f}% "
                f"n={obj.times_seen}"
            )

            ma.markers.extend([sphere, text])

        self._pub.publish(ma)

    def _get_color(self, label: str) -> tuple[float, float, float]:
        if label not in self._label_colors:
            self._label_colors[label] = _color_for_label(label)
        return self._label_colors[label]


# ── Entry point ──────────────────────────────────────────────────────────────

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SeedTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
