#!/usr/bin/env python3
"""极简可视化节点：直接把 /semantic_seed 的所有原始种子显示为彩色点。

绕过所有过滤（无占用栅格吸附、无几何验证、无记忆追踪），
纯粹展示"相机看到了什么"。用于调试和对比。

用法::
    ros2 run my_work_pkg raw_seed_visualizer_node
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


# 24 色调色板 — 同一个 label 始终对应同一种颜色
_PALETTE: list[tuple[float, float, float]] = [
    (1.0, 0.0, 0.0),   # red
    (0.0, 0.6, 1.0),   # sky blue
    (0.0, 0.8, 0.2),   # green
    (1.0, 0.7, 0.0),   # orange
    (0.7, 0.0, 1.0),   # purple
    (0.0, 1.0, 0.8),   # teal
    (1.0, 0.3, 0.6),   # pink
    (0.5, 1.0, 0.0),   # lime
    (0.0, 0.4, 1.0),   # blue
    (1.0, 0.5, 0.0),   # dark orange
    (0.4, 0.0, 0.8),   # indigo
    (0.0, 0.8, 0.6),   # aqua
    (1.0, 0.0, 0.5),   # hot pink
    (0.6, 0.8, 0.0),   # olive
    (0.0, 0.2, 0.8),   # dark blue
    (0.8, 0.3, 0.0),   # brown
    (0.8, 0.0, 0.8),   # magenta
    (0.0, 0.7, 0.3),   # emerald
    (0.9, 0.9, 0.0),   # yellow
    (0.6, 0.0, 0.3),   # maroon
    (0.0, 0.5, 0.5),   # dark teal
    (0.8, 0.6, 0.0),   # gold
    (0.3, 0.3, 1.0),   # periwinkle
    (0.9, 0.4, 0.6),   # rose
]


def _color_for_label(label: str) -> tuple[float, float, float]:
    """Stable colour assignment based on label hash."""
    idx = hash(label) % len(_PALETTE)
    return _PALETTE[idx]


class RawSeedVisualizerNode(Node):
    """Subscribe to /semantic_seed and display every seed as a coloured point."""

    def __init__(self) -> None:
        super().__init__("raw_seed_visualizer_node")

        self.declare_parameter("input_topic", "/semantic_seed")
        self.declare_parameter("marker_lifetime_s", 5.0)
        self.declare_parameter("point_scale", 0.10)
        self.declare_parameter("publish_hz", 10.0)

        self._lifetime_s = self.get_parameter("marker_lifetime_s").value
        self._point_scale = self.get_parameter("point_scale").value
        pub_hz = self.get_parameter("publish_hz").value

        # ----- subscriptions -----
        self._sub = self.create_subscription(
            String,
            self.get_parameter("input_topic").value,
            self._seed_cb,
            10,
        )

        # ----- publisher (batched via timer) -----
        self._pub = self.create_publisher(MarkerArray, "~/markers", 10)

        # ----- state -----
        self._pending: list[Marker] = []
        self._id_counter = 0
        self._label_colors: dict[str, tuple[float, float, float]] = {}

        self.create_timer(1.0 / pub_hz, self._publish_tick)

        self.get_logger().info(
            f"Raw Seed Visualizer Ready — showing ALL seeds as points "
            f"(lifetime={self._lifetime_s}s, scale={self._point_scale}m)"
        )

    # ------------------------------------------------------------------
    def _seed_cb(self, msg: String) -> None:
        """Queue a point marker for every incoming seed."""
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        label = payload.get("label", "unknown")
        gx = payload.get("gx")
        gy = payload.get("gy")
        if gx is None or gy is None:
            return

        confidence = payload.get("confidence", 0.0)
        r, g, b = self._get_color(label)

        # ---- sphere ----
        sphere = Marker()
        sphere.header.frame_id = "map"
        sphere.header.stamp = self.get_clock().now().to_msg()
        sphere.ns = f"raw_seed_sphere/{label}"
        sphere.id = self._id_counter
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.lifetime = rclpy.duration.Duration(seconds=self._lifetime_s).to_msg()

        sphere.pose.position.x = gx
        sphere.pose.position.y = gy
        sphere.pose.position.z = 0.05
        sphere.pose.orientation.w = 1.0

        s = self._point_scale
        sphere.scale.x = s
        sphere.scale.y = s
        sphere.scale.z = s

        sphere.color.r = r
        sphere.color.g = g
        sphere.color.b = b
        sphere.color.a = 0.85

        # ---- text label ----
        text = Marker()
        text.header.frame_id = "map"
        text.header.stamp = sphere.header.stamp
        text.ns = f"raw_seed_text/{label}"
        text.id = self._id_counter
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.lifetime = sphere.lifetime

        text.pose.position.x = gx
        text.pose.position.y = gy
        text.pose.position.z = 0.25
        text.pose.orientation.w = 1.0

        text.scale.z = 0.10
        text.color.r = r
        text.color.g = g
        text.color.b = b
        text.color.a = 0.95
        text.text = f"{label} {confidence*100:.0f}%"

        self._id_counter = (self._id_counter + 1) % 100000
        self._pending.extend([sphere, text])

    def _publish_tick(self) -> None:
        """Flush queued markers as a MarkerArray."""
        if not self._pending:
            return
        ma = MarkerArray(markers=self._pending)
        self._pub.publish(ma)
        self._pending.clear()

    def _get_color(self, label: str) -> tuple[float, float, float]:
        """Return cached or newly assigned colour for a label."""
        if label not in self._label_colors:
            self._label_colors[label] = _color_for_label(label)
        return self._label_colors[label]


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RawSeedVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
