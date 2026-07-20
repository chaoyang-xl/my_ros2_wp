#!/usr/bin/env python3
"""Load a saved semantic JSON snapshot and publish its static objects."""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from semantic_navigation_pkg.semantic_map import (
    SemanticMap,
    load_semantic_map,
    semantic_map_snapshot,
)


class SemanticMapLoaderNode(Node):
    """Publish a durable, static-only view of an existing semantic JSON file."""

    def __init__(self) -> None:
        super().__init__("semantic_map_loader_node")
        self.declare_parameter("semantic_map_path", "/tmp/semantic_objects.json")
        self.declare_parameter("output_topic", "/semantic_map/objects")
        self.declare_parameter("excluded_labels", ["person"])
        self.declare_parameter("reload_period_s", 1.0)
        self.declare_parameter("publish_markers", True)

        durable_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._objects_pub = self.create_publisher(
            String,
            str(self.get_parameter("output_topic").value),
            durable_qos,
        )
        self._marker_pub = self.create_publisher(MarkerArray, "~/markers", durable_qos)

        self._semantic_map: SemanticMap | None = None
        self._last_mtime_ns: int | None = None
        self._last_error: str | None = None
        period = max(0.2, float(self.get_parameter("reload_period_s").value))
        self.create_timer(period, self._reload_cb)
        self._reload(force=True)

    def _reload_cb(self) -> None:
        self._reload(force=False)

    def _reload(self, force: bool) -> None:
        path = Path(str(self.get_parameter("semantic_map_path").value)).expanduser()
        try:
            mtime_ns = path.stat().st_mtime_ns
            if not force and mtime_ns == self._last_mtime_ns:
                return
            excluded = [str(value) for value in self.get_parameter("excluded_labels").value]
            semantic_map = load_semantic_map(path, excluded_labels=excluded)
        except Exception as exc:
            message = f"Failed to load semantic map '{path}': {exc}"
            if message != self._last_error:
                self.get_logger().error(message)
                self._last_error = message
            return

        self._semantic_map = semantic_map
        self._last_mtime_ns = mtime_ns
        self._last_error = None
        self._publish()
        self.get_logger().info(
            f"Loaded {len(semantic_map.objects)} static objects from {path}"
        )

    def _publish(self) -> None:
        if self._semantic_map is None:
            return
        snapshot = semantic_map_snapshot(self._semantic_map)
        self._objects_pub.publish(
            String(data=json.dumps(snapshot, ensure_ascii=False, separators=(",", ":")))
        )
        if bool(self.get_parameter("publish_markers").value):
            self._marker_pub.publish(self._build_markers(self._semantic_map))

    def _build_markers(self, semantic_map: SemanticMap) -> MarkerArray:
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = semantic_map.frame_id
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for index, obj in enumerate(semantic_map.objects):
            box = Marker()
            box.header = clear.header
            box.ns = "static_semantic_objects"
            box.id = index
            box.type = Marker.LINE_STRIP
            box.action = Marker.ADD
            box.pose.position.x = obj.x
            box.pose.position.y = obj.y
            box.pose.position.z = 0.1
            box.pose.orientation.w = 1.0
            box.scale.x = 0.04
            box.color.r = 0.1
            box.color.g = 0.75
            box.color.b = 0.85
            box.color.a = 0.95
            hx = max(0.1, obj.size_x) / 2.0
            hy = max(0.1, obj.size_y) / 2.0
            box.points = [
                Point(x=hx, y=hy),
                Point(x=-hx, y=hy),
                Point(x=-hx, y=-hy),
                Point(x=hx, y=-hy),
                Point(x=hx, y=hy),
            ]

            text = Marker()
            text.header = clear.header
            text.ns = "static_semantic_labels"
            text.id = index
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = obj.x
            text.pose.position.y = obj.y
            text.pose.position.z = 0.45
            text.pose.orientation.w = 1.0
            text.scale.z = 0.16
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f"{obj.object_id} ({obj.confidence * 100:.0f}%)"
            markers.markers.extend([box, text])
        return markers


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SemanticMapLoaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

