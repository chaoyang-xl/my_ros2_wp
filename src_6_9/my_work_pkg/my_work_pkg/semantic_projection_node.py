#!/usr/bin/env python3
#ros2 topic pub /semantic_seed std_msgs/msg/String "data: '{\"label\": \"bin\", \"confidence\": 0.9, \"gx\": -2.6, \"gy\": 3.8}'" -1
from __future__ import annotations

import json
from dataclasses import asdict

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
from my_work_pkg.semantic_projection import (
    OccupancyGridMap,
    SemanticSeed,
    SinglePointSemanticProjector,
    SemanticMemory,     # <--- 导入新写的记忆类
    TrackedObject,       # <--- 导入数据类
    SnappedObject,
)

class SemanticProjectionNode(Node):
    def __init__(self) -> None:
        super().__init__("semantic_projection_node")

        # 原有参数
        self.declare_parameter("search_radius_m", 0.5)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("min_island_pixels", 2)
        
        # 新增：记忆池与追踪参数
        self.declare_parameter("match_distance", 1.0)
        self.declare_parameter("smoothing_alpha", 0.3)
        self.declare_parameter("memory_timeout", 5.0)  # 单位秒，inf表示永不过期

        self._projector = SinglePointSemanticProjector(
            search_radius_m=self.get_parameter("search_radius_m").value,
            min_total_pixels=self.get_parameter("min_island_pixels").value,
        )

        # 实例化记忆池
        self.memory = SemanticMemory(
            match_distance=self.get_parameter("match_distance").value,
            alpha=self.get_parameter("smoothing_alpha").value,
            timeout=self.get_parameter("memory_timeout").value,
        )

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._sub_map = self.create_subscription(OccupancyGrid, "/map", self._map_cb, map_qos)
        self._sub_seed = self.create_subscription(String, "/semantic_seed", self._seed_cb, 10)
        self._pub_marker = self.create_publisher(MarkerArray, "~/markers", 10)

        self._map: OccupancyGridMap | None = None
        
        # 新增：定时器，以 2Hz 的频率老化对象并发布 Marker
        self.create_timer(0.5, self._publish_memory_cb)

        self.get_logger().info("Semantic Tracking Node Ready: snap + match + EMA smooth")
        
    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = OccupancyGridMap(
            data=msg.data,
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            occupied_threshold=int(self.get_parameter("occupied_threshold").value)
        )

    def _seed_cb(self, msg: String) -> None:

        self.get_logger().info(f"Received seed JSON: {msg.data[:100]}...")  # 打印前100字符预览
        if self._map is None:
            return

        try:
            payload = json.loads(msg.data)
            seed = SemanticSeed(
                label=payload["label"],
                confidence=payload["confidence"],
                gx=payload["gx"],
                gy=payload["gy"],
                track_id=payload.get("track_id"),
            )
        except Exception as exc:
            self.get_logger().warn(f"Invalid seed: {exc}")
            return

        snapped = self._projector.project(seed , self._map)
        if snapped is None:
            self.get_logger().debug(f"Rejected seed '{seed.label}' at ({seed.gx:.2f}, {seed.gy:.2f})")
            return

        # 核心改动：不再直接发布，而是送入记忆池更新
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        tracked = self.memory.update(snapped, now_sec)

        self.get_logger().info(
            f"[Memory] {tracked.object_id} updated: pos=({tracked.x:.2f}, {tracked.y:.2f}), seen={tracked.times_seen}"
        )

    def _publish_memory_cb(self) -> None:
        """定时老化并发布所有存活的对象"""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.memory.age(now_sec) #清除过期对象
        
        active_objects = self.memory.get_active_objects()
        #self.get_logger().info(f"Publishing {len(active_objects)} active objects from memory.")
        if not active_objects:

            return

        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, obj in enumerate(active_objects):
            # 1. 绘制空心矩形框
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "tracked_objects_box"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            # 让 Marker 在1秒后自动消失。如果定时器0.5秒后还在发，它就会一直亮着。如果被 age() 清理了，RViz也会自动清除它。
            marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg() 
            
            marker.pose.position.x = obj.x
            marker.pose.position.y = obj.y
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.05  # 线宽
            
            sx = max(0.1, min(obj.size_x, 1.0))
            sy = max(0.1, min(obj.size_y, 1.0))
            hx, hy = sx / 2.0, sy / 2.0
            
            marker.points = [
                Point(x=hx, y=hy, z=0.0),
                Point(x=-hx, y=hy, z=0.0),
                Point(x=-hx, y=-hy, z=0.0),
                Point(x=hx, y=-hy, z=0.0),
                Point(x=hx, y=hy, z=0.0),
            ]

            marker.color.r = 0.2
            marker.color.g = 0.8
            marker.color.b = 0.3
            marker.color.a = 0.9

            # 2. 绘制文本标签
            text = Marker()
            text.header = marker.header
            text.ns = "tracked_objects_text"
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.lifetime = marker.lifetime
            
            text.pose.position.x = obj.x
            text.pose.position.y = obj.y
            text.pose.position.z = 0.55
            text.pose.orientation.w = 1.0
            
            text.scale.z = 0.12
            text.color.r = 1.0
            text.color.g = 0.0
            text.color.b = 0.0
            text.color.a = 0.95
            
            # 显示 ID、置信度和累积观测次数
            text.text = f"{obj.object_id}({obj.confidence*100:.0f}%)n={obj.times_seen}"

            ma.markers.extend([marker, text])

        self._pub_marker.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticProjectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()