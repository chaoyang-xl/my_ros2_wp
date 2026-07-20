#!/usr/bin/env python3
#ros2 topic pub /semantic_seed std_msgs/msg/String "data: '{\"label\": \"bin\", \"confidence\": 0.9, \"gx\": -2.6, \"gy\": 3.8}'" -1
from __future__ import annotations

import json
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from my_work_pkg.semantic_projection import (
    OccupancyGridMap,
    SemanticSeed,
    SinglePointSemanticProjector,
    SemanticMemory,     # <--- 导入新写的记忆类
    TrackedObject,       # <--- 导入数据类
    SnappedObject,
    semantic_objects_snapshot,
)


DEFAULT_LABEL_POLICIES = {
    "default": {
        "snap_to_occupancy": True,#是否吸附
        "fallback_visual_on_reject": False,#是否在栅格投影失败时使用视觉信息
        "min_confirmed_seen": 50,#最小确认观测次数
        "visual_size_m": 0.25,#视觉后备尺寸
        "visual_clearance_m": 0.35,#视觉后备净空半径
    },
    "person": {
        "snap_to_occupancy": False,
        "fallback_visual_on_reject": True,
        "min_confirmed_seen": 30,
        "visual_size_m": 0.25,
        "visual_clearance_m": 0.35,
    },
}

class SemanticProjectionNode(Node):
    def __init__(self) -> None:
        super().__init__("semantic_projection_node")

        self.declare_parameter("search_radius_m", 0.2)
        self.declare_parameter("occupied_threshold", 80)
        self.declare_parameter("min_island_pixels", 2)
        self.declare_parameter("max_island_size_m", 2.0)
        self.declare_parameter("max_total_pixels", 0)

        # 新增：记忆池与追踪参数
        self.declare_parameter("match_distance", 1.0)
        self.declare_parameter("smoothing_alpha", 0.3)
        self.declare_parameter("memory_timeout", 5.0)  # 单位秒，inf表示永不过期
        self.declare_parameter("min_confirmed_seen", 50)
        self.declare_parameter("show_candidates", False)
        self.declare_parameter("label_policy_json", "")
        self.declare_parameter("semantic_objects_topic", "/semantic_objects")
        self.declare_parameter("semantic_objects_path", "/tmp/semantic_objects.json")

        self._label_policies = self._load_label_policies()

        self._projector = SinglePointSemanticProjector(
            search_radius_m=self.get_parameter("search_radius_m").value,
            min_total_pixels=self.get_parameter("min_island_pixels").value,
            max_island_size_m=self.get_parameter("max_island_size_m").value,
            max_total_pixels=self.get_parameter("max_total_pixels").value,
        )

        # 实例化记忆池
        self.memory = SemanticMemory(
            match_distance=self.get_parameter("match_distance").value,
            alpha=self.get_parameter("smoothing_alpha").value,
            timeout=self.get_parameter("memory_timeout").value,
            min_confirmed_seen=self.get_parameter("min_confirmed_seen").value,
            label_min_confirmed_seen={
                label: policy.get(
                    "min_confirmed_seen",
                    self.get_parameter("min_confirmed_seen").value,
                )
                for label, policy in self._label_policies.items()
                if label != "default"
            },
        )

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._sub_map = self.create_subscription(OccupancyGrid, "/map", self._map_cb, map_qos)
        self._sub_seed = self.create_subscription(String, "/semantic_seed", self._seed_cb, 10)
        self._pub_marker = self.create_publisher(MarkerArray, "~/markers", 10)
        self._pub_objects = self.create_publisher(
            String,
            self.get_parameter("semantic_objects_topic").value,
            10,
        )

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
                pose=(
                    payload.get("pose")
                    if isinstance(payload.get("pose"), dict) and payload.get("pose")
                    else None
                ),
            )
        except Exception as exc:
            self.get_logger().warn(f"Invalid seed: {exc}")
            return

        policy = self._policy_for(seed.label)#根据类别获取处理策略
        snapped = None
        if bool(policy.get("snap_to_occupancy", True)):
            snapped = self._projector.project(seed, self._map)
        if snapped is None and bool(policy.get("fallback_visual_on_reject", False)):
            snapped = self._visual_seed_object(seed, policy)
        if snapped is None:
            self.get_logger().debug(f"Rejected seed '{seed.label}' at ({seed.gx:.2f}, {seed.gy:.2f})")
            return

        # 核心改动：不再直接发布，而是送入记忆池更新
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        tracked = self.memory.update(snapped, now_sec)

        self.get_logger().info(
            f"[Memory] {tracked.object_id} {tracked.state}: pos=({tracked.x:.2f}, {tracked.y:.2f}), "
            f"seen={tracked.times_seen}, source={tracked.source}"
        )

    def _publish_memory_cb(self) -> None:
        """定时老化并发布所有存活的对象"""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.memory.age(now_sec) #清除过期对象
        
        active_objects = self.memory.get_active_objects(
            include_candidates=bool(self.get_parameter("show_candidates").value)
        )
        confirmed_objects = self.memory.get_active_objects(include_candidates=False)
        self._publish_semantic_objects(confirmed_objects, now_sec)
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

            if obj.state == "confirmed":
                marker.color.r = 0.2
                marker.color.g = 0.8
                marker.color.b = 0.3
            else:
                marker.color.r = 1.0
                marker.color.g = 0.65
                marker.color.b = 0.1
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
            text.text = (
                f"{obj.object_id}{obj.state}\n"
                f"{obj.source}({obj.confidence*100:.0f}%)\n"
                f"n={obj.times_seen}"
            )
            if obj.pose:
                posture = obj.pose.get("posture")
                actions = obj.pose.get("action_tags", [])
                semantic_parts = ([str(posture)] if posture else []) + [
                    str(action) for action in actions
                ]
                if semantic_parts:
                    text.text += "\n" + ", ".join(semantic_parts)

            ma.markers.extend([marker, text])

        self._pub_marker.publish(ma)

    def _publish_semantic_objects(
        self,
        active_objects: list[TrackedObject],
        now_sec: float,
    ) -> None:
        """Publish and overwrite the JSON semantic-object snapshot.

        The JSON file is intentionally rewritten on every timer tick.  This
        keeps it as a current-state document rather than an append-only log, so
        expired objects disappear from disk as soon as they disappear from RViz.
        """
        snapshot = semantic_objects_snapshot(active_objects, now_sec, frame_id="map")
        text = json.dumps(snapshot, ensure_ascii=False, indent=2)
        self._pub_objects.publish(String(data=text))

        output_path = str(self.get_parameter("semantic_objects_path").value or "").strip()
        if not output_path:
            return
        path = Path(output_path).expanduser()
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            tmp_path = path.with_suffix(path.suffix + ".tmp")
            tmp_path.write_text(text + "\n", encoding="utf-8")
            tmp_path.replace(path)
        except Exception as exc:
            self.get_logger().warn(f"Failed to write semantic objects JSON: {exc}")

    def _load_label_policies(self) -> dict[str, dict]:
        policies = {
            label: dict(policy)
            for label, policy in DEFAULT_LABEL_POLICIES.items()
        }
        raw = str(self.get_parameter("label_policy_json").value or "").strip()
        if raw:
            try:
                user_policies = json.loads(raw)
                if isinstance(user_policies, dict):
                    for label, policy in user_policies.items():
                        if isinstance(policy, dict):
                            key = self._normalize_label(label)
                            base = dict(policies.get(key, policies["default"]))
                            base.update(policy)
                            policies[key] = base
            except Exception as exc:
                self.get_logger().warn(f"Invalid label_policy_json: {exc}")
        return policies

    def _policy_for(self, label: str) -> dict:
        key = self._normalize_label(label)
        return self._label_policies.get(key, self._label_policies["default"])

    def _visual_seed_object(self, seed: SemanticSeed, policy: dict) -> SnappedObject | None:
        if self._map is None:
            return None
        cell = self._map.world_to_grid(seed.gx, seed.gy)#转换坐标
        if cell is None:
            return None
        if self._map.is_occupied(*cell):
            return None
        clearance_m = float(policy.get("visual_clearance_m", 0.35))
        if clearance_m > 0.0 and self._has_occupied_cell_near(seed.gx, seed.gy, clearance_m):
            return None
        size = float(policy.get("visual_size_m", 0.35))
        return SnappedObject(
            label=self._normalize_label(seed.label),
            confidence=float(seed.confidence),
            center_x=float(seed.gx),
            center_y=float(seed.gy),
            size_x=size,
            size_y=size,
            source="visual_only",
            pose=seed.pose,
        )

    def _has_occupied_cell_near(self, wx: float, wy: float, radius_m: float) -> bool:
        if self._map is None:
            return False
        center = self._map.world_to_grid(wx, wy)
        if center is None:
            return False
        radius_cells = int(radius_m / self._map.resolution) + 1
        cgx, cgy = center
        for gy in range(cgy - radius_cells, cgy + radius_cells + 1):
            for gx in range(cgx - radius_cells, cgx + radius_cells + 1):
                if not self._map.is_occupied(gx, gy):
                    continue
                ox, oy = self._map.grid_to_world_center(gx, gy)
                if ((ox - wx) ** 2 + (oy - wy) ** 2) ** 0.5 <= radius_m:
                    return True
        return False

    @staticmethod
    def _normalize_label(label: str) -> str:
        return (label.strip() or "unknown").lower().replace(" ", "_")

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
