#!/usr/bin/env python3
"""ROS 2 node: semantic projection + loop-closure-aware coordinate correction.

Cartographer 回环检测后会重新优化全局位姿图，导致 ``map`` 坐标系整体偏移。
本节点在 ``semantic_projection_node`` 全部功能之上增加了对 ``odom -> map``
TF 跳变的监控——一旦检测到回环，自动修正记忆池中所有已追踪对象的位置。

用法：直接替代 ``semantic_projection_node``，不需要同时跑两个。
"""

from __future__ import annotations

import json
from math import hypot
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from visualization_msgs.msg import Marker, MarkerArray

from my_work_pkg.semantic_projection import (
    OccupancyGridMap,
    SemanticSeed,
    SinglePointSemanticProjector,
    SemanticMemory,
    SnappedObject,
    TrackedObject,
    semantic_objects_snapshot,
)
from my_work_pkg.semantic_projection_node import DEFAULT_LABEL_POLICIES
from my_work_pkg.loop_closure_geometry import transform_old_map_to_new_map


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class LoopClosureGuardNode(Node):
    """Semantic projection node that survives Cartographer loop closure."""

    def __init__(self) -> None:
        super().__init__("loop_closure_guard_node")

        # ---- semantic projection parameters (same as semantic_projection_node) ----
        self.declare_parameter("search_radius_m", 0.2)
        self.declare_parameter("occupied_threshold", 80)
        self.declare_parameter("min_island_pixels", 2)
        self.declare_parameter("max_island_size_m", 2.0)
        self.declare_parameter("max_total_pixels", 0)
        self.declare_parameter("match_distance", 1.0)
        self.declare_parameter("smoothing_alpha", 0.3)
        self.declare_parameter("memory_timeout", 5.0)
        self.declare_parameter("min_confirmed_seen", 50)
        self.declare_parameter("show_candidates", False)
        self.declare_parameter("label_policy_json", "")
        self.declare_parameter("semantic_objects_topic", "/semantic_objects")
        self.declare_parameter("semantic_objects_path", "/tmp/semantic_objects.json")

        # 与 semantic_projection_node 保持一致：按类别决定是否吸附、
        # 是否允许 visual_only fallback、以及确认所需观测次数。
        self._label_policies = self._load_label_policies()

        # ---- loop-closure detection parameters ----
        self.declare_parameter("lc_check_period", 1.0)
        self.declare_parameter("lc_translation_threshold", 0.20)   # metres
        self.declare_parameter("lc_rotation_threshold", 0.08)      # radians ≈4°
        self.declare_parameter("lc_source_frame", "odom")          # watch odom→map
        self.declare_parameter("lc_resnap_enabled", True)
        self.declare_parameter("lc_resnap_radius_m", 0.3)

        self._lc_period = self.get_parameter("lc_check_period").value
        self._lc_trans_thresh = self.get_parameter("lc_translation_threshold").value
        self._lc_rot_thresh = self.get_parameter("lc_rotation_threshold").value
        self._lc_source = self.get_parameter("lc_source_frame").value
        self._target_frame = "map"
        self._lc_resnap_enabled = bool(self.get_parameter("lc_resnap_enabled").value)

        # ---- projector & memory ----
        self._projector = SinglePointSemanticProjector(
            search_radius_m=self.get_parameter("search_radius_m").value,
            min_total_pixels=self.get_parameter("min_island_pixels").value,
            max_island_size_m=self.get_parameter("max_island_size_m").value,
            max_total_pixels=self.get_parameter("max_total_pixels").value,
        )
        # 回环后专用的小半径吸附器。只用于已吸附到地图结构的
        # snapped 对象在新 /map 上重新贴合，不影响正常 seed 投影半径。
        self._resnap_projector = SinglePointSemanticProjector(
            search_radius_m=self.get_parameter("lc_resnap_radius_m").value,
            min_total_pixels=self.get_parameter("min_island_pixels").value,
            max_island_size_m=self.get_parameter("max_island_size_m").value,
            max_total_pixels=self.get_parameter("max_total_pixels").value,
        )
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

        # ---- TF ----
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---- last known odom->map transform for loop-closure detection ----
        self._last_map_odom: tuple[float, float, float] | None = None

        # ---- subscriptions (same as semantic_projection_node) ----
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

        # ---- timers ----
        # Marker publishing (same as before)
        self.create_timer(0.5, self._publish_memory_cb)
        # Loop-closure TF monitor
        self.create_timer(self._lc_period, self._check_loop_closure_cb)

        self.get_logger().info(
            "LoopClosureGuardNode Ready (snap + track + loop-closure-aware)."
        )

    # ==================================================================
    # Semantic projection callbacks  (identical to semantic_projection_node)
    # ==================================================================

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = OccupancyGridMap(
            data=msg.data,
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            occupied_threshold=int(self.get_parameter("occupied_threshold").value),
        )

    def _seed_cb(self, msg: String) -> None:
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

        # 与 semantic_projection_node 一致：
        # 1. 先按类别策略决定是否尝试 OccupancyGrid 吸附。
        # 2. 吸附失败时，只有策略允许才保留 visual_only 点。
        # 3. visual_only 还会检查和 occupied cell 的安全距离，避免墙边刷点。
        policy = self._policy_for(seed.label)
        snapped = None
        if bool(policy.get("snap_to_occupancy", True)):
            snapped = self._projector.project(seed, self._map)
        if snapped is None and bool(policy.get("fallback_visual_on_reject", False)):
            snapped = self._visual_seed_object(seed, policy)
        if snapped is None:
            self.get_logger().debug(
                f"Rejected seed '{seed.label}' at ({seed.gx:.2f}, {seed.gy:.2f})"
            )
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        tracked = self.memory.update(snapped, now_sec)

        self.get_logger().info(
            f"[Memory] {tracked.object_id} updated: "
            f"pos=({tracked.x:.2f}, {tracked.y:.2f}), seen={tracked.times_seen}, "
            f"state={tracked.state}, source={tracked.source}"
        )

    def _publish_memory_cb(self) -> None:
        """Age objects and publish MarkerArray (same as semantic_projection_node)."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.memory.age(now_sec)

        active = self.memory.get_active_objects(
            include_candidates=bool(self.get_parameter("show_candidates").value)
        )
        confirmed_objects = self.memory.get_active_objects(include_candidates=False)
        self._publish_semantic_objects(confirmed_objects, now_sec)
        if not active:
            return

        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, obj in enumerate(active):
            # ---- hollow rectangle ----
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "tracked_objects_box"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()

            marker.pose.position.x = obj.x
            marker.pose.position.y = obj.y
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05

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

            # ---- text label ----
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
            text.text = (
                f"{obj.object_id} {obj.state} "
                f"{obj.source}({obj.confidence * 100:.0f}%)n={obj.times_seen}"
            )

            ma.markers.extend([marker, text])

        self._pub_marker.publish(ma)

    def _publish_semantic_objects(
        self,
        active_objects: list[TrackedObject],
        now_sec: float,
    ) -> None:
        """Publish and overwrite the JSON semantic-object snapshot.

        This mirrors semantic_projection_node.  The file is a current-state
        document, so every timer tick rewrites it with exactly the objects that
        would be published as markers.
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

    # ==================================================================
    # Label policy helpers  (same semantic behaviour as semantic_projection_node)
    # ==================================================================

    def _load_label_policies(self) -> dict[str, dict]:
        """Load built-in label policies and optional JSON overrides.

        ``label_policy_json`` lets launch files adjust behaviour without editing
        code, for example:
        {"person":{"visual_clearance_m":0.6},"chair":{"min_confirmed_seen":3}}
        """
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
        """Create a visual-only semantic object when occupancy snapping is skipped.

        This is mainly for dynamic objects such as ``person``.  The clearance
        check prevents rejected wall-near seeds from becoming marker clutter.
        """
        if self._map is None:
            return None
        cell = self._map.world_to_grid(seed.gx, seed.gy)
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
        )

    def _has_occupied_cell_near(self, wx: float, wy: float, radius_m: float) -> bool:
        """Return True when a visual-only seed is too close to a wall/obstacle."""
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

    # ==================================================================
    # Loop-closure detection & correction
    # ==================================================================

    def _check_loop_closure_cb(self) -> None:
        """Periodically compare ``odom -> map`` against last known value.

        A significant jump signals a Cartographer loop-closure event.
        We then compute the correction transform and apply it to every
        tracked object in the memory pool.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._target_frame,          # target: map
                self._lc_source,             # source: odom
                Time(seconds=0).to_msg(),    # latest available
                timeout=Duration(seconds=0.2),
            )
        except (LookupException, ConnectivityException,
                ExtrapolationException):
            # TF chain not ready yet — nothing to compare
            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        # Convert quaternion to yaw (we only care about 2D)
        q = tf.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny, cosy)

        current: tuple[float, float, float] = (tx, ty, yaw)

        if self._last_map_odom is None:
            # First reading — just record, no correction needed
            self._last_map_odom = current
            self.get_logger().debug(
                f"Initialised odom->map reference: ({tx:.3f}, {ty:.3f}, {yaw:.3f})"
            )
            return

        old = self._last_map_odom
        dtrans = hypot(current[0] - old[0], current[1] - old[1])
        drot = abs(_normalise_angle(current[2] - old[2]))

        if dtrans < self._lc_trans_thresh and drot < self._lc_rot_thresh:
            # No sudden jump — absorb slow drift into the reference
            self._last_map_odom = current
            return

        # ---- loop closure detected! ----
        self.get_logger().warn(
            f"[LoopClosure] TF jump detected: "
            f"dtrans={dtrans:.3f}m, drot={drot:.3f}rad. "
            f"Old=({old[0]:.3f},{old[1]:.3f},{old[2]:.3f}) "
            f"New=({tx:.3f},{ty:.3f},{yaw:.3f})"
        )

        # 回环修正应覆盖整个记忆池，包括还没 confirmed 的候选对象。
        active = self.memory.get_active_objects(include_candidates=True)
        if not active:
            self._last_map_odom = current
            return

        corrected = 0
        resnapped = 0
        resnap_failed = 0
        visual_only = 0
        for obj in active:
            nx, ny = transform_old_map_to_new_map(obj.x, obj.y, old, current)
            d = hypot(nx - obj.x, ny - obj.y)
            if d > 0.001:  # only count non-trivial moves
                corrected += 1

            # visual_only 对象不依赖 OccupancyGrid。回环后只应用几何修正，
            # 不做 re-snap，避免动态/纯视觉目标被拉到墙体或静态障碍上。
            if obj.source != "snapped":
                obj.x = nx
                obj.y = ny
                visual_only += 1
                continue

            # snapped 对象代表已经和地图占据结构绑定的静态语义目标。
            # 先用 TF 几何修正得到新 map 里的预测位置，再在新 /map 上
            # 小半径 re-snap，更新中心和尺寸；失败时保留几何修正结果。
            if self._lc_resnap_enabled and self._map is not None:
                if self._resnap_object(obj, nx, ny):
                    resnapped += 1
                    continue
                resnap_failed += 1

            obj.x = nx
            obj.y = ny

        self._last_map_odom = current

        self.get_logger().info(
            f"[LoopClosure] Applied correction to {corrected}/{len(active)} objects; "
            f"resnapped={resnapped}, resnap_failed={resnap_failed}, "
            f"visual_only={visual_only}"
        )

    def _resnap_object(self, obj: TrackedObject, predicted_x: float, predicted_y: float) -> bool:
        """Re-snap one snapped object on the current /map near its corrected pose.

        This is a map-coordinate maintenance step, not a new observation:
        ``times_seen``, ``confidence`` and ``last_seen`` are intentionally kept.
        On success only geometric properties are refreshed.
        """
        if self._map is None:
            return False

        seed = SemanticSeed(
            label=obj.label,
            confidence=obj.confidence,
            gx=predicted_x,
            gy=predicted_y,
        )
        snapped = self._resnap_projector.project(seed, self._map)
        if snapped is None:
            return False

        obj.x = snapped.center_x
        obj.y = snapped.center_y
        obj.size_x = snapped.size_x
        obj.size_y = snapped.size_y
        obj.source = snapped.source
        return True


def _normalise_angle(a: float) -> float:
    """Wrap angle into [-pi, pi)."""
    return ((a + np.pi) % (2.0 * np.pi)) - np.pi


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LoopClosureGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
