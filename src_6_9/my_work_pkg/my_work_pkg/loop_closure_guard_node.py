#!/usr/bin/env python3
"""ROS 2 node: semantic projection + loop-closure-aware coordinate correction.

Cartographer 回环检测后会重新优化全局位姿图，导致 ``map`` 坐标系整体偏移。
本节点在 ``semantic_projection_node`` 全部功能之上增加了对 ``map -> odom``
TF 跳变的监控——一旦检测到回环，自动修正记忆池中所有已追踪对象的位置。

用法：直接替代 ``semantic_projection_node``，不需要同时跑两个。
"""

from __future__ import annotations

import json
from math import cos, hypot, sin
from typing import cast

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
    TrackedObject,
)


# ---------------------------------------------------------------------------
# 2D rigid-transform helpers
# ---------------------------------------------------------------------------

def _compose_2d(x: float, y: float, yaw: float,
                px: float, py: float) -> tuple[float, float]:
    """Apply 2D rigid transform (x, y, yaw) to point (px, py)."""
    return (
        cos(yaw) * px - sin(yaw) * py + x,
        sin(yaw) * px + cos(yaw) * py + y,
    )


def _inverse_2d(x: float, y: float, yaw: float) -> tuple[float, float, float]:
    """Inverse of a 2D rigid transform.

    T  :  P_out = R(yaw) * P_in + (x, y)
    T⁻¹:  P_in  = R(-yaw) * P_out  -  R(-yaw) * [x; y]
    """
    c = cos(yaw)   # cos(-yaw)
    s = -sin(yaw)  # sin(-yaw) … R(-yaw) = [[c, -s], [s, c]]
    # -R(-yaw) * [x; y] = -[c*x - s*y;  s*x + c*y]
    return (-c * x + s * y, -s * x - c * y, -yaw)


def _transform_old_map_to_new_map(
    px: float, py: float,
    tf_old: tuple[float, float, float],
    tf_new: tuple[float, float, float],
) -> tuple[float, float]:
    """Correct a point in old-map frame into new-map frame.

    ``tf_old`` / ``tf_new`` are the ``map -> odom`` transforms before and
    after loop closure, each given as ``(x, y, yaw)``.

    Derivation: for a world-fixed point P in odom:
        P_odom = T_old(P_old_map) = T_new(P_new_map)
    =>  P_new_map = T_new^{-1}(T_old(P_old_map))
    """
    # 1. old-map → odom
    ox, oy = _compose_2d(*tf_old, px, py)
    # 2. odom → new-map  (inverse of tf_new)
    inv_x, inv_y, inv_yaw = _inverse_2d(*tf_new)
    return _compose_2d(inv_x, inv_y, inv_yaw, ox, oy)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class LoopClosureGuardNode(Node):
    """Semantic projection node that survives Cartographer loop closure."""

    def __init__(self) -> None:
        super().__init__("loop_closure_guard_node")

        # ---- semantic projection parameters (same as semantic_projection_node) ----
        self.declare_parameter("search_radius_m", 0.5)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("min_island_pixels", 2)
        self.declare_parameter("match_distance", 1.0)
        self.declare_parameter("smoothing_alpha", 0.3)
        self.declare_parameter("memory_timeout", np.inf)

        # ---- loop-closure detection parameters ----
        self.declare_parameter("lc_check_period", 1.0)
        self.declare_parameter("lc_translation_threshold", 0.05)   # metres
        self.declare_parameter("lc_rotation_threshold", 0.02)      # radians ≈1.1°
        self.declare_parameter("lc_source_frame", "odom")          # watch map→odom

        self._lc_period = self.get_parameter("lc_check_period").value
        self._lc_trans_thresh = self.get_parameter("lc_translation_threshold").value
        self._lc_rot_thresh = self.get_parameter("lc_rotation_threshold").value
        self._lc_source = self.get_parameter("lc_source_frame").value
        self._target_frame = "map"

        # ---- projector & memory ----
        self._projector = SinglePointSemanticProjector(
            search_radius_m=self.get_parameter("search_radius_m").value,
            min_total_pixels=self.get_parameter("min_island_pixels").value,
        )
        self.memory = SemanticMemory(
            match_distance=self.get_parameter("match_distance").value,
            alpha=self.get_parameter("smoothing_alpha").value,
            timeout=self.get_parameter("memory_timeout").value,
        )

        # ---- TF ----
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---- last known map->odom transform for loop-closure detection ----
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

        snapped = self._projector.project(seed, self._map)
        if snapped is None:
            self.get_logger().debug(
                f"Rejected seed '{seed.label}' at ({seed.gx:.2f}, {seed.gy:.2f})"
            )
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        tracked = self.memory.update(snapped, now_sec)

        self.get_logger().info(
            f"[Memory] {tracked.object_id} updated: "
            f"pos=({tracked.x:.2f}, {tracked.y:.2f}), seen={tracked.times_seen}"
        )

    def _publish_memory_cb(self) -> None:
        """Age objects and publish MarkerArray (same as semantic_projection_node)."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.memory.age(now_sec)

        active = self.memory.get_active_objects()
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
            marker.color.r = 0.2
            marker.color.g = 0.8
            marker.color.b = 0.3
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
            text.text = f"{obj.object_id}({obj.confidence * 100:.0f}%)n={obj.times_seen}"

            ma.markers.extend([marker, text])

        self._pub_marker.publish(ma)

    # ==================================================================
    # Loop-closure detection & correction
    # ==================================================================

    def _check_loop_closure_cb(self) -> None:
        """Periodically compare ``map -> odom`` against last known value.

        A significant jump signals a Cartographer loop-closure event.
        We then compute the correction transform and apply it to every
        tracked object in the memory pool.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._target_frame,          # map
                self._lc_source,             # odom
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
                f"Initialised map->odom reference: ({tx:.3f}, {ty:.3f}, {yaw:.3f})"
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

        active = self.memory.get_active_objects()
        if not active:
            self._last_map_odom = current
            return

        corrected = 0
        for obj in active:
            nx, ny = _transform_old_map_to_new_map(obj.x, obj.y, old, current)
            d = hypot(nx - obj.x, ny - obj.y)
            if d > 0.001:  # only count non-trivial moves
                corrected += 1
            obj.x = nx
            obj.y = ny

        self._last_map_odom = current

        self.get_logger().info(
            f"[LoopClosure] Applied correction to {corrected}/{len(active)} "
            f"objects (max shift: see above jump magnitude)"
        )


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
