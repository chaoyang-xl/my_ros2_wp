#!/usr/bin/env python3
"""Resolve semantic object IDs into reachable Nav2 approach goals."""

from __future__ import annotations

import json
from math import atan2, cos, sin

import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from semantic_navigation_pkg.semantic_map import (
    GridMap,
    SemanticMap,
    SemanticObject,
    choose_approach_pose,
)


class SemanticGoalResolverNode(Node):
    """Turn a semantic command into a collision-checked NavigateToPose goal."""

    def __init__(self) -> None:
        super().__init__("semantic_goal_resolver_node")
        self.declare_parameter("semantic_objects_topic", "/semantic_map/objects")
        self.declare_parameter("command_topic", "/semantic_navigation/goal")
        self.declare_parameter("status_topic", "/semantic_navigation/status")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("navigate_action", "/navigate_to_pose")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("reject_unknown", True)
        self.declare_parameter("standoff_m", 0.7)
        self.declare_parameter("robot_clearance_m", 0.35)
        self.declare_parameter("approach_samples", 32)
        self.declare_parameter("action_server_timeout_s", 2.0)
        self.declare_parameter("behavior_tree", "")

        durable_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("semantic_objects_topic").value),
            self._objects_cb,
            durable_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._map_cb,
            durable_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("command_topic").value),
            self._command_cb,
            10,
        )
        self._status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self._navigate_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigate_action").value),
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._semantic_map: SemanticMap | None = None
        self._grid: GridMap | None = None
        self._goal_active = False

    def _objects_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self._semantic_map = self._parse_loader_snapshot(payload)
        except Exception as exc:
            self.get_logger().error(f"Invalid semantic map topic payload: {exc}")

    def _map_cb(self, msg: OccupancyGrid) -> None:
        q = msg.info.origin.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._grid = GridMap(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            origin_yaw=atan2(siny, cosy),
            data=msg.data,
            occupied_threshold=int(self.get_parameter("occupied_threshold").value),
            reject_unknown=bool(self.get_parameter("reject_unknown").value),
        )

    def _command_cb(self, msg: String) -> None:
        try:
            object_id, standoff, clearance = self._parse_command(msg.data)
            self._start_navigation(object_id, standoff, clearance)
        except Exception as exc:
            self._publish_status("rejected", error=str(exc))
            self.get_logger().warn(f"Semantic navigation command rejected: {exc}")

    def _start_navigation(
        self,
        object_id: str,
        standoff_m: float,
        clearance_m: float,
    ) -> None:
        if self._goal_active:
            raise RuntimeError("a semantic navigation goal is already active")
        if self._semantic_map is None:
            raise RuntimeError("semantic map has not been received")
        if self._grid is None:
            raise RuntimeError("occupancy map has not been received")
        obj = self._semantic_map.object_by_id(object_id)
        if obj is None:
            raise ValueError(f"unknown semantic object id: {object_id}")

        frame_id = self._semantic_map.frame_id
        base_frame = str(self.get_parameter("base_frame").value)
        try:
            transform = self._tf_buffer.lookup_transform(
                frame_id,
                base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as exc:
            raise RuntimeError(f"TF unavailable: {frame_id} <- {base_frame}: {exc}") from exc

        pose = choose_approach_pose(
            self._grid,
            obj,
            robot_x=transform.transform.translation.x,
            robot_y=transform.transform.translation.y,
            standoff_m=standoff_m,
            clearance_m=clearance_m,
            sample_count=int(self.get_parameter("approach_samples").value),
        )
        if pose is None:
            raise RuntimeError(f"no collision-free approach pose for {object_id}")

        timeout = float(self.get_parameter("action_server_timeout_s").value)
        if not self._navigate_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("Nav2 NavigateToPose action server is unavailable")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose.x
        goal.pose.pose.position.y = pose.y
        goal.pose.pose.orientation.z = sin(pose.yaw / 2.0)
        goal.pose.pose.orientation.w = cos(pose.yaw / 2.0)
        goal.behavior_tree = str(self.get_parameter("behavior_tree").value)

        self._goal_active = True
        self._publish_status(
            "sending",
            object_id=object_id,
            goal={"x": pose.x, "y": pose.y, "yaw": pose.yaw},
        )
        future = self._navigate_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed: self._goal_response_cb(completed, object_id)
        )

    def _goal_response_cb(self, future, object_id: str) -> None:
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._goal_active = False
                self._publish_status("rejected_by_nav2", object_id=object_id)
                return
            self._publish_status("accepted", object_id=object_id)
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda completed: self._result_cb(completed, object_id)
            )
        except Exception as exc:
            self._goal_active = False
            self._publish_status("error", object_id=object_id, error=str(exc))

    def _result_cb(self, future, object_id: str) -> None:
        self._goal_active = False
        try:
            wrapped_result = future.result()
            self._publish_status(
                "finished",
                object_id=object_id,
                nav2_status=int(wrapped_result.status),
            )
        except Exception as exc:
            self._publish_status("error", object_id=object_id, error=str(exc))

    def _parse_command(self, text: str) -> tuple[str, float, float]:
        stripped = text.strip()
        if not stripped:
            raise ValueError("empty semantic navigation command")
        default_standoff = float(self.get_parameter("standoff_m").value)
        default_clearance = float(self.get_parameter("robot_clearance_m").value)
        if not stripped.startswith("{"):
            return stripped, default_standoff, default_clearance

        payload = json.loads(stripped)
        object_id = str(payload.get("object_id", "")).strip()
        if not object_id:
            raise ValueError("command object_id is required")
        standoff = float(payload.get("standoff_m", default_standoff))
        clearance = float(payload.get("clearance_m", default_clearance))
        if standoff < 0.0 or clearance < 0.0:
            raise ValueError("standoff_m and clearance_m must be non-negative")
        return object_id, standoff, clearance

    @staticmethod
    def _parse_loader_snapshot(payload: object) -> SemanticMap:
        if not isinstance(payload, dict) or not isinstance(payload.get("objects"), list):
            raise ValueError("expected a semantic-map JSON object with an objects array")
        frame_id = str(payload.get("frame_id", "map")).strip() or "map"
        objects = []
        for raw in payload["objects"]:
            objects.append(
                SemanticObject(
                    object_id=str(raw["id"]),
                    label=str(raw["label"]),
                    x=float(raw["x"]),
                    y=float(raw["y"]),
                    size_x=max(0.0, float(raw.get("size_x", 0.0))),
                    size_y=max(0.0, float(raw.get("size_y", 0.0))),
                    confidence=float(raw.get("confidence", 0.0)),
                    source=str(raw.get("source", "unknown")),
                )
            )
        return SemanticMap(frame_id=frame_id, objects=tuple(objects))

    def _publish_status(self, state: str, **fields) -> None:
        payload = {"state": state, **fields}
        self._status_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SemanticGoalResolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

