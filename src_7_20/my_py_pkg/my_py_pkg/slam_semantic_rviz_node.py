#!/usr/bin/env python3
"""Publish a saved SLAM map and semantic object markers for RViz."""

from __future__ import annotations

import colorsys
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


WORKSPACE_SRC = Path('/home/weiyu/vscode_workspace/ros2_wp/src')
DEFAULT_OUTPUT_ROOT = WORKSPACE_SRC / 'semantic_map_offline' / 'outputs' / 'semantic_map_05_processed_tf'
DEFAULT_MAP_YAML = DEFAULT_OUTPUT_ROOT / 'slam_map' / 'semantic_05.yaml'
DEFAULT_OBJECTS_JSON = DEFAULT_OUTPUT_ROOT / 'tracking_sam_final' / 'semantic_objects.json'


class SlamSemanticRvizNode(Node):
    def __init__(self) -> None:
        super().__init__('slam_semantic_rviz_node')
        self.declare_parameter('map_yaml', str(DEFAULT_MAP_YAML))
        self.declare_parameter('objects_json', str(DEFAULT_OBJECTS_JSON))
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('marker_topic', '/semantic_objects_markers')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_period_s', 1.0)
        self.declare_parameter('min_observations', 5)
        self.declare_parameter('min_confidence', 0.0)
        self.declare_parameter('classes', '')
        self.declare_parameter('rectangle_z', 0.02)
        self.declare_parameter('line_width', 0.05)
        self.declare_parameter('label_z', 0.18)
        self.declare_parameter('label_scale', 0.28)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('map_topic').value), map_qos
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('marker_topic').value), marker_qos
        )

        self._map_msg = self._load_map(Path(str(self.get_parameter('map_yaml').value)))
        self._marker_msg = self._load_markers(Path(str(self.get_parameter('objects_json').value)))

        self._publish()
        period = max(0.1, float(self.get_parameter('publish_period_s').value))
        self._timer = self.create_timer(period, self._publish)
        self.get_logger().info(
            f"Publishing SLAM map on {self.get_parameter('map_topic').value} and "
            f"{len(self._marker_msg.markers)} semantic 2D rectangle/label markers on "
            f"{self.get_parameter('marker_topic').value}"
        )

    def _load_map(self, map_yaml: Path) -> OccupancyGrid:
        yaml_path = map_yaml.expanduser().resolve()
        if not yaml_path.is_file():
            raise FileNotFoundError(f'Map YAML not found: {yaml_path}')
        document = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
        if not isinstance(document, dict):
            raise ValueError(f'Invalid map YAML: {yaml_path}')

        image_path = Path(str(document.get('image', ''))).expanduser()
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        image_path = image_path.resolve()
        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError(f'Failed to read map image: {image_path}')

        resolution = float(document.get('resolution', 0.0))
        origin = [float(v) for v in document.get('origin', [])]
        if resolution <= 0.0:
            raise ValueError('Map resolution must be positive')
        if len(origin) != 3:
            raise ValueError('Map origin must be [x, y, yaw]')

        negate = int(document.get('negate', 0))
        occupied_thresh = float(document.get('occupied_thresh', 0.65))
        free_thresh = float(document.get('free_thresh', 0.196))

        # nav2/map_server stores data from the bottom row upward, while image files
        # are top-left origin. Flip vertically before flattening.
        flipped = np.flipud(image).astype(np.float32)
        if negate:
            occupancy_prob = flipped / 255.0
        else:
            occupancy_prob = (255.0 - flipped) / 255.0
        data = np.full(flipped.shape, -1, dtype=np.int8)
        data[occupancy_prob > occupied_thresh] = 100
        data[occupancy_prob < free_thresh] = 0

        msg = OccupancyGrid()
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        msg.info = MapMetaData()
        msg.info.resolution = resolution
        msg.info.width = int(image.shape[1])
        msg.info.height = int(image.shape[0])
        msg.info.origin.position.x = origin[0]
        msg.info.origin.position.y = origin[1]
        msg.info.origin.position.z = 0.0
        yaw = origin[2]
        msg.info.origin.orientation.z = float(np.sin(yaw * 0.5))
        msg.info.origin.orientation.w = float(np.cos(yaw * 0.5))
        msg.data = data.reshape(-1).astype(int).tolist()
        return msg

    def _load_markers(self, objects_json: Path) -> MarkerArray:
        path = objects_json.expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f'Semantic objects JSON not found: {path}')
        document = json.loads(path.read_text(encoding='utf-8'))
        objects = document.get('objects', document if isinstance(document, list) else [])
        if not isinstance(objects, list):
            raise ValueError('Semantic objects JSON must contain an objects list')

        allowed_classes = {
            item.strip().replace('_', ' ')
            for item in str(self.get_parameter('classes').value).split(',')
            if item.strip()
        }
        min_observations = int(self.get_parameter('min_observations').value)
        min_confidence = float(self.get_parameter('min_confidence').value)
        rectangle_z = float(self.get_parameter('rectangle_z').value)
        line_width = max(0.001, float(self.get_parameter('line_width').value))
        label_z = float(self.get_parameter('label_z').value)
        label_scale = max(0.01, float(self.get_parameter('label_scale').value))
        frame_id = str(self.get_parameter('frame_id').value)

        marker_array = MarkerArray()
        marker_id = 0
        for fallback_id, item in enumerate(objects, start=1):
            if not isinstance(item, dict):
                continue
            label = self._object_label(item)
            observations = self._object_observations(item)
            confidence = self._object_confidence(item)
            if observations < min_observations or confidence < min_confidence:
                continue
            if allowed_classes and label not in allowed_classes:
                continue

            bounds_min = item.get('bounds_min')
            bounds_max = item.get('bounds_max')
            if not (isinstance(bounds_min, list) and isinstance(bounds_max, list)):
                continue
            if len(bounds_min) < 2 or len(bounds_max) < 2:
                continue

            track_id = self._object_track_id(item, fallback_id)
            color = self._stable_rgb(track_id)

            rectangle = Marker()
            rectangle.header.frame_id = frame_id
            rectangle.ns = 'semantic_object_rectangles_2d'
            rectangle.id = marker_id
            marker_id += 1
            rectangle.type = Marker.LINE_STRIP
            rectangle.action = Marker.ADD
            rectangle.pose.orientation.w = 1.0
            rectangle.scale.x = line_width
            rectangle.color.r = color[0]
            rectangle.color.g = color[1]
            rectangle.color.b = color[2]
            rectangle.color.a = 1.0
            rectangle.points = self._rectangle_points_2d(bounds_min, bounds_max, rectangle_z)
            marker_array.markers.append(rectangle)

            center_x = (float(bounds_min[0]) + float(bounds_max[0])) * 0.5
            center_y = (float(bounds_min[1]) + float(bounds_max[1])) * 0.5
            label_marker = Marker()
            label_marker.header.frame_id = frame_id
            label_marker.ns = 'semantic_object_labels_2d'
            label_marker.id = marker_id
            marker_id += 1
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD
            label_marker.pose.position.x = center_x
            label_marker.pose.position.y = center_y
            label_marker.pose.position.z = label_z
            label_marker.pose.orientation.w = 1.0
            label_marker.scale.z = label_scale
            label_marker.color.r = 0.0
            label_marker.color.g = 1.0
            label_marker.color.b = 0.0
            label_marker.color.a = 1.0
            label_marker.text = label
            marker_array.markers.append(label_marker)
        return marker_array

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._map_msg.header.stamp = stamp
        self._map_pub.publish(self._map_msg)
        for marker in self._marker_msg.markers:
            marker.header.stamp = stamp
        self._marker_pub.publish(self._marker_msg)

    @staticmethod
    def _stable_rgb(track_id: int) -> tuple[float, float, float]:
        hue = (track_id * 0.61803398875) % 1.0
        return colorsys.hsv_to_rgb(hue, 0.82, 0.92)

    @staticmethod
    def _object_track_id(item: dict[str, Any], fallback: int) -> int:
        value = item.get('track_id', item.get('id'))
        if isinstance(value, int):
            return value
        if isinstance(value, str):
            digits = ''.join(ch for ch in value if ch.isdigit())
            if digits:
                return int(digits)
        return fallback

    @staticmethod
    def _object_label(item: dict[str, Any]) -> str:
        return str(item.get('class_name') or item.get('label') or 'object').replace('_', ' ')

    @staticmethod
    def _object_observations(item: dict[str, Any]) -> int:
        try:
            return int(item.get('times_seen', item.get('observation_count', 0)))
        except (TypeError, ValueError):
            return 0

    @staticmethod
    def _object_confidence(item: dict[str, Any]) -> float:
        try:
            return float(item.get('confidence', 0.0))
        except (TypeError, ValueError):
            return 0.0

    @staticmethod
    def _rectangle_points_2d(bounds_min: list[Any], bounds_max: list[Any], z: float) -> list[Point]:
        xmin = float(bounds_min[0])
        ymin = float(bounds_min[1])
        xmax = float(bounds_max[0])
        ymax = float(bounds_max[1])
        corners = [
            (xmin, ymin, z),
            (xmax, ymin, z),
            (xmax, ymax, z),
            (xmin, ymax, z),
            (xmin, ymin, z),
        ]
        points: list[Point] = []
        for x, y, point_z in corners:
            point = Point()
            point.x = x
            point.y = y
            point.z = point_z
            points.append(point)
        return points

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SlamSemanticRvizNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
