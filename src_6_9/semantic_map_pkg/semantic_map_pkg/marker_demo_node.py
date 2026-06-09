"""ROS 2 demo node that publishes semantic rectangles as MarkerArray."""

from __future__ import annotations

from typing import List

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from semantic_map_pkg.fusion import SemanticObjectTracker
from semantic_map_pkg.projection import ProjectedObject
from semantic_map_pkg.visualization import object_to_rectangle

'''
ros2 run nav2_map_server map_server --ros-args --params-file config.yaml
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/weiyu/my_slam_map.yaml
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
'''
class SemanticMarkerDemoNode(Node):
    """Publish a small, stable semantic overlay in the map frame."""

    def __init__(self) -> None:
        super().__init__("semantic_marker_demo")
        self.publisher = self.create_publisher(MarkerArray, "/semantic_map/markers", 10)
        self.timer = self.create_timer(0.5, self._on_timer)
        self.tracker = SemanticObjectTracker()
        self._tick = 0
        self.get_logger().info("Publishing demo semantic markers on /semantic_map/markers")

    def _on_timer(self) -> None:
        self._tick += 1
        dx = 0.03 if self._tick % 4 < 2 else -0.03
        projected = self._build_demo_projected_objects(dx)
        objects = self.tracker.update(projected)
        #maker数据结构
        markers = MarkerArray()
        marker_id = 0
        for item in objects:
            rect = object_to_rectangle(item)
            line = Marker()
            line.header.frame_id = "map"#marker的坐标系
            line.header.stamp = self.get_clock().now().to_msg()#marker的时间戳
            line.ns = "semantic_rectangles"#marker的命名空间
            line.id = marker_id#marker的id，必须唯一
            marker_id += 1#marker的类型，这里使用LINE_STRIP来画矩形框
            line.type = Marker.LINE_STRIP#marker的操作，这里使用ADD来添加marker
            line.action = Marker.ADD
            line.scale.x = 0.06#marker的线宽
            line.color.r, line.color.g, line.color.b, line.color.a = rect.color_rgba
            line.pose.orientation.w = 1.0#无旋转，点坐标在map系
            line.lifetime.sec = 1#marker的生命周期，这里设置为1秒，过期后会自动删除
            line.points = [Point(x=x, y=y, z=0.05) for x, y in rect.points]
            markers.markers.append(line)
            #文字标签
            text = Marker()
            text.header.frame_id = "map"
            text.header.stamp = line.header.stamp
            text.ns = "semantic_labels"
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = item.center_x
            text.pose.position.y = item.center_y
            text.pose.position.z = 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = 0.25
            text.color.r, text.color.g, text.color.b, text.color.a = rect.color_rgba
            text.lifetime.sec = 1
            text.text = f"{item.display_label} ({item.id})"
            markers.markers.append(text)

        self.publisher.publish(markers)

    def _build_demo_projected_objects(self, dx: float) -> List[ProjectedObject]:
        return [
            ProjectedObject(
                label="sofa",
                display_label="沙发",
                confidence=0.92,
                center_x=1.8 + dx,
                center_y=0.2,
                yaw=0.12,
                size_x=1.8,
                size_y=0.9,
                source_frame="camera_link",
            ),
            ProjectedObject(
                label="tea_table",
                display_label="茶几",
                confidence=0.88,
                center_x=1.1,
                center_y=-0.1 + dx * 0.3,
                yaw=0.0,
                size_x=1.0,
                size_y=0.6,
                source_frame="camera_link",
            ),
            ProjectedObject(
                label="toilet",
                display_label="马桶",
                confidence=0.9,
                center_x=-0.6,
                center_y=1.3,
                yaw=-0.05,
                size_x=0.8,
                size_y=0.8,
                source_frame="camera_link",
            ),
            ProjectedObject(
                label="chair",
                display_label="椅子",
                confidence=0.86,
                center_x=-1.2 + dx * 0.2,
                center_y=-0.9,
                yaw=0.25,
                size_x=0.6,
                size_y=0.6,
                source_frame="camera_link",
            ),
        ]


def main() -> None:
    rclpy.init()
    node = SemanticMarkerDemoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
