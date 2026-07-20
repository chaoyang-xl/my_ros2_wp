#!/usr/bin/env python3
"""Pure-pursuit path tracking controller for differential-drive robots."""

import math

from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def infer_motion_direction(path: list[tuple[float, float, float]], index: int) -> int:
    """Infer whether a path segment should be tracked forward or in reverse."""
    if len(path) < 2:
        return 1

    index = max(0, min(index, len(path) - 1))
    if index < len(path) - 1:
        x0, y0, yaw = path[index]
        x1, y1, _ = path[index + 1]
    else:
        x0, y0, yaw = path[index - 1]
        x1, y1, _ = path[index]

    segment_x = x1 - x0
    segment_y = y1 - y0
    heading_dot = segment_x * math.cos(yaw) + segment_y * math.sin(yaw)
    return 1 if heading_dot >= 0.0 else -1


class PurePursuitController(Node):
    """Track the planner's Path output with bounded velocity commands."""

    def __init__(self) -> None:
        super().__init__("pure_pursuit_controller")
        self.path = []
        self.path_frame = "map"
        self.current_pose = None
        self.closest_index = 0

        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("lookahead_distance", 0.4)
        self.declare_parameter("target_speed", 0.2)
        self.declare_parameter("goal_tolerance", 0.25)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("control_frequency", 20.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Path, self.get_parameter("path_topic").value, self.path_callback, 10
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 10
        )
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        frequency = float(self.get_parameter("control_frequency").value)
        if frequency <= 0.0:
            raise ValueError("control_frequency must be positive")
        self.timer = self.create_timer(1.0 / frequency, self.control_loop)
        self.get_logger().info("Pure pursuit controller is ready.")

    def path_callback(self, msg: Path) -> None:
        """Replace the active path with a newly planned one."""
        self.path_frame = (
            msg.header.frame_id or str(self.get_parameter("map_frame").value)
        )
        self.path = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
                self._get_yaw(pose.pose.orientation),
            )
            for pose in msg.poses
        ]
        self.closest_index = 0
        self.get_logger().info(f"Received a path with {len(self.path)} poses.")

    def odom_callback(self, msg: Odometry) -> None:
        """Update the current planar robot pose in the active path frame."""
        pose = self._pose_to_frame(
            msg.pose.pose,
            msg.header.frame_id,
            msg.header.stamp,
            self.path_frame,
            str(self.get_parameter("odom_topic").value).lstrip("/"),
        )
        if pose is None:
            self.current_pose = None
            return

        position = pose.position
        self.current_pose = (
            position.x,
            position.y,
            self._get_yaw(pose.orientation),
        )

    def _pose_to_frame(
        self,
        pose: Pose,
        frame_id: str,
        stamp,
        target_frame: str,
        default_source_frame: str,
    ):
        source_frame = frame_id or default_source_frame
        target_frame = target_frame or str(self.get_parameter("map_frame").value)
        if source_frame == target_frame:
            return pose

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, stamp, timeout=Duration(seconds=0.1)
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Cannot transform pose from {source_frame} to {target_frame}: {exc}"
            )
            return None
        return self._transform_pose(pose, transform.transform)

    @staticmethod
    def _transform_pose(pose: Pose, transform) -> Pose:
        rotation_yaw = PurePursuitController._get_yaw(transform.rotation)
        cos_yaw = math.cos(rotation_yaw)
        sin_yaw = math.sin(rotation_yaw)

        result = Pose()
        result.position.x = (
            cos_yaw * pose.position.x
            - sin_yaw * pose.position.y
            + transform.translation.x
        )
        result.position.y = (
            sin_yaw * pose.position.x
            + cos_yaw * pose.position.y
            + transform.translation.y
        )
        result.position.z = pose.position.z + transform.translation.z

        yaw = PurePursuitController._get_yaw(pose.orientation) + rotation_yaw
        result.orientation.z = math.sin(yaw / 2.0)
        result.orientation.w = math.cos(yaw / 2.0)
        return result

    @staticmethod
    def _get_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self) -> None:
        """Publish one bounded pure-pursuit control command."""
        if not self.path or self.current_pose is None:
            self._stop()
            return

        current_x, current_y, current_yaw = self.current_pose
        goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        distance_to_goal = math.hypot(
            self.path[-1][0] - current_x, self.path[-1][1] - current_y
        )
        if distance_to_goal <= goal_tolerance:
            self.path = []
            self._stop()
            self.get_logger().info("Goal reached.")
            return

        self.closest_index = min(
            range(self.closest_index, len(self.path)),
            key=lambda index: math.hypot(
                self.path[index][0] - current_x,
                self.path[index][1] - current_y,
            ),
        )
        lookahead = float(self.get_parameter("lookahead_distance").value)
        if lookahead <= 0.0:
            self.get_logger().error("lookahead_distance must be positive.")
            self._stop()
            return

        target_index = len(self.path) - 1
        for index in range(self.closest_index, len(self.path)):
            distance = math.hypot(
                self.path[index][0] - current_x,
                self.path[index][1] - current_y,
            )
            if distance >= lookahead:
                target_index = index
                break

        target_x, target_y, _ = self.path[target_index]
        alpha = normalize_angle(
            math.atan2(target_y - current_y, target_x - current_x) - current_yaw
        )
        direction = infer_motion_direction(self.path, self.closest_index)

        target_speed = abs(float(self.get_parameter("target_speed").value))
        signed_speed = direction * target_speed
        max_angular_speed = float(
            self.get_parameter("max_angular_speed").value
        )
        command = Twist()
        command.linear.x = signed_speed
        angular_speed = 2.0 * signed_speed * math.sin(alpha) / lookahead
        command.angular.z = max(
            -max_angular_speed, min(max_angular_speed, angular_speed)
        )
        self.cmd_pub.publish(command)

    def _stop(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    """Run the pure-pursuit controller node."""
    rclpy.init(args=args)
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._stop()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
