"""ROS 2 entry point for exporting a final Cartographer RGB-D dataset."""

from __future__ import annotations

from pathlib import Path
import traceback

from cartographer_ros_msgs.srv import TrajectoryQuery
import rclpy
from rclpy.node import Node

from semantic_map_offline.cartographer_dataset import (
    ExportConfig,
    export_dataset,
)
from semantic_map_offline.cartographer_trajectory import (
    PoseSample,
    TrajectoryInterpolator,
)


class CartographerDatasetExporterNode(Node):
    """Query a loaded pbstream trajectory and export synchronized bag frames."""

    def __init__(self) -> None:
        super().__init__("cartographer_dataset_exporter")
        defaults = (
            ("bag_uri", ""),
            ("output_directory", ""),
            ("jsonl_path", ""),
            ("trajectory_id", 0),
            ("trajectory_service", "/trajectory_query"),
            ("service_timeout_s", 30.0),
            ("color_topic", "/camera/color/image_raw"),
            ("depth_topic", "/camera/depth/image_raw"),
            ("camera_info_topic", "/camera/color/camera_info"),
            ("static_tf_topic", "/tf_static"),
            ("tracking_frame", "camera_gyro_frame"),
            ("camera_frame", "camera_color_optical_frame"),
            ("max_rgb_depth_time_diff_s", 0.05),
            ("max_detection_time_diff_s", 0.15),
            ("max_trajectory_interval_s", 10.0),
            ("input_depth_scale", 0.001),
            ("start_frame", 0),
            ("max_frames", 0),
            ("jpeg_quality", 95),
            ("overwrite", False),
            ("allow_resolution_mismatch", False),
        )
        for name, value in defaults:
            self.declare_parameter(name, value)

    def _required_path(self, name: str) -> Path:
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise ValueError(f"Required parameter {name!r} is empty")
        return Path(value).expanduser().resolve()

    def _optional_path(self, name: str) -> Path | None:
        value = str(self.get_parameter(name).value).strip()
        return Path(value).expanduser().resolve() if value else None

    def _query_trajectory(self) -> tuple[TrajectoryInterpolator, int, str]:
        trajectory_id = int(self.get_parameter("trajectory_id").value)
        service_name = str(self.get_parameter("trajectory_service").value)
        timeout_s = float(self.get_parameter("service_timeout_s").value)
        client = self.create_client(TrajectoryQuery, service_name)
        self.get_logger().info(
            f"Waiting for final trajectory service {service_name}"
        )
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError(
                f"Trajectory service {service_name} was not available "
                f"within {timeout_s:.1f}s"
            )

        request = TrajectoryQuery.Request()
        request.trajectory_id = trajectory_id
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            raise TimeoutError(
                f"Trajectory query did not finish within {timeout_s:.1f}s"
            )
        response = future.result()
        if not response.trajectory:
            raise RuntimeError(
                f"Trajectory {trajectory_id} has no optimized nodes: "
                f"{response.status.message}"
            )

        samples = []
        for stamped_pose in response.trajectory:
            position = stamped_pose.pose.position
            orientation = stamped_pose.pose.orientation
            samples.append(
                PoseSample(
                    stamp_ns=(
                        int(stamped_pose.header.stamp.sec) * 1_000_000_000
                        + int(stamped_pose.header.stamp.nanosec)
                    ),
                    translation=(position.x, position.y, position.z),
                    quaternion_xyzw=(
                        orientation.x,
                        orientation.y,
                        orientation.z,
                        orientation.w,
                    ),
                )
            )
        map_frame = response.trajectory[0].header.frame_id or "map"
        self.get_logger().info(
            f"Loaded {len(samples)} optimized nodes for trajectory "
            f"{trajectory_id}, frame={map_frame}, "
            f"time=[{samples[0].stamp_ns}, {samples[-1].stamp_ns}]"
        )
        return TrajectoryInterpolator(samples), trajectory_id, map_frame

    def run(self) -> dict:
        trajectory, trajectory_id, map_frame = self._query_trajectory()
        config = ExportConfig(
            bag_uri=self._required_path("bag_uri"),
            output_directory=self._required_path("output_directory"),
            jsonl_path=self._optional_path("jsonl_path"),
            color_topic=str(self.get_parameter("color_topic").value),
            depth_topic=str(self.get_parameter("depth_topic").value),
            camera_info_topic=str(
                self.get_parameter("camera_info_topic").value
            ),
            static_tf_topic=str(self.get_parameter("static_tf_topic").value),
            tracking_frame=str(self.get_parameter("tracking_frame").value),
            camera_frame=str(self.get_parameter("camera_frame").value),
            max_rgb_depth_delta_ns=int(
                float(
                    self.get_parameter(
                        "max_rgb_depth_time_diff_s"
                    ).value
                )
                * 1e9
            ),
            max_detection_delta_ns=int(
                float(
                    self.get_parameter(
                        "max_detection_time_diff_s"
                    ).value
                )
                * 1e9
            ),
            max_trajectory_interval_ns=int(
                float(
                    self.get_parameter(
                        "max_trajectory_interval_s"
                    ).value
                )
                * 1e9
            ),
            input_depth_scale=float(
                self.get_parameter("input_depth_scale").value
            ),
            start_frame=int(self.get_parameter("start_frame").value),
            max_frames=int(self.get_parameter("max_frames").value),
            jpeg_quality=int(self.get_parameter("jpeg_quality").value),
            overwrite=bool(self.get_parameter("overwrite").value),
            allow_resolution_mismatch=bool(
                self.get_parameter("allow_resolution_mismatch").value
            ),
        )
        self.get_logger().info(
            f"Exporting {config.bag_uri} to {config.output_directory}; "
            f"tracking={config.tracking_frame}, camera={config.camera_frame}"
        )
        report = export_dataset(
            config,
            trajectory,
            trajectory_id=trajectory_id,
            map_frame=map_frame,
        )
        self.get_logger().info(
            "Dataset export complete: "
            f"frames={report['exported_frames']}, "
            f"RGB-D median dt="
            f"{report['rgb_depth_delta_ms']['median']:.3f}ms, "
            f"output={report['output_directory']}"
        )
        return report


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartographerDatasetExporterNode()
    exit_code = 0
    try:
        node.run()
    except Exception as exc:
        exit_code = 1
        node.get_logger().error(f"Dataset export failed: {exc}")
        node.get_logger().debug(traceback.format_exc())
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)
