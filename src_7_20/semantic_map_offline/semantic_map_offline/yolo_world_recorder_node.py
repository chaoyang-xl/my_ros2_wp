"""ROS 2 YOLO-World recorder compatible with opi_yolo_rknn_recorder output."""

from datetime import datetime
import json
import os
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .world_detection import detections_from_result, load_class_prompts, load_yolo_world


class YoloWorldRecorderNode(Node):
    """Run open-vocabulary detection and publish/save the established JSON schema."""

    def __init__(self) -> None:
        super().__init__("yolo_world_recorder_node")
        self.declare_parameters(namespace="", parameters=[
            ("image_topic", "/camera/color/image_raw"),
            ("result_topic", "/yolo/results_json"),
            ("debug_image_topic", "/yolo/debug_image"),
            ("output_dir", "/tmp/yolo_world_results"),
            ("detect_model", ""),
            ("clip_model", ""),
            ("classes_path", ""),
            ("excluded_labels", "person,wall,floor,ceiling,unknown"),
            ("imgsz", 640),
            ("conf", 0.25),
            ("iou", 0.45),
            ("frame_skip", 0),
            ("save_annotated_images", True),
            ("save_every_n", 30),
            ("image_jpeg_quality", 85),
            ("publish_debug_image", True),
        ])
        self.declare_parameter(
            "device",
            "0",
            ParameterDescriptor(dynamic_typing=True),
        )
        value = lambda name: self.get_parameter(name).value
        self.image_topic = str(value("image_topic"))
        self.result_topic = str(value("result_topic"))
        self.debug_image_topic = str(value("debug_image_topic"))
        self.output_dir = Path(str(value("output_dir"))).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.model_path = str(value("detect_model"))
        self.clip_model_path = str(value("clip_model"))
        self.classes_path = str(value("classes_path"))
        excluded = str(value("excluded_labels")).split(",")
        self.prompts = load_class_prompts(self.classes_path, excluded)
        self.device = str(value("device")).strip() or None
        self.imgsz = int(value("imgsz"))
        self.conf = float(value("conf"))
        self.iou = float(value("iou"))
        self.frame_skip = int(value("frame_skip"))
        self.save_annotated_images = bool(value("save_annotated_images"))
        self.save_every_n = int(value("save_every_n"))
        self.image_jpeg_quality = max(1, min(100, int(value("image_jpeg_quality"))))
        self.publish_debug_image = bool(value("publish_debug_image"))
        if self.save_annotated_images and self.save_every_n > 0:
            (self.output_dir / "frames").mkdir(parents=True, exist_ok=True)

        self.get_logger().info(f"Loading YOLO-World with {len(self.prompts)} prompts: {self.model_path}")
        self.model = load_yolo_world(self.model_path, self.prompts, self.clip_model_path)
        self.bridge = CvBridge()
        self.frame_count = 0
        self.infer_count = 0
        self.result_publisher = self.create_publisher(String, self.result_topic, 10)
        self.debug_publisher = self.create_publisher(Image, self.debug_image_topic, 10) if self.publish_debug_image else None
        # Offline rosbag playback publishes RELIABLE images. Matching that QoS
        # avoids losing the beginning of a bag while DDS endpoints are discovered.
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, 5
        )
        self.get_logger().info(f"Listening on {self.image_topic}; output: {self.output_dir}")

    def image_callback(self, message: Image) -> None:
        self.frame_count += 1
        if self.frame_skip > 0 and self.frame_count % (self.frame_skip + 1) != 1:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc!r}")
            return

        started = time.perf_counter()
        record = {
            "stamp_sec": int(message.header.stamp.sec),
            "stamp_nanosec": int(message.header.stamp.nanosec),
            "wall_time": datetime.now().isoformat(timespec="milliseconds"),
            "image_topic": self.image_topic,
            "image_shape": list(frame.shape),
            "detect_model": self.model_path,
            "pose_model": None,
            "detections": [],
            "poses": [],
            "posture_summary": {},
            "fall_suspected": False,
            "latency_ms": None,
            "infer_count": self.infer_count,
        }
        annotated = frame.copy()
        try:
            results = self.model.predict(source=frame, imgsz=self.imgsz, conf=self.conf, iou=self.iou, device=self.device, verbose=False)
            if results:
                record["detections"] = detections_from_result(results[0])
                if self.publish_debug_image or self.save_annotated_images:
                    annotated = results[0].plot()
        except Exception as exc:
            record["error"] = repr(exc)
            self.get_logger().error(f"YOLO-World inference failed: {exc!r}")

        self.infer_count += 1
        record["infer_count"] = self.infer_count
        record["latency_ms"] = round((time.perf_counter() - started) * 1000.0, 2)
        self._write_outputs(record, annotated)
        output = String()
        output.data = json.dumps(record, ensure_ascii=False)
        self.result_publisher.publish(output)
        if self.debug_publisher is not None:
            debug = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            debug.header = message.header
            self.debug_publisher.publish(debug)
        if self.infer_count % 10 == 0:
            self.get_logger().info(f"infer_count={self.infer_count}, latency={record['latency_ms']} ms, detections={len(record['detections'])}")

    def _write_outputs(self, record: dict, annotated) -> None:
        json_tmp = self.output_dir / "latest.json.tmp"
        with json_tmp.open("w", encoding="utf-8") as stream:
            json.dump(record, stream, ensure_ascii=False, indent=2)
        os.replace(json_tmp, self.output_dir / "latest.json")
        with (self.output_dir / "results.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        if self.save_annotated_images:
            image_tmp = self.output_dir / "latest.jpg.tmp.jpg"
            options = [int(cv2.IMWRITE_JPEG_QUALITY), self.image_jpeg_quality]
            cv2.imwrite(str(image_tmp), annotated, options)
            os.replace(image_tmp, self.output_dir / "latest.jpg")
            if self.save_every_n > 0 and self.infer_count % self.save_every_n == 0:
                stamp = f"{record['stamp_sec']}_{record['stamp_nanosec']:09d}"
                filename = f"stamp_{stamp}_infer_{self.infer_count:06d}.jpg"
                cv2.imwrite(str(self.output_dir / "frames" / filename), annotated, options)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloWorldRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
