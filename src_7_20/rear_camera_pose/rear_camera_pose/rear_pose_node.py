#!/usr/bin/env python3
"""
Rear camera YOLO-pose node with posture classification and HTTP preview.

Supports two camera input modes:
  1. V4L2 direct capture (set `camera_device` to e.g. `/dev/video2`)
  2. ROS image topic subscription (set `image_topic`)

Pose-only — no object detection.  Posture classification, skeleton drawing,
JSON result publishing, local recording, and an HTTP preview dashboard are
included (reused from opi_yolo_rknn_recorder).
"""

import json
import os
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# Constants (COCO 17-keypoint skeleton)
# ---------------------------------------------------------------------------

COCO_SKELETON = [
    (5, 7), (7, 9),
    (6, 8), (8, 10),
    (5, 6),
    (5, 11), (6, 12),
    (11, 12),
    (11, 13), (13, 15),
    (12, 14), (14, 16),
    (0, 1), (0, 2),
    (1, 3), (2, 4),
]

KPT_NAMES = [
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle",
]

KPT_INDEX = {name: i for i, name in enumerate(KPT_NAMES)}


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def safe_float(x, default=0.0):
    """Cast to float, guarding against None / NaN."""
    try:
        if x is None:
            return default
        if np.isnan(x):
            return default
        return float(x)
    except Exception:
        return default


# ---------------------------------------------------------------------------
# HTTP preview server (reused & adapted from opi_yolo_rknn_recorder)
# ---------------------------------------------------------------------------

class PreviewHandler(BaseHTTPRequestHandler):
    output_dir = Path("~/rear_pose_results").expanduser()
    mjpeg_fps = 8
    # Shared buffer refs — set by RearPoseNode._start_http_server
    jpg_lock: Optional[threading.Lock] = None
    jpg_data_ref = None  # reference to RearPoseNode for _jpg_bytes

    def log_message(self, fmt, *args):
        return  # suppress access logs

    @classmethod
    def _read_latest_jpg(cls) -> Optional[bytes]:
        """Read latest JPEG from shared memory buffer (fast) or disk (fallback)."""
        if cls.jpg_lock and cls.jpg_data_ref is not None:
            with cls.jpg_lock:
                return cls.jpg_data_ref._jpg_bytes
        # Fallback: disk read
        p = cls.output_dir / "latest.jpg"
        if p.exists():
            return p.read_bytes()
        return None

    def _send_bytes(self, data: bytes, content_type: str):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):  # noqa: N802
        if self.path in ["/", "/index.html"]:
            html = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Rear Camera - Pose Monitor</title>
  <style>
    body { margin: 0; background: #0b0f14; color: #e6edf3; font-family: Arial, sans-serif; }
    header { padding: 14px 20px; background: #151b23; border-bottom: 1px solid #30363d; }
    main { display: grid; grid-template-columns: minmax(320px, 1fr) 430px; gap: 16px; padding: 16px; }
    .card { background: #111820; border: 1px solid #30363d; border-radius: 12px; padding: 12px; }
    img { width: 100%; max-height: 82vh; object-fit: contain; background: #000; border-radius: 8px; }
    pre { white-space: pre-wrap; word-break: break-word; max-height: 58vh; overflow: auto;
          background: #05080c; padding: 10px; border-radius: 8px; font-size: 12px; }
    a { color: #58a6ff; }
    .small { color: #8b949e; font-size: 13px; }
    .status { padding: 10px; border-radius: 8px; background: #05080c; margin-bottom: 10px;
              line-height: 1.7; font-size: 14px; }
    .alert { color: #ff6b6b; font-weight: bold; }
    .ok { color: #7ee787; font-weight: bold; }
  </style>
</head>
<body>
  <header>
    <h2>Rear Camera - YOLO-pose Posture Monitor</h2>
    <div class="small">
      MJPEG: <a href="/stream.mjpg">/stream.mjpg</a> |
      Snapshot: <a href="/latest.jpg">/latest.jpg</a> |
      JSON: <a href="/latest.json">/latest.json</a>
    </div>
  </header>
  <main>
    <section class="card">
      <h3>Real-time View</h3>
      <img src="/stream.mjpg" alt="Rear camera live stream">
    </section>
    <section class="card">
      <h3>Posture Status</h3>
      <div id="status" class="status">loading...</div>
      <h3>latest.json</h3>
      <pre id="json">loading...</pre>
    </section>
  </main>
  <script>
    function countPostures(data) {
      const counts = {};
      const poses = data.poses || [];
      for (const p of poses) {
        const k = p.posture || 'unknown';
        counts[k] = (counts[k] || 0) + 1;
      }
      return counts;
    }
    function renderStatus(data) {
      const poses = data.poses || [];
      const counts = countPostures(data);
      const fall = poses.some(p => p.posture === 'lying_or_fall_suspected');
      const arms = poses
        .map((p, i) => {
          const tags = p.action_tags || [];
          return tags.length ? `person ${i}: ${tags.join(', ')}` : '';
        })
        .filter(Boolean);
      let html = '';
      html += fall
        ? '<div class="alert">WARNING: lying / fall suspected</div>'
        : '<div class="ok">Status: no fall suspected</div>';
      html += `<div>People: ${poses.length}</div>`;
      html += `<div>Posture counts: ${JSON.stringify(counts)}</div>`;
      html += `<div>Latency: ${data.latency_ms ?? '-'} ms</div>`;
      html += `<div>Infer count: ${data.infer_count ?? '-'}</div>`;
      if (arms.length) {
        html += `<div>Actions: ${arms.join(' | ')}</div>`;
      }
      return html;
    }
    async function updateJson() {
      try {
        const r = await fetch('/latest.json?ts=' + Date.now());
        const data = await r.json();
        document.getElementById('json').textContent = JSON.stringify(data, null, 2);
        document.getElementById('status').innerHTML = renderStatus(data);
      } catch (e) {
        document.getElementById('json').textContent = String(e);
        document.getElementById('status').textContent = String(e);
      }
    }
    updateJson();
    setInterval(updateJson, 1000);
  </script>
</body>
</html>
"""
            self._send_bytes(html.encode("utf-8"), "text/html; charset=utf-8")
            return

        if self.path.startswith("/latest.json"):
            p = self.output_dir / "latest.json"
            if not p.exists():
                self._send_bytes(b'{"status":"waiting for first inference"}', "application/json")
                return
            self._send_bytes(p.read_bytes(), "application/json")
            return

        if self.path.startswith("/latest.jpg") or self.path.startswith("/snapshot.jpg"):
            jpg = self._read_latest_jpg()
            if jpg is None:
                img = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(img, "waiting for first inference...", (30, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                ok, buf = cv2.imencode(".jpg", img)
                jpg = buf.tobytes() if ok else b""
            self._send_bytes(jpg, "image/jpeg")
            return

        if self.path.startswith("/stream.mjpg"):
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            delay = 1.0 / max(1, int(self.mjpeg_fps))
            while True:
                try:
                    jpg = self._read_latest_jpg()
                    if jpg is None:
                        img = np.zeros((480, 640, 3), dtype=np.uint8)
                        cv2.putText(img, "waiting for first inference...", (30, 240),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                        ok, buf = cv2.imencode(".jpg", img)
                        jpg = buf.tobytes() if ok else b""
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                    time.sleep(delay)
                except Exception:
                    break
            return

        self.send_response(404)
        self.end_headers()
        self.wfile.write(b"404 not found")


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class RearPoseNode(Node):
    """YOLO-pose-only node for the rear USB camera."""

    def __init__(self):
        super().__init__("rear_pose_node")

        # --- Image input ---
        self.declare_parameter("image_topic", "/rear_camera/image_raw")

        # --- Model parameters ---
        self.declare_parameter("pose_model", "/home/orangepi/models/yolo11n-pose.pt")
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("frame_skip", 0)

        # --- Output parameters ---
        self.declare_parameter("output_dir", "~/rear_pose_results")
        self.declare_parameter("result_topic", "/rear_pose/results_json")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "/rear_pose/debug_image")
        self.declare_parameter("save_every_n", 30)

        # --- HTTP parameters ---
        self.declare_parameter("enable_http", True)
        self.declare_parameter("http_host", "0.0.0.0")
        self.declare_parameter("http_port", 8089)
        self.declare_parameter("mjpeg_fps", 8)

        # --- Resolve parameters ---
        self.image_topic = str(self.get_parameter("image_topic").value)

        self.pose_model_path = str(self.get_parameter("pose_model").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        self.frame_skip = int(self.get_parameter("frame_skip").value)

        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / "frames").mkdir(parents=True, exist_ok=True)

        self.result_topic = str(self.get_parameter("result_topic").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.save_every_n = int(self.get_parameter("save_every_n").value)

        # --- State ---
        self.bridge = CvBridge()
        self.pose_model = None
        self.frame_count = 0
        self.infer_count = 0

        # Shared buffer: HTTP serves latest JPEG from RAM (avoids SD card I/O)
        self._jpg_lock = threading.Lock()
        self._jpg_bytes: Optional[bytes] = None

        # --- Load model ---
        self._load_pose_model()

        # --- Publishers ---
        self.result_pub = self.create_publisher(String, self.result_topic, 10)
        self.debug_pub = (
            self.create_publisher(Image, self.debug_image_topic, 10)
            if self.publish_debug_image
            else None
        )

        # --- Subscribe to image topic ---
        self.sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 5
        )
        self.get_logger().info(f"Subscribing to image topic: {self.image_topic}")

        # --- HTTP preview ---
        if bool(self.get_parameter("enable_http").value):
            self._start_http_server()

        # --- Log startup info ---
        self.get_logger().info("=" * 60)
        self.get_logger().info("Rear Camera Pose Node started")
        self.get_logger().info(f"  Image topic:  {self.image_topic}")
        self.get_logger().info(f"  Pose model:   {self.pose_model_path}")
        if self.pose_model is None:
            self.get_logger().warn("  !!! POSE MODEL NOT LOADED — inference is DISABLED !!!")
        else:
            self.get_logger().info("  Pose model:   LOADED")
        self.get_logger().info(f"  Output dir:   {self.output_dir}")
        self.get_logger().info(f"  Result topic: {self.result_topic}")
        self.get_logger().info(f"  HTTP port:    {self.get_parameter('http_port').value}")
        self.get_logger().info("=" * 60)

    # ------------------------------------------------------------------
    # Model loading
    # ------------------------------------------------------------------

    def _load_pose_model(self):
        try:
            from ultralytics import YOLO
        except Exception as exc:
            raise RuntimeError(
                "Failed to import ultralytics. Install: "
                "python3 -m pip install --user ultralytics numpy==1.26.4"
            ) from exc

        model_path = self.pose_model_path

        # If it's a relative name like "yolo11n-pose.pt" (no path separator),
        # ultralytics will auto-download it — skip the existence check.
        if os.path.sep not in model_path and not os.path.exists(model_path):
            self.get_logger().info(
                f"Model '{model_path}' not found locally — ultralytics will auto-download it."
            )
        elif not os.path.exists(model_path):
            self.get_logger().warn(
                f"Pose model not found: {model_path}. "
                f"Download it from https://github.com/ultralytics/assets/releases "
                f"or place it in the workspace."
            )
            return

        self.get_logger().info(f"Loading pose model: {model_path}")
        try:
            task = "pose" if "rknn_model" in model_path else None
            self.pose_model = YOLO(model_path, task=task)
            self.get_logger().info("Pose model loaded successfully")
        except Exception as exc:
            self.get_logger().error(
                f"Failed to load pose model: {repr(exc)}. "
                f"Pose inference will be DISABLED."
            )
            self.pose_model = None

    # ------------------------------------------------------------------
    # Camera direct capture
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Image callback
    # ------------------------------------------------------------------

    def image_callback(self, msg: Image):
        self.frame_count += 1
        if self.frame_skip > 0 and (self.frame_count % (self.frame_skip + 1)) != 1:
            return

        # Decode
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            frame = self.bridge.imgmsg_to_cv2(msg)
            if len(frame.shape) == 3 and frame.shape[2] == 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        start = time.time()

        result_record: Dict[str, Any] = {
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
            "wall_time": datetime.now().isoformat(timespec="milliseconds"),
            "image_topic": self.image_topic,
            "image_shape": list(frame.shape),
            "pose_model": self.pose_model_path,
            "poses": [],
            "posture_summary": {},
            "fall_suspected": False,
            "latency_ms": None,
            "infer_count": self.infer_count,
        }

        annotated = frame.copy()

        # --- Pose inference ---
        if self.pose_model is not None:
            try:
                pose_results = self.pose_model.predict(
                    source=frame,
                    imgsz=self.imgsz,
                    conf=self.conf,
                    iou=self.iou,
                    verbose=False,
                )
                if pose_results:
                    poses = self._extract_poses(pose_results[0])
                    result_record["poses"] = poses
                    result_record["posture_summary"] = self._summarize_postures(poses)
                    result_record["fall_suspected"] = (
                        result_record["posture_summary"].get(
                            "lying_or_fall_suspected", 0
                        )
                        > 0
                    )
                    self._draw_poses(annotated, poses)
            except Exception as exc:
                self.get_logger().error(f"Pose inference failed: {repr(exc)}")
                result_record["error"] = repr(exc)

        latency = time.time() - start
        result_record["latency_ms"] = round(latency * 1000.0, 2)
        self.infer_count += 1
        result_record["infer_count"] = self.infer_count

        # --- Post-processing ---
        self._draw_overlay(annotated, result_record)
        self._write_outputs(result_record, annotated)

        # --- Publish JSON result ---
        out_msg = String()
        out_msg.data = json.dumps(result_record, ensure_ascii=False)
        self.result_pub.publish(out_msg)

        # --- Publish debug image ---
        if self.debug_pub is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to publish debug image: {repr(exc)}"
                )

        # --- Periodic log ---
        if self.infer_count % 10 == 0:
            if self.pose_model is None:
                self.get_logger().warn(
                    f"infer_count={self.infer_count}, "
                    f"NO MODEL LOADED — skipping inference, "
                    f"latency={result_record['latency_ms']} ms"
                )
            else:
                self.get_logger().info(
                    f"infer_count={self.infer_count}, "
                    f"latency={result_record['latency_ms']} ms, "
                    f"poses={len(result_record['poses'])}, "
                    f"posture_summary={result_record['posture_summary']}"
                )

    # ------------------------------------------------------------------
    # Pose extraction
    # ------------------------------------------------------------------

    def _extract_poses(self, result) -> List[Dict[str, Any]]:
        poses: List[Dict[str, Any]] = []
        if result.boxes is None or result.keypoints is None:
            return poses

        names = getattr(result, "names", {}) or {}
        boxes = result.boxes
        xyxy = boxes.xyxy.cpu().numpy() if boxes.xyxy is not None else np.empty((0, 4))
        confs = (
            boxes.conf.cpu().numpy()
            if boxes.conf is not None
            else np.zeros((len(xyxy),))
        )
        clss = (
            boxes.cls.cpu().numpy().astype(int)
            if boxes.cls is not None
            else np.zeros((len(xyxy),), dtype=int)
        )
        kpts = result.keypoints.data.cpu().numpy()

        for i in range(len(kpts)):
            cls_id = int(clss[i]) if i < len(clss) else 0
            pose_obj = {
                "class_id": cls_id,
                "class_name": str(names.get(cls_id, cls_id)),
                "confidence": float(confs[i]) if i < len(confs) else 0.0,
                "xyxy": [float(x) for x in xyxy[i].tolist()] if i < len(xyxy) else [],
                "keypoints": [
                    {
                        "name": KPT_NAMES[j] if j < len(KPT_NAMES) else str(j),
                        "x": float(p[0]),
                        "y": float(p[1]),
                        "confidence": float(p[2]),
                    }
                    for j, p in enumerate(kpts[i])
                ],
            }
            posture_info = self._classify_posture(pose_obj)
            pose_obj.update(posture_info)
            poses.append(pose_obj)
        return poses

    # ------------------------------------------------------------------
    # Keypoint helpers
    # ------------------------------------------------------------------

    def _get_kpt(self, pose: Dict[str, Any], name: str, min_conf: float = 0.25):
        idx = KPT_INDEX[name]
        kpts = pose.get("keypoints", [])
        if idx >= len(kpts):
            return None
        k = kpts[idx]
        c = safe_float(k.get("confidence", 0.0))
        if c < min_conf:
            return None
        return (safe_float(k.get("x")), safe_float(k.get("y")), c)

    def _pair_midpoint(
        self, pose: Dict[str, Any], left: str, right: str, min_conf: float = 0.25
    ):
        a = self._get_kpt(pose, left, min_conf)
        b = self._get_kpt(pose, right, min_conf)
        if a and b:
            return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)
        if a:
            return a
        if b:
            return b
        return None

    # ------------------------------------------------------------------
    # Posture classification (rule-based, single-frame)
    # ------------------------------------------------------------------

    def _classify_posture(self, pose: Dict[str, Any]) -> Dict[str, Any]:
        """Rule-based single-frame posture classification.

        Image coords: x → right, y → down.
        """
        xyxy = pose.get("xyxy", [])
        if len(xyxy) == 4:
            x1, y1, x2, y2 = [safe_float(v) for v in xyxy]
            bbox_w = max(1.0, x2 - x1)
            bbox_h = max(1.0, y2 - y1)
        else:
            bbox_w, bbox_h = 1.0, 1.0

        bbox_aspect = bbox_w / bbox_h

        shoulder_mid = self._pair_midpoint(pose, "left_shoulder", "right_shoulder")
        hip_mid = self._pair_midpoint(pose, "left_hip", "right_hip")
        knee_mid = self._pair_midpoint(pose, "left_knee", "right_knee")
        ankle_mid = self._pair_midpoint(pose, "left_ankle", "right_ankle")
        nose = self._get_kpt(pose, "nose", 0.20)

        valid_core = [
            p for p in [shoulder_mid, hip_mid, knee_mid, ankle_mid] if p is not None
        ]
        valid_core_count = len(valid_core)

        features = {
            "bbox_aspect_w_div_h": round(float(bbox_aspect), 3),
            "valid_core_keypoints": valid_core_count,
            "torso_angle_from_vertical_deg": None,
            "body_angle_from_vertical_deg": None,
            "hip_knee_vertical_ratio": None,
            "knee_ankle_vertical_ratio": None,
            "head_to_hip_vertical_ratio": None,
        }

        # --- Arm-up action tags ---
        action_tags: List[str] = []
        left_wrist = self._get_kpt(pose, "left_wrist", 0.25)
        right_wrist = self._get_kpt(pose, "right_wrist", 0.25)
        left_shoulder = self._get_kpt(pose, "left_shoulder", 0.25)
        right_shoulder = self._get_kpt(pose, "right_shoulder", 0.25)

        left_arm_up = bool(
            left_wrist
            and left_shoulder
            and left_wrist[1] < left_shoulder[1] - 0.08 * bbox_h
        )
        right_arm_up = bool(
            right_wrist
            and right_shoulder
            and right_wrist[1] < right_shoulder[1] - 0.08 * bbox_h
        )

        if left_arm_up and right_arm_up:
            action_tags.append("both_arms_up")
        elif left_arm_up:
            action_tags.append("left_arm_up")
        elif right_arm_up:
            action_tags.append("right_arm_up")

        # --- Angles ---
        torso_angle = None
        if shoulder_mid and hip_mid:
            dx = abs(shoulder_mid[0] - hip_mid[0])
            dy = abs(shoulder_mid[1] - hip_mid[1])
            torso_angle = float(np.degrees(np.arctan2(dx, max(dy, 1e-6))))
            features["torso_angle_from_vertical_deg"] = round(torso_angle, 2)

        body_angle = None
        upper = shoulder_mid or nose
        lower = ankle_mid or hip_mid
        if upper and lower:
            dx = abs(upper[0] - lower[0])
            dy = abs(upper[1] - lower[1])
            body_angle = float(np.degrees(np.arctan2(dx, max(dy, 1e-6))))
            features["body_angle_from_vertical_deg"] = round(body_angle, 2)

        if hip_mid and knee_mid:
            features["hip_knee_vertical_ratio"] = round(
                abs(knee_mid[1] - hip_mid[1]) / bbox_h, 3
            )
        if knee_mid and ankle_mid:
            features["knee_ankle_vertical_ratio"] = round(
                abs(ankle_mid[1] - knee_mid[1]) / bbox_h, 3
            )
        if nose and hip_mid:
            features["head_to_hip_vertical_ratio"] = round(
                abs(hip_mid[1] - nose[1]) / bbox_h, 3
            )

        posture = "unknown"
        confidence = 0.25
        reasons: List[str] = []

        # 1) lying / fall suspected
        lying_score = 0
        if bbox_aspect > 1.20:
            lying_score += 1
            reasons.append("bbox_width_greater_than_height")
        if body_angle is not None and body_angle > 60:
            lying_score += 1
            reasons.append("body_axis_near_horizontal")
        if torso_angle is not None and torso_angle > 55:
            lying_score += 1
            reasons.append("torso_axis_near_horizontal")
        if (
            shoulder_mid
            and hip_mid
            and abs(shoulder_mid[1] - hip_mid[1]) < 0.20 * bbox_h
        ):
            lying_score += 1
            reasons.append("shoulder_and_hip_similar_height")

        if lying_score >= 2:
            posture = "lying_or_fall_suspected"
            confidence = min(0.95, 0.55 + 0.12 * lying_score)

        # 2) sitting / squatting
        elif hip_mid and knee_mid and ankle_mid:
            hip_knee_ratio = abs(knee_mid[1] - hip_mid[1]) / bbox_h
            knee_ankle_ratio = abs(ankle_mid[1] - knee_mid[1]) / bbox_h
            if hip_knee_ratio < 0.28 and knee_ankle_ratio > 0.12:
                posture = "sitting_or_squatting"
                confidence = 0.72
                reasons.append("hip_close_to_knee")
            elif hip_mid[1] > knee_mid[1] - 0.04 * bbox_h:
                posture = "sitting_or_squatting"
                confidence = 0.68
                reasons.append("hip_and_knee_similar_vertical_level")

        # 3) bending
        if posture == "unknown":
            if (
                torso_angle is not None
                and 35 <= torso_angle <= 60
                and bbox_aspect <= 1.20
            ):
                posture = "bending"
                confidence = 0.62
                reasons.append("torso_tilted")

        # 4) standing
        if posture == "unknown":
            standing_score = 0
            if bbox_h > bbox_w * 1.15:
                standing_score += 1
                reasons.append("bbox_tall")
            if body_angle is not None and body_angle < 35:
                standing_score += 1
                reasons.append("body_axis_vertical")
            if (
                hip_mid
                and knee_mid
                and ankle_mid
                and hip_mid[1] < knee_mid[1] < ankle_mid[1]
            ):
                standing_score += 1
                reasons.append("hip_knee_ankle_vertical_order")
            if shoulder_mid and hip_mid and shoulder_mid[1] < hip_mid[1]:
                standing_score += 1
                reasons.append("shoulder_above_hip")
            if standing_score >= 2:
                posture = "standing"
                confidence = min(0.90, 0.55 + 0.10 * standing_score)

        if valid_core_count < 2:
            posture = "unknown"
            confidence = 0.20
            reasons.append("not_enough_core_keypoints")

        return {
            "posture": posture,
            "posture_confidence": round(float(confidence), 3),
            "action_tags": action_tags,
            "posture_features": features,
            "posture_reasons": reasons,
        }

    @staticmethod
    def _summarize_postures(poses: List[Dict[str, Any]]) -> Dict[str, int]:
        summary: Dict[str, int] = {}
        for p in poses:
            k = p.get("posture", "unknown")
            summary[k] = summary.get(k, 0) + 1
        return summary

    # ------------------------------------------------------------------
    # Drawing
    # ------------------------------------------------------------------

    def _draw_poses(self, img: np.ndarray, poses: List[Dict[str, Any]]):
        for idx, pose in enumerate(poses):
            posture = pose.get("posture", "unknown")
            posture_conf = pose.get("posture_confidence", 0.0)
            action_tags = pose.get("action_tags", [])

            if posture == "lying_or_fall_suspected":
                color = (0, 0, 255)          # red
            elif posture == "sitting_or_squatting":
                color = (0, 165, 255)        # orange
            elif posture == "standing":
                color = (0, 255, 80)          # green
            elif posture == "bending":
                color = (255, 200, 0)         # cyan/yellow
            else:
                color = (200, 200, 200)       # grey

            # Bbox
            if pose.get("xyxy"):
                x1, y1, x2, y2 = [int(v) for v in pose["xyxy"]]
                thickness = 3 if posture == "lying_or_fall_suspected" else 2
                cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)

                label1 = f"P{idx} {posture} {posture_conf:.2f}"
                self._draw_label(img, label1, x1, max(18, y1 - 24), color)

                if action_tags:
                    label2 = ",".join(action_tags)
                    self._draw_label(img, label2, x1, max(42, y1 + 18), (180, 220, 255))

            # Skeleton
            kpts = pose.get("keypoints", [])
            pts = []
            for k in kpts:
                x = float(k["x"])
                y = float(k["y"])
                c = float(k["confidence"])
                pts.append((x, y, c))

            for a, b in COCO_SKELETON:
                if a < len(pts) and b < len(pts):
                    xa, ya, ca = pts[a]
                    xb, yb, cb = pts[b]
                    if ca > 0.25 and cb > 0.25:
                        cv2.line(
                            img,
                            (int(xa), int(ya)),
                            (int(xb), int(yb)),
                            (255, 80, 80),
                            2,
                        )

            for x, y, c in pts:
                if c > 0.25:
                    cv2.circle(img, (int(x), int(y)), 4, (80, 255, 80), -1)

    def _draw_overlay(self, img: np.ndarray, record: Dict[str, Any]):
        h, _ = img.shape[:2]
        posture_summary = record.get("posture_summary", {})
        fall_suspected = bool(record.get("fall_suspected", False))

        lines = [
            f"Poses: {len(record['poses'])}  posture={posture_summary}",
            f"latency={record['latency_ms']} ms  infer={record['infer_count']}",
            datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        ]

        x, y = 12, 24
        for line in lines:
            cv2.putText(
                img, line, (x + 1, y + 1),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 3,
            )
            cv2.putText(
                img, line, (x, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2,
            )
            y += 26

        if fall_suspected:
            alert = "WARNING: lying / fall suspected"
            cv2.rectangle(
                img, (8, h - 58), (min(650, img.shape[1] - 8), h - 10),
                (0, 0, 255), -1,
            )
            cv2.putText(
                img, alert, (18, h - 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.95, (255, 255, 255), 3,
            )

    @staticmethod
    def _draw_label(img: np.ndarray, text: str, x: int, y: int, color):
        y = max(18, y)
        x = max(0, x)
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.55
        thickness = 2
        (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
        cv2.rectangle(
            img,
            (x, y - th - baseline - 4),
            (min(img.shape[1] - 1, x + tw + 6), y + 4),
            color,
            -1,
        )
        cv2.putText(
            img, text, (x + 3, y - baseline), font, scale, (0, 0, 0), thickness,
        )

    # ------------------------------------------------------------------
    # Output writing
    # ------------------------------------------------------------------

    def _write_outputs(self, record: Dict[str, Any], annotated: np.ndarray):
        latest_json = self.output_dir / "latest.json"
        latest_jpg = self.output_dir / "latest.jpg"
        results_jsonl = self.output_dir / "results.jsonl"

        tmp_json = self.output_dir / "latest.json.tmp"
        tmp_jpg = self.output_dir / "latest.jpg.tmp.jpg"

        with open(tmp_json, "w", encoding="utf-8") as f:
            json.dump(record, f, ensure_ascii=False, indent=2)
        os.replace(tmp_json, latest_json)

        # Encode JPEG once, store to memory + disk
        ok, buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if ok:
            jpg_bytes = buf.tobytes()
            with self._jpg_lock:
                self._jpg_bytes = jpg_bytes
            with open(tmp_jpg, "wb") as f:
                f.write(jpg_bytes)
            os.replace(tmp_jpg, latest_jpg)
        else:
            cv2.imwrite(str(tmp_jpg), annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            os.replace(tmp_jpg, latest_jpg)

        with open(results_jsonl, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")

        if self.save_every_n > 0 and self.infer_count % self.save_every_n == 0:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            frame_name = f"{ts}_infer{self.infer_count:06d}.jpg"
            cv2.imwrite(str(self.output_dir / "frames" / frame_name), annotated)

    # ------------------------------------------------------------------
    # HTTP server
    # ------------------------------------------------------------------

    def _start_http_server(self):
        host = str(self.get_parameter("http_host").value)
        port = int(self.get_parameter("http_port").value)
        PreviewHandler.output_dir = self.output_dir
        PreviewHandler.mjpeg_fps = int(self.get_parameter("mjpeg_fps").value)
        # Share in-memory JPEG buffer so HTTP serves from RAM, not SD card
        PreviewHandler.jpg_lock = self._jpg_lock
        PreviewHandler.jpg_data_ref = self  # allows reading _jpg_bytes directly

        def serve():
            httpd = ThreadingHTTPServer((host, port), PreviewHandler)
            self.get_logger().info(f"HTTP preview server: http://{host}:{port}/")
            httpd.serve_forever()

        thread = threading.Thread(target=serve, daemon=True, name="http-preview")
        thread.start()

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self):
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RearPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
