#!/usr/bin/env bash

set -Eeuo pipefail

source_ros_setup() {
  local setup_file="$1"
  if [ -f "$setup_file" ]; then
    set +u
    source "$setup_file"
    set -u
  else
    echo "[ERROR] setup file not found: $setup_file"
    exit 1
  fi
}

echo "[INFO] Sourcing ROS 2 Humble..."
source_ros_setup /opt/ros/humble/setup.bash

echo "[INFO] Sourcing workspace..."
source_ros_setup "$HOME/ros2_ws/install/setup.bash"

IMAGE_TOPIC="${IMAGE_TOPIC:-/camera/color/image_raw}"
OUTPUT_DIR="${OUTPUT_DIR:-/home/orangepi/yolo_results}"
DETECT_MODEL="${DETECT_MODEL:-/home/orangepi/models/yolo11n.pt}"
POSE_MODEL="${POSE_MODEL:-/home/orangepi/models/yolo11n-pose.pt}"
CONF="${CONF:-0.35}"
IMGSZ="${IMGSZ:-640}"
HTTP_PORT="${HTTP_PORT:-8088}"
FRAME_SKIP="${FRAME_SKIP:-0}"
SAVE_EVERY_N="${SAVE_EVERY_N:-10}"

mkdir -p "$OUTPUT_DIR"

echo "[INFO] Starting AI recorder with PyTorch fallback"
echo "[INFO] image_topic  = $IMAGE_TOPIC"
echo "[INFO] output_dir   = $OUTPUT_DIR"
echo "[INFO] detect_model = $DETECT_MODEL"
echo "[INFO] pose_model   = $POSE_MODEL"
echo "[INFO] conf         = $CONF"
echo "[INFO] imgsz        = $IMGSZ"
echo "[INFO] http_port    = $HTTP_PORT"

ros2 launch opi_yolo_rknn_recorder ai_recorder.launch.py \
  image_topic:="$IMAGE_TOPIC" \
  output_dir:="$OUTPUT_DIR" \
  detect_model:="$DETECT_MODEL" \
  pose_model:="$POSE_MODEL" \
  enable_detect:=true \
  enable_pose:=true \
  imgsz:="$IMGSZ" \
  conf:="$CONF" \
  http_port:="$HTTP_PORT" \
  frame_skip:="$FRAME_SKIP" \
  save_every_n:="$SAVE_EVERY_N"
