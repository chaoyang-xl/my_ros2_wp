#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch opi_yolo_rknn_recorder ai_recorder.launch.py \
  image_topic:=/camera/color/image_raw \
  output_dir:=/home/orangepi/yolo_results \
  detect_model:=/home/orangepi/models/yolo11n.rknn \
  pose_model:=/home/orangepi/models/yolo11n-pose.rknn \
  enable_detect:=true \
  enable_pose:=true \
  imgsz:=640 \
  conf:=0.35 \
  frame_skip:=0 \
  http_port:=8088
