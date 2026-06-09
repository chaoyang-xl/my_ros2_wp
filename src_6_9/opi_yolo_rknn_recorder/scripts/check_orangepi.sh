#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash

echo "== Network =="
ip addr | grep -E "10\.42|inet " || true
ip route || true

echo "== Camera topics =="
ros2 topic list | grep -E "camera|image|yolo" || true

echo "== Camera Hz =="
timeout 8 ros2 topic hz /camera/color/image_raw || true
