#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y \
  python3-pip \
  python3-colcon-common-extensions \
  ros-humble-cv-bridge \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs \
  python3-opencv

# cv_bridge in ROS2 Humble on Ubuntu 22.04 is commonly incompatible with NumPy 2.x.
python3 -m pip install --user --force-reinstall "numpy==1.26.4" \
  -i https://mirrors.aliyun.com/pypi/simple/ \
  --timeout 300 --retries 10 --prefer-binary

python3 -m pip install --user ultralytics \
  -i https://mirrors.aliyun.com/pypi/simple/ \
  --timeout 300 --retries 10 --prefer-binary

echo "Install done. Next: source /opt/ros/humble/setup.bash && colcon build"
