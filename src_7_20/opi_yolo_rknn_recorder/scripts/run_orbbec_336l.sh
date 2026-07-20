#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
ros2 launch orbbec_camera gemini_330_series.launch.py depth_registration:=true
