#!/usr/bin/env bash
# ------------------------------------------------------------------------
# Run the rear camera pose node (local testing with USB camera).
#
# Usage:
#   ./run_rear_pose.sh                          # V4L2 direct capture
#   ./run_rear_pose.sh /dev/video0              # use a different device
#   CAMERA_TOPIC=/usb_cam/image_raw ./run_rear_pose.sh   # ROS topic mode
#
# Model: downloads yolo11n-pose.pt if not already present.
# ------------------------------------------------------------------------
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
MODEL_DIR="${HOME}/.cache/rear_camera_pose"
MODEL_PATH="${MODEL_DIR}/yolo11n-pose.pt"
DEVICE="${1:-/dev/video2}"

# ---- Download model if needed ----
if [ ! -f "${MODEL_PATH}" ]; then
  echo "==> Downloading yolo11n-pose.pt ..."
  mkdir -p "${MODEL_DIR}"
  cd "${MODEL_DIR}"
  wget -q --show-progress \
    "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-pose.pt" \
    -O "${MODEL_PATH}" || {
    echo "ERROR: failed to download model. Check network or download manually."
    exit 1
  }
  cd -
fi

# ---- Source ROS ----
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
else
  echo "WARNING: No standard ROS2 setup.bash found. Trying sourced environment."
fi

# ---- Source workspace ----
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  source "${WS_DIR}/install/setup.bash"
else
  echo "WARNING: Workspace not built. Run: colcon build --packages-select rear_camera_pose"
fi

# ---- Decide camera mode ----
if [ -n "${CAMERA_TOPIC:-}" ]; then
  echo "==> Using ROS topic mode: ${CAMERA_TOPIC}"
  ros2 launch rear_camera_pose rear_pose.launch.py \
    camera_device:='' \
    image_topic:=${CAMERA_TOPIC} \
    pose_model:=${MODEL_PATH}
else
  echo "==> Using V4L2 direct capture: ${DEVICE}"
  ros2 launch rear_camera_pose rear_pose.launch.py \
    camera_device:=${DEVICE} \
    pose_model:=${MODEL_PATH}
fi
