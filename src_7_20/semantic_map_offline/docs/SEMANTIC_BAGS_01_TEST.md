# semantic_bags/semantic_01 test

## Bag and registration

This workflow targets:

```text
/home/weiyu/vscode_workspace/ros2_wp/ros_bag/semantic_bags/semantic_01
```

The bag is about 336 seconds long and contains `/scan`, `/odom`, `/tf`,
`/tf_static`, RGB and depth data. Cartographer rebuilds the map from `/scan + /odom`;
recorded `/map` and dynamic `/tf` are not replayed. The bag already contains the
calibrated static robot transforms, so the package URDF remains disabled.

RGB is `640x480`, while the unregistered depth image is `640x400`. The launch file
uses `depth_image_proc/register_node` and publishes RGB-aligned depth on:

```text
/camera/depth_registered/image_raw
/camera/depth_registered/camera_info
```

The registered image was verified to use `camera_color_optical_frame`. Projection
uses the RGB camera model:

```text
fx = 365.1741638183594
fy = 365.42144775390625
cx = 318.27630615234375
cy = 243.80377197265625
depth_scale = 0.001
```

## Build

```bash
cd /home/weiyu/vscode_workspace/ros2_wp
source /opt/ros/jazzy/setup.bash
colcon build \
  --packages-ignore cartographer cartographer_ros cartographer_ros_msgs \
  --packages-select opi_yolo_rknn_recorder semantic_map_pkg semantic_map_offline \
  --symlink-install
source install/setup.bash
```

## Recommended two-pass workflow

Running detection and projection as separate bag passes preserves all detections and
prevents inference speed from disturbing timestamp synchronization.

### Pass 1: generate YOLO JSONL

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source /home/weiyu/vscode_workspace/ros2_wp/install/setup.bash

ros2 launch opi_yolo_rknn_recorder ai_recorder.launch.py \
  image_topic:=/camera/color/image_raw \
  output_dir:=/tmp/semantic_bags_01_yolo \
  detect_model:=/home/weiyu/vscode_workspace/models/yolo11n.pt \
  enable_pose:=false \
  classes_filter:=$(seq -s, 1 79) \
  enable_http:=false
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source /home/weiyu/vscode_workspace/ros2_wp/install/setup.bash

ros2 bag play \
  /home/weiyu/vscode_workspace/ros2_wp/ros_bag/semantic_bags/semantic_01 \
  --clock --topics /camera/color/image_raw
```

The result is `/tmp/semantic_bags_01_yolo/results.jsonl`. The recorder appends to an
existing JSONL, so use an empty output directory for each experiment.

### Pass 2: SLAM, registration, projection and fusion

```bash
source /opt/ros/jazzy/setup.bash
source /home/weiyu/vscode_workspace/ros2_wp/install/setup.bash

ros2 launch semantic_map_offline semantic_bags_01_test.launch.py \
  run_yolo:=false \
  jsonl_path:=/tmp/semantic_bags_01_yolo/results.jsonl \
  output_directory:=/tmp/semantic_bags_01_output \
  snapshot_path:=/tmp/semantic_bags_01_output/semantic_objects.json \
  rate:=0.5 \
  enable_rviz:=true
```

Set the RViz Fixed Frame to `map` and inspect:

- `Map`: `/map`
- `PointCloud2`: `/semantic_offline/points`
- `PointCloud2`: `/semantic_offline/fused_points`
- `Image`: `/camera/depth_registered/image_raw`

Useful runtime checks:

```bash
ros2 topic echo /camera/depth_registered/image_raw --once --field header
ros2 topic hz /camera/depth_registered/image_raw
ros2 run tf2_ros tf2_echo map camera_color_optical_frame
```

## Outputs

```text
/tmp/semantic_bags_01_output/
|-- semantic_objects.json
|-- fused_objects.npz
`-- objects/
    |-- object_0001_chair.ply
    |-- object_0001_chair.npz
    `-- ...
```

`semantic_objects.json` is the navigation-facing object map. Per-object PLY and NPZ
files are intended for geometry inspection. SLAM pose error propagates directly into
cross-frame association and fusion. If individual observations look correct but the
fused object is blurred or duplicated, inspect trajectory and association quality
before changing the camera intrinsics.

## One-pass preview

This is useful for checking topics and RViz, but the two-pass output is preferred:

```bash
ros2 launch semantic_map_offline semantic_bags_01_test.launch.py \
  run_yolo:=true \
  detect_model:=/home/weiyu/vscode_workspace/models/yolo11n.pt \
  rate:=0.5
```
