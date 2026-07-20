# semantic_01 rosbag test

## Verified bag properties

Bag directory:

```text
/home/weiyu/vscode_workspace/ros2_wp/ros_bag/semantic_01
```

- sqlite3, 3.5 GiB, 243.82 seconds, 12674 messages.
- RGB: `/camera/color/image_raw`, `640x480`, frame `camera_color_optical_frame`.
- Depth: `/camera/depth/image_raw`, `640x480`, frame `camera_color_optical_frame`.
- The depth image is already registered to RGB and uses the RGB camera model.
- LaserScan: `/scan`, frame `laser`, 2925 messages.
- `/tf_static` contains `base_link -> laser`, `base_link -> camera_link` and the camera optical frames.
- The bag has no usable `/odom`, dynamic `/tf`, or `/map` messages.

Camera parameters read from `/camera/depth/camera_info`:

```text
fx = 365.1741638183594
fy = 365.42144775390625
cx = 318.27630615234375
cy = 243.80377197265625
depth_scale = 0.001
```

Use `offline_ros_bag_no_odom.lua`. Do not publish the package URDF while replaying
this bag because the bag already contains calibrated robot static TF.

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

## Recommended two-pass test

Offline processing is deterministic when YOLO inference and map projection are run
as separate passes. The JSON timestamps come directly from the RGB image header.

### Pass 1: OPI YOLO

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source /home/weiyu/vscode_workspace/ros2_wp/install/setup.bash

ros2 launch opi_yolo_rknn_recorder ai_recorder.launch.py \
  image_topic:=/camera/color/image_raw \
  output_dir:=/tmp/semantic_01_yolo \
  detect_model:=/home/weiyu/vscode_workspace/models/yolo11n.pt \
  enable_pose:=false \
  classes_filter:=$(seq -s, 1 79) \
  enable_http:=true
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source /home/weiyu/vscode_workspace/ros2_wp/install/setup.bash

ros2 bag play /home/weiyu/vscode_workspace/ros2_wp/ros_bag/semantic_01 \
  --clock \
  --topics /camera/color/image_raw
```

Detection records are written to `/tmp/semantic_01_yolo/results.jsonl`. Use a new
output directory for another run because the recorder appends to an existing JSONL.

### Pass 2: SLAM, projection and fusion

```bash
source /opt/ros/jazzy/setup.bash
source /home/weiyu/vscode_workspace/ros2_wp/install/setup.bash

ros2 launch semantic_map_offline semantic_01_bag_test.launch.py \
  run_yolo:=false \
  jsonl_path:=/tmp/semantic_01_yolo/results.jsonl \
  snapshot_path:=/tmp/semantic_01_output/semantic_objects.json \
  output_directory:=/tmp/semantic_01_output \
  rate:=0.5 \
  enable_rviz:=true
```

The launch starts pure-LaserScan Cartographer, bag playback, bbox projection and
object fusion. It intentionally does not use SAM or pose detection. Depth projection
is delayed by five frames so Cartographer TF can catch up; each cloud still uses the
original depth timestamp for the map transform.

## RViz and outputs

Use `map` as Fixed Frame. Add these displays:

- `Map`: `/map`
- `PointCloud2`: `/semantic_offline/points`
- `PointCloud2`: `/semantic_offline/fused_points`
- `Image`: `/yolo/debug_image` only during the YOLO pass

Useful checks:

```bash
ros2 topic hz /map
ros2 topic hz /semantic_offline/points
ros2 topic echo /semantic_offline/objects --once
ros2 run tf2_ros tf2_echo map camera_color_optical_frame
```

The navigation snapshot is refreshed at `/tmp/semantic_01_output/semantic_objects.json`. Per-object PLY and NPZ files are stored in its `objects/` directory, and all fused arrays are also written to `fused_objects.npz`.

## One-pass preview

The launch can also run OPI YOLO directly:

```bash
ros2 launch semantic_map_offline semantic_01_bag_test.launch.py \
  run_yolo:=true \
  detect_model:=/home/weiyu/vscode_workspace/models/yolo11n.pt
```

This is useful for quick visualization. For final offline results, use the two-pass
workflow so every depth frame is evaluated after its timestamped map TF exists.

## Replica-style object files

The ROS fusion node refreshes the same per-object artifacts used by the Replica
evaluation workflow:

```text
/tmp/semantic_01_output/
├── semantic_objects.json
├── fused_objects.npz
└── objects/
    ├── object_0001_chair.ply
    ├── object_0001_chair.npz
    └── ...
```

`semantic_objects.json` contains navigation fields, 3D bounds, observation counts
and the relative PLY/NPZ path for every object. `fused_objects.npz` contains all
object arrays together, while `objects/` is intended for inspecting or sharing one
object at a time.

Poor SLAM does not invalidate camera-frame single-frame projection, but it directly
affects map-frame association and fusion. Pose drift produces duplicated objects,
incorrect merges and blurred or repeated geometry in the saved object clouds.
Evaluate SLAM trajectory quality before treating the fused files as navigation data.
