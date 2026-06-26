# Rear Camera Pose — 后置摄像头姿态检测 ROS2 包

## 1. 包概述

`rear_camera_pose` 是一个 **YOLO-pose 专用** ROS2 包，用于后置 USB 摄像头的**人体关键点检测 + 姿态分类**。

与前置摄像头包 `opi_yolo_rknn_recorder` 并行运行，互不冲突。

### 架构

```
┌─────────────────────┐     /rear_camera/image_raw     ┌──────────────────────┐
│  usb_cam_publisher  │ ──────────────────────────────→ │   rear_pose_node     │
│  (V4L2 → ROS topic) │   sensor_msgs/Image              │  (YOLO-pose + HTTP)  │
└─────────────────────┘                                  └──────────────────────┘
```

采集和处理完全解耦，和 `opi_yolo_rknn_recorder` 使用相同的 ROS 订阅模式。如需使用外部 camera driver，只需跳过 `usb_cam_publisher`，让 `rear_pose_node` 订阅外部话题即可。

### 核心功能

| 功能 | 说明 |
|---|---|
| YOLO-pose 关键点检测 | 17 点 COCO 关键点 (nose ~ ankles) |
| 姿态分类 | 站立 / 坐蹲 / 弯腰 / 疑似跌倒 / 未知 |
| 举手动作检测 | both_arms_up / left_arm_up / right_arm_up |
| 实时可视化 | 骨架绘制 + 色标 + 文字标注 |
| JSON 结果发布 | ROS2 topic |
| Debug 图像发布 | ROS2 topic（带标注的图像） |
| 本地录制 | latest.json / latest.jpg / results.jsonl / 定时帧截图 |
| HTTP 预览面板 | MJPEG 流 + JSON 展示 + 姿态状态面板 |

### 与 `opi_yolo_rknn_recorder` 的关系

`rear_camera_pose` 是从 `opi_yolo_rknn_recorder` 裁剪、适配而来的**派生包**，代码复用如下：

| 模块 | 来源 | 改动 |
|---|---|---|
| COCO 骨架常量 (`COCO_SKELETON`, `KPT_NAMES`, `KPT_INDEX`) | `ai_recorder_node.py` L20-38 | 直接复用，不变 |
| 辅助函数 (`safe_float`) | `ai_recorder_node.py` L42-50 | 直接复用，不变 |
| HTTP 预览服务器 (`PreviewHandler`) | `ai_recorder_node.py` L61-275 | 复用，端口/路径改为后摄专用 |
| 姿态提取 (`_extract_poses`) | `ai_recorder_node.py` L510-542 | 复用，去掉了 detect 相关 |
| 关键点访问 (`_get_kpt`, `_pair_midpoint`) | `ai_recorder_node.py` L544-564 | 直接复用，不变 |
| 姿态分类规则 (`_classify_posture`) | `ai_recorder_node.py` L566-715 | 直接复用，不变 |
| 姿态汇总 (`_summarize_postures`) | `ai_recorder_node.py` L717-723 | 直接复用，不变 |
| 绘制函数 (`_draw_poses`, `_draw_overlay`, `_draw_label`) | `ai_recorder_node.py` L735-815 | 复用，去掉了 `_draw_detections` |
| 输出写入 (`_write_outputs`) | `ai_recorder_node.py` L817-838 | 复用，增加内存缓冲 |
| HTTP 服务器启动 (`_start_http_server`) | `ai_recorder_node.py` L384-396 | 直接复用，不变 |

**移除的模块**（不需要的）：
- YOLO 目标检测 (`_extract_detections`, `_draw_detections`, `detect_model`)
- `classes_filter` 参数

**新增的模块**：
- `usb_cam_publisher` — V4L2 采集节点，与 YOLO 处理解耦

---

## 2. 包结构

```
rear_camera_pose/
├── package.xml
├── setup.py / setup.cfg
├── resource/rear_camera_pose
├── rear_camera_pose/
│   ├── __init__.py
│   ├── rear_pose_node.py        # YOLO-pose 处理节点
│   └── usb_cam_publisher.py     # V4L2 USB 摄像头发布节点
├── launch/
│   └── rear_pose.launch.py      # 启动文件（同时启动两个节点）
├── config/
│   └── default.yaml             # 默认参数
└── scripts/
    └── run_rear_pose.sh          # 一键运行脚本
```

---

## 3. 快速启动

### 3.1 前置条件

```bash
# Python 依赖
python3 -m pip install --user ultralytics opencv-python numpy==1.26.4

# 编译
cd ~/vscode_workspace/ros2_wp
colcon build --packages-select rear_camera_pose --symlink-install
source install/setup.bash
```

### 3.2 启动（默认：USB 摄像头 + YOLO-pose）
#查看设备名
for dev in /dev/video*; do
  echo "$dev: $(cat /sys/class/video4linux/$(basename $dev)/name)"
done


```bash
ros2 launch rear_camera_pose rear_pose.launch.py
```

这会启动两个节点：
- `usb_cam_publisher` — 从 `/dev/video2` 采集，发布到 `/rear_camera/image_raw`
- `rear_pose_node` — 订阅 `/rear_camera/image_raw`，推理 + HTTP 预览

HTTP 预览面板: `http://<板端IP>:8089/`

### 3.3 配合外部 camera driver

```bash
# 先启动外部 camera driver
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2

# 只启动 rear_pose_node，指向外部话题
ros2 run rear_camera_pose rear_pose_node --ros-args \
  -p image_topic:=/usb_cam/image_raw \
  -p pose_model:=/home/orangepi/models/yolo11n-pose.pt
```

### 3.4 常用参数覆盖

```bash
ros2 launch rear_camera_pose rear_pose.launch.py \
  conf:=0.35 \
  width:=1280 \
  height:=720 \
  http_port:=8090
```

---

## 4. 参数列表

### 4.1 usb_cam_publisher 参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `camera_device` | string | `/dev/video2` | V4L2 设备路径 |
| `image_topic` | string | `/rear_camera/image_raw` | 发布图像的话题 |
| `width` | int | `640` | 采集宽度 |
| `height` | int | `480` | 采集高度 |
| `fps` | int | `15` | 发布帧率 |

### 4.2 rear_pose_node 参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `image_topic` | string | `/rear_camera/image_raw` | 订阅的图像话题 |
| `pose_model` | string | `...models/yolo11n-pose.pt` | YOLO-pose 模型路径 |
| `imgsz` | int | `640` | 推理图像尺寸 |
| `conf` | float | `0.25` | 置信度阈值 |
| `iou` | float | `0.45` | NMS IoU 阈值 |
| `frame_skip` | int | `0` | 跳帧数（0 = 每帧推理） |
| `output_dir` | string | `~/rear_pose_results` | 结果输出目录 |
| `result_topic` | string | `/rear_pose/results_json` | JSON 结果发布话题 |
| `publish_debug_image` | bool | `true` | 是否发布标注图像 |
| `debug_image_topic` | string | `/rear_pose/debug_image` | 标注图像话题 |
| `save_every_n` | int | `30` | 每 N 帧保存一张图片 |
| `enable_http` | bool | `true` | 启用 HTTP 预览 |
| `http_port` | int | `8089` | HTTP 端口 |
| `mjpeg_fps` | int | `8` | MJPEG 流帧率 |

---

## 5. ROS2 API

### 5.1 发布的话题

| 话题 | 类型 | 节点 | 说明 |
|---|---|---|---|
| `/rear_camera/image_raw` | `sensor_msgs/Image` | usb_cam_publisher | 原始图像 |
| `/rear_pose/results_json` | `std_msgs/String` | rear_pose_node | 推理结果 JSON |
| `/rear_pose/debug_image` | `sensor_msgs/Image` | rear_pose_node | 标注后的图像 |

### 5.2 订阅的话题

| 话题 | 类型 | 节点 | 说明 |
|---|---|---|---|
| `image_topic`（可配置） | `sensor_msgs/Image` | rear_pose_node | 输入图像 |

### 5.3 JSON 结果格式

```json
{
  "wall_time": "2026-06-23T12:29:38.351",
  "image_shape": [480, 640, 3],
  "poses": [
    {
      "class_id": 0,
      "class_name": "person",
      "confidence": 0.87,
      "xyxy": [120.5, 80.3, 380.2, 460.7],
      "keypoints": [
        {"name": "nose", "x": 250.1, "y": 120.2, "confidence": 0.91}
      ],
      "posture": "standing",
      "posture_confidence": 0.75,
      "action_tags": [],
      "posture_features": {
        "bbox_aspect_w_div_h": 0.68,
        "torso_angle_from_vertical_deg": 12.5
      },
      "posture_reasons": ["bbox_tall", "body_axis_vertical"]
    }
  ],
  "posture_summary": {"standing": 1},
  "fall_suspected": false,
  "latency_ms": 5.14,
  "infer_count": 10
}
```

---

## 6. 姿态分类规则

规则引擎（单帧、非医学级判定），基于 COCO 17 关键点的几何关系：

| 姿态 | 判定条件（满足 ≥2 条） |
|---|---|
| **standing** (站立) | bbox 高>宽×1.15、身体轴近竖直(<35°)、髋-膝-踝垂直有序、肩在髋上方 |
| **sitting_or_squatting** (坐/蹲) | 髋膝垂直距 < bbox_h×0.28 或髋与膝高度接近 |
| **bending** (弯腰) | 躯干倾角 35°~60°、bbox 偏竖 |
| **lying_or_fall_suspected** (疑似跌倒) | bbox 宽>高×1.2、身体轴近水平(>60°)、躯干轴近水平(>55°)、肩髋同高 |
| **unknown** (未知) | 有效核心关键点 < 2 个 |

**举手动作**：`left_arm_up` / `right_arm_up` / `both_arms_up`

---

## 7. HTTP 预览面板

访问 `http://<IP>:8089/`：

- **左侧**: MJPEG 实时视频流（带骨架/bbox/标签叠加）
- **右侧**: 姿态状态面板 + latest.json 实时刷新
- 端点: `/` `/stream.mjpg` `/latest.jpg` `/latest.json`

---

## 8. 输出文件

```
~/rear_pose_results/
├── latest.json         # 最新一帧推理结果（原子写入）
├── latest.jpg          # 最新标注帧（85% JPEG）
├── results.jsonl       # 所有推理结果追加写入
└── frames/             # 定时截图（每 save_every_n 帧）
```

---

## 9. 与前置摄像头同时运行

两个包使用不同的话题和端口，可以并行运行：

```
终端1: ros2 launch opi_yolo_rknn_recorder ai_recorder.launch.py  # 前摄
终端2: ros2 launch rear_camera_pose rear_pose.launch.py           # 后摄
```

| 资源 | 前摄 (opi_yolo_rknn_recorder) | 后摄 (rear_camera_pose) |
|---|---|---|
| 图像输入 | Gemini 336L: `/camera/color/image_raw` | USB 摄像头 → `/rear_camera/image_raw` |
| 推理 | detect + pose | pose only |
| JSON topic | `/yolo/results_json` | `/rear_pose/results_json` |
| Debug topic | `/yolo/debug_image` | `/rear_pose/debug_image` |
| HTTP 端口 | 8088 | 8089 |

---

## 10. 迁移到 Orange Pi 板端

```bash
# 1. 复制包到板端
scp -r rear_camera_pose orangepi@<ip>:~/vscode_workspace/ros2_wp/src/

# 2. 板端编译
ssh orangepi@<ip>
cd ~/vscode_workspace/ros2_wp
colcon build --packages-select rear_camera_pose --symlink-install

# 3. 启动（使用 RKNN 模型）
ros2 launch rear_camera_pose rear_pose.launch.py \
  pose_model:=/home/orangepi/models/yolo11n-pose-rk3588.rknn

# 4. 浏览器查看
# http://<orangepi_ip>:8089/
```

---

## 11. 故障排查

| 现象 | 原因 | 解决 |
|---|---|---|
| `Cannot open V4L2 device` | 设备路径错误或被占用 | 检查 `ls /dev/video*`，杀掉占用进程 |
| `Pose model not found` | 模型路径不正确 | 设置 `pose_model:=` 指向正确的文件 |
| poses=0 一直没检测到人 | 置信度阈值太高或画面中无人 | 降低 `conf:=0.15` |
| HTTP 端口被占用 | 端口 8089 冲突 | 修改 `http_port:=8090` |
| 浏览器画面延迟 | MJPEG 缓冲 | 降低 `mjpeg_fps:=4` 或刷新页面 |
