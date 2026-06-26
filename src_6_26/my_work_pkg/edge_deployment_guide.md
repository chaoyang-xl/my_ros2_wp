# OrangePi 5 Plus 边缘部署指南

> 硬件: OrangePi 5 Plus (RK3588, 8GB RAM)
> 相机: Orbbec Gemini 336L (深度相机)
> 软件包: `opi_yolo_rknn_recorder` + `my_work_pkg`

---

## 1. 硬件能力与瓶颈分析

### RK3588 核心指标

| 资源 | 规格 | 对本系统的影响 |
|------|------|--------------|
| CPU | 4×A76 @2.4GHz + 4×A55 @1.8GHz | A76核跑ROS回调/TF/Python；A55核可做轻量任务 |
| NPU | 6 TOPS (INT8/INT16/FP16) | RKNN格式YOLO推理的主要加速器，比CPU快5-10倍 |
| GPU | Mali-G610 | 本方案暂不使用（YOLO走NPU） |
| 内存 | 8GB LPDDR4/4X | 操作系统约1.5GB，留给推理+ROS+图像处理约5GB |
| 存储 | eMMC / NVMe | 影响模型加载速度和日志写入寿命 |

### 关键约束

1. **双模型并发**：同时跑 detect + pose 两个 YOLO 模型，NPU 负载接近饱和
2. **内存压力**：深度图 (640×480×2B≈600KB/帧) + RGB图 (1280×720×3B≈2.6MB/帧) 在 ROS 管道中多份拷贝
3. **Python GIL**：`ai_recorder_node` 推理 + HTTP 服务 + `frontend_bridge_node` 全在 Python 进程里，CPU密集段互斥
4. **热量**：长时间满载 NPU+CPU 会触发降频，需要主动散热

---

## 2. 系统级优化（部署前必做）

### 2.1 CPU 调度策略

```bash
# 设置 CPU 调度为 performance 模式，避免动态调频带来的延迟抖动
echo performance | sudo tee /sys/devices/system/cpu/cpufreq/policy*/scaling_governor

# 验证
cat /sys/devices/system/cpu/cpufreq/policy*/scaling_cur_freq
```

### 2.2 NPU 驱动确认

```bash
# 确认 RKNN 驱动已加载
ls /dev/rknpu
# 应输出: /dev/rknpu

# 查看 NPU 使用率（需安装 rknn_server）
rknpu_load=$(cat /sys/kernel/debug/rknpu/load 2>/dev/null || echo "N/A")
echo "NPU load: $rknpu_load"
```

### 2.3 内存管理

```bash
# 增加 swap (如果还没设置)
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# 降低 swappiness，优先用物理内存
sudo sysctl vm.swappiness=10
```

### 2.4 禁用不需要的服务

```bash
# 禁用桌面环境（如果是 headless 运行）
sudo systemctl set-default multi-user.target

# 禁用蓝牙（如果不用）
sudo systemctl disable bluetooth.service
```

---

## 3. 模型选择：RKNN vs PyTorch

### 优先级：必须用 RKNN 格式

| 模型格式 | 推理设备 | 预估帧率 (imgsz=416) | 推荐 |
|---------|---------|---------------------|------|
| `.rknn` (INT8量化) | NPU | 30-60 FPS | ✅ 首选 |
| `.pt` (PyTorch FP32) | CPU | 3-8 FPS | ❌ 仅作调试 |
| `.onnx` | CPU (RKNN不支持) | 5-12 FPS | ❌ 不推荐 |

### 模型准备

```bash
# 模型存放位置
/home/orangepi/models/
├── yolo11n.rknn          # 检测模型 (INT8量化)
└── yolo11n-pose.rknn     # 姿态模型 (INT8量化)

# 如果只有 .pt 文件，需要在 x86 主机上用 rknn-toolkit2 转换
# 参考: https://github.com/airockchip/rknn-toolkit2
```

> **注意**：`ai_recorder_node.py` 根据文件名是否包含 `rknn_model` 来判断推理后端。
> 如果模型路径中包含 `rknn_model` 字符串，则使用 NPU 加速；否则走 PyTorch CPU 推理。
> 确保你的 RKNN 模型文件路径名中有 `rknn_model` 字样，或修改代码判断逻辑。

---

## 4. opi_yolo_rknn_recorder 参数调优

### 4.1 核心推理参数（对性能影响最大）

| 参数 | x86默认值 | OPi5+推荐值 | 理由 |
|------|----------|------------|------|
| `imgsz` | 640 | **416** 或 **320** | RK3588 NPU在imgsz=416下detect约40FPS；640降到约20FPS。双模型叠加后更明显。416是精度-速度最佳平衡点 |
| `conf` | 0.25 | **0.35** | 提高阈值减少误检，降低下游处理负载 |
| `iou` | 0.45 | **0.5** | 稍微放宽NMS，减少密集场景的漏检 |
| `frame_skip` | 0 | **1** 或 **2** | 每N帧才推理一次。相机30fps + frame_skip=1 → 实际推理15fps，足以满足语义地图需求 |
| `enable_detect` | true | **true** | 保留，为下游提供检测框 |
| `enable_pose` | true | 按需 | 如果不需要姿态/跌倒检测，关闭可节省约40% NPU算力 |
| `save_every_n` | 30 | **0** 或 **100** | 0=不存磁盘帧，减少eMMC写入磨损。如需存档用100 |
| `publish_debug_image` | true | **false** | 关闭后不发布 `/yolo/debug_image`，节省约30%带宽（RGB图每帧2.6MB）。调试时再开 |
| `enable_http` | true | **true** | HTTP预览开销小（8FPS JPEG），保留便于远程调试 |
| `mjpeg_fps` | 8 | **4** | 降低MJPEG流帧率，减少CPU编码开销 |

### 4.2 话题与路径参数

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| `image_topic` | `/camera/color/image_raw` | Orbbec Gemini 336L 的 RGB 话题 |
| `output_dir` | `/home/orangepi/yolo_results` | 确保目录存在且有写权限 |
| `result_topic` | `/yolo/results_json` | 必须与 `frontend_bridge_node` 的 `input_topic` 一致 |
| `detect_model` | `/home/orangepi/models/yolo11n.rknn` | RKNN 模型路径 |
| `pose_model` | `/home/orangepi/models/yolo11n-pose.rknn` | RKNN 模型路径 |

### 4.3 推荐启动命令

```bash
# 生产模式 (最精简)
ros2 launch opi_yolo_rknn_recorder ai_recorder.launch.py \
  imgsz:=416 \
  conf:=0.35 \
  frame_skip:=1 \
  enable_pose:=false \
  publish_debug_image:=false \
  save_every_n:=0 \
  mjpeg_fps:=4

# 完整功能模式 (detect + pose + HTTP预览)
ros2 launch opi_yolo_rknn_recorder ai_recorder.launch.py \
  imgsz:=416 \
  conf:=0.35 \
  frame_skip:=1 \
  enable_pose:=true \
  publish_debug_image:=false \
  save_every_n:=0 \
  mjpeg_fps:=4 \
  http_port:=8088
```

---

## 5. my_work_pkg 参数调优

### 5.1 frontend_bridge_node 参数

| 参数 | x86默认值 | OPi5+推荐值 | 理由 |
|------|----------|------------|------|
| `camera_fx` | 320.0 | **按实际标定** | Gemini 336L 出厂内参，需从相机参数文件或标定获取 |
| `camera_fy` | 320.0 | **按实际标定** | 同上 |
| `camera_cx` | 320.0 | **按实际标定** | 通常是图像宽/2 |
| `camera_cy` | 240.0 | **按实际标定** | 通常是图像高/2 |
| `depth_scale` | 1.0 | **1.0** | Gemini 336L 深度图是 16UC1(mm)，但 Orbbec ROS2 驱动已转为米，需实测确认 |
| `input_topic` | `/yolo/results_json` | `/yolo/results_json` | 与 ai_recorder_node 的 result_topic 一致 |
| `depth_topic` | `/camera/depth_image` | `/camera/depth/image_raw` | Gemini 336L 的深度话题名，启动相机时确认 |
| `camera_frame` | `""` | `camera_depth_optical_frame` | 明确指定，避免 frame_id 不一致导致 TF 失败 |
| `target_frame` | `map` | `map` | Cartographer 提供 |
| `tf_timeout_s` | 0.1 | **0.2** | 边缘设备 TF 计算稍慢，适当放宽 |
| `slop` | 0.1 | **0.15** | JSON消息无header时间戳，放宽同步容差 |
| `pose_iou_threshold` | 0.3 | 0.3 | 保持不变 |
| `enable_pose` | true | 按需 | 如果 ai_recorder 关了 pose，这里也关 |

### 5.2 semantic_projection_node / loop_closure_guard_node 参数

| 参数 | x86默认值 | OPi5+推荐值 | 理由 |
|------|----------|------------|------|
| `search_radius_m` | 0.2 | **0.3** | 边缘设备深度估计精度可能稍差，适当放大搜索范围 |
| `occupied_threshold` | 80 | **65** | Gemini 336L 建的栅格地图障碍值可能偏低，降低阈值提高吸附成功率 |
| `min_island_pixels` | 2 | **3** | 边缘设备建图噪声可能更大，提高阈值过滤碎噪声 |
| `match_distance` | 1.0 | **0.8** | 室内场景物体间距通常<1m，缩小匹配距离避免错误合并 |
| `smoothing_alpha` | 0.3 | **0.25** | 边缘设备检测精度波动更大，多平滑一些 |
| `memory_timeout` | 5.0 | **15.0** | 机器人移动中可能几秒看不到物体，延长记忆时间 |
| `use_loop_closure` | false | **true** | Cartographer 必然有回环，必须用 loop_closure_guard_node |

### 5.3 回环检测专属参数 (loop_closure_guard_node)

| 参数 | 推荐值 | 理由 |
|------|--------|------|
| `lc_check_period` | 1.0 | 1Hz足够，不需要更频繁 |
| `lc_translation_threshold` | 0.05 | 保持默认，Cartographer 回环通常跳变 >5cm |
| `lc_rotation_threshold` | 0.02 | 保持默认 |
| `lc_source_frame` | `odom` | 保持默认，节点通过 `lookup_transform("map", "odom")` 监控 odom→map |
| `lc_resnap_enabled` | true | 回环后 snapped 静态对象在新 /map 上小半径重吸附 |
| `lc_resnap_radius_m` | 0.3 | re-snap 搜索半径，现场可在 0.2~0.5m 调整 |

### 5.4 可视化节点参数

| 参数 | 推荐值 | 理由 |
|------|--------|------|
| `enable_debug_viz` | **false** (生产) / true (调试) | raw_seed_visualizer 会发布大量 Marker，生产环境关闭 |
| `marker_lifetime_s` | 3.0 | 缩短，避免旧 Marker 堆积 |
| `raw_point_scale` | 0.08 | 小一点更清晰 |
| `raw_publish_hz` | 5.0 | 5Hz 足够调试 |

### 5.5 推荐启动命令

```bash
# 生产模式 (loop_closure + frontend_bridge，无 debug viz)
ros2 launch my_work_pkg semantic_map.launch.py \
  frontend:=frontend \
  use_loop_closure:=true \
  enable_debug_viz:=false \
  camera_fx:=525.0 camera_fy:=525.0 camera_cx:=640.0 camera_cy:=360.0 \
  depth_scale:=1.0 \
  input_topic:=/yolo/results_json \
  depth_topic:=/camera/depth/image_raw \
  camera_frame:=camera_depth_optical_frame \
  tf_timeout_s:=0.2 \
  slop:=0.15 \
  search_radius_m:=0.3 \
  occupied_threshold:=65 \
  min_island_pixels:=3 \
  match_distance:=0.8 \
  smoothing_alpha:=0.25 \
  memory_timeout:=15.0
```

---

## 6. 相机 (Orbbec Gemini 336L) 配置

### 6.1 启动命令

```bash
# 启动 Orbbec 驱动，开启深度配准
ros2 launch orbbec_camera gemini_330_series.launch.py \
  depth_registration:=true \
  color_width:=640 \
  color_height:=480 \
  depth_width:=640 \
  depth_height:=480 \
  color_fps:=15 \
  depth_fps:=15
```

### 6.2 分辨率选择

| 配置 | 带宽开销 | 推荐场景 |
|------|---------|---------|
| 640×480 @15fps | ~50 MB/s | ✅ **OPi5+ 推荐**，兼顾精度和带宽 |
| 640×480 @30fps | ~100 MB/s | ⚠️ 可能丢帧，frame_skip=1时没必要 |
| 1280×720 @15fps | ~140 MB/s | ❌ 带宽太高，USB3.0 可能不够 |

> **关键**：降低相机帧率到 15fps 配合 `frame_skip=1`，实际推理约 7-8 FPS，对语义地图已足够。

### 6.3 确认深度图编码

```bash
# 检查深度图的编码和 frame_id
ros2 topic echo /camera/depth/image_raw --no-arr -1

# 期望输出:
# encoding: "16UC1"  (毫米) 或 "32FC1" (米)
# frame_id: "camera_depth_optical_frame"
```

如果深度图是 `16UC1`(毫米)：`depth_scale=0.001`
如果深度图是 `32FC1`(米)：`depth_scale=1.0`

---

## 7. 完整数据流与话题对应表

```
Orbbec Gemini 336L
  ├── /camera/color/image_raw (sensor_msgs/Image, 640×480@15fps)
  ├── /camera/depth/image_raw (sensor_msgs/Image, 640×480@15fps)
  └── /camera/depth_to_color/parameter_description (TF)

ai_recorder_node
  订阅: /camera/color/image_raw
  发布: /yolo/results_json (std_msgs/String, 含检测框+姿态 JSON)

frontend_bridge_node
  订阅: /yolo/results_json + /camera/depth/image_raw
  发布: /semantic_seed (std_msgs/String, 含 gx/gy map坐标)

loop_closure_guard_node
  订阅: /semantic_seed + /map (OccupancyGrid)
  监听: TF odom→map
  发布: ~/markers (visualization_msgs/MarkerArray)
```

**话题名对应检查表**：

| 上游发布 | 下游订阅参数 | 必须一致 |
|---------|------------|---------|
| ai_recorder: `result_topic:=/yolo/results_json` | frontend_bridge: `input_topic:=/yolo/results_json` | ✅ |
| 相机: `/camera/depth/image_raw` | frontend_bridge: `depth_topic:=/camera/depth/image_raw` | ✅ |
| 相机 frame_id | frontend_bridge: `camera_frame:=camera_depth_optical_frame` | ✅ |

---

## 8. 一键启动脚本

在 OPi 上创建 `/home/orangepi/start_semantic_map.sh`：

```bash
#!/usr/bin/env bash
set -Eeuo pipefail

echo "=== [1/5] 设置 CPU 调度 ==="
echo performance | sudo tee /sys/devices/system/cpu/cpufreq/policy*/scaling_governor

echo "=== [2/5] 启动 ROS 2 环境 ==="
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "=== [3/5] 启动 Orbbec 相机 ==="
ros2 launch orbbec_camera gemini_330_series.launch.py \
  depth_registration:=true \
  color_width:=640 color_height:=480 \
  depth_width:=640 depth_height:=480 \
  color_fps:=15 depth_fps:=15 &
sleep 3

echo "=== [4/5] 启动 YOLO 推理节点 ==="
ros2 launch opi_yolo_rknn_recorder ai_recorder.launch.py \
  imgsz:=416 \
  conf:=0.35 \
  frame_skip:=1 \
  enable_pose:=false \
  publish_debug_image:=false \
  save_every_n:=0 \
  mjpeg_fps:=4 \
  detect_model:=/home/orangepi/models/yolo11n.rknn \
  pose_model:=/home/orangepi/models/yolo11n-pose.rknn &
sleep 2

echo "=== [5/5] 启动语义地图管道 ==="
ros2 launch my_work_pkg semantic_map.launch.py \
  frontend:=frontend \
  use_loop_closure:=true \
  enable_debug_viz:=false \
  camera_fx:=525.0 camera_fy:=525.0 \
  camera_cx:=320.0 camera_cy:=240.0 \
  depth_scale:=1.0 \
  input_topic:=/yolo/results_json \
  depth_topic:=/camera/depth/image_raw \
  camera_frame:=camera_depth_optical_frame \
  tf_timeout_s:=0.2 \
  slop:=0.15 \
  search_radius_m:=0.3 \
  occupied_threshold:=65 \
  min_island_pixels:=3 \
  match_distance:=0.8 \
  smoothing_alpha:=0.25 \
  memory_timeout:=15.0

echo "=== 所有节点已启动 ==="
wait
```

> **注意**：`camera_fx/fy/cx/cy` 的值需要替换为你 Gemini 336L 的实际内参。
> 可通过 `ros2 topic echo /camera/color/camera_info` 获取。

---

## 9. 性能监控与调优

### 9.1 实时监控

#### 方法一：一键监控脚本（推荐）

在 OPi 上创建 `/home/orangepi/monitor.sh`，每 2 秒刷新一次全部指标：

```bash
#!/usr/bin/env bash
# RK3588 实时监控面板 -- CPU/NPU/GPU/温度/内存
# 用法: sudo bash ~/monitor.sh   (Ctrl+C 退出)

while true; do
  clear
  echo "=========================================="
  echo "  RK3588 系统监控  $(date '+%H:%M:%S')"
  echo "=========================================="

  # --- CPU 各核频率与占用 ---
  echo "[CPU]"
  for i in 0 1 2 3 4 5 6 7; do
    freq=$(cat /sys/devices/system/cpu/cpufreq/policy${i}/scaling_cur_freq 2>/dev/null || echo 0)
    freq_mhz=$((freq / 1000))
    usage=$(grep "^cpu${i} " /proc/stat | awk '{t=$2+$3+$4+$5+$6+$7+$8; printf "%.0f",100*(t-$5)/t}')
    [ $i -lt 4 ] && ct="A55" || ct="A76"
    printf "  cpu%d(%s) %4d MHz  %3s%%\n" "$i" "$ct" "$freq_mhz" "$usage"
  done

  # --- NPU ---
  echo "------------------------------------------"
  echo "[NPU]"
  if [ -f /sys/kernel/debug/rknpu/load ]; then
    cat /sys/kernel/debug/rknpu/load
  else
    echo "  不可用 (需 sudo 或驱动未加载)"
  fi

  # --- GPU ---
  echo "------------------------------------------"
  echo "[GPU]"
  gpu_freq=$(cat /sys/class/devfreq/fb000000.gpu/cur_freq 2>/dev/null)
  [ -n "$gpu_freq" ] && echo "  freq: $((gpu_freq / 1000000)) MHz" || echo "  不可用"

  # --- 温度 ---
  echo "------------------------------------------"
  echo "[温度]"
  for tz in /sys/class/thermal/thermal_zone*/; do
    type=$(cat "${tz}type" 2>/dev/null || continue)
    temp=$(cat "${tz}temp" 2>/dev/null || echo 0)
    printf "  %-22s %s C\n" "$type" "$(echo "scale=1;$temp/1000" | bc)"
  done

  # --- 内存 ---
  echo "------------------------------------------"
  echo "[内存]"
  free -h | awk '/^Mem:/{printf "  总计:%s  已用:%s  可用:%s\n",$2,$3,$7}'
  free -h | awk '/^Swap:/{printf "  Swap 总计:%s  已用:%s\n",$2,$3}'

  echo "=========================================="
  sleep 2
done
```

运行（需要 sudo 读 NPU debug 文件）：

```bash
sudo bash ~/monitor.sh
```

#### 方法二：单独命令速查

```bash
# ---- NPU 占用 (需要 sudo) ----
sudo watch -n1 cat /sys/kernel/debug/rknpu/load
# 输出示例: NPU load: Core0: 85%, Core1: 72%, Core2: 60%
# RK3588 有 3 个 NPU 核心，每个核心 0-100%

# ---- CPU 占用 (htop 更直观) ----
sudo apt install -y htop
htop
# 按 F1 看各核心，按 F6 按 CPU% 排序

# ---- CPU 频率 ----
watch -n1 'cat /sys/devices/system/cpu/cpufreq/policy*/scaling_cur_freq'
# policy0-3 = A55 小核 (最高1.8GHz)
# policy4-7 = A76 大核 (最高2.4GHz)

# ---- CPU/GPU 温度 ----
watch -n1 'for t in /sys/class/thermal/thermal_zone*/; do
  echo "$(cat ${t}type): $(cat ${t}temp)mC"
done'
# 重点关注 soc-thermal，超过 85C 会降频

# ---- GPU 频率 ----
watch -n1 'echo "GPU: $(cat /sys/class/devfreq/fb000000.gpu/cur_freq) Hz"'

# ---- 内存 ----
watch -n2 free -h

# ---- ROS 话题帧率 (各开一个终端) ----
ros2 topic hz /camera/color/image_raw    # 期望 15 Hz
ros2 topic hz /yolo/results_json         # 期望 5-15 Hz
ros2 topic hz /semantic_seed             # 期望 3-10 Hz
```

#### 方法三：tmux 四格分屏监控

同时看 NPU + CPU + 内存 + 温度，最推荐的方式：

```bash
sudo apt install -y tmux

# 启动 4 格分屏
tmux new-session -d -s mon \
  'sudo watch -n1 cat /sys/kernel/debug/rknpu/load'
tmux split-window -h 'htop'
tmux split-window -v 'watch -n2 free -h'
tmux select-pane -t 0
tmux split-window -v 'watch -n1 "for t in /sys/class/thermal/thermal_zone*/; do echo \$(cat \${t}type): \$(cat \${t}temp)mC; done"'
tmux attach -t mon

# 退出分屏: Ctrl+B 然后按 D (detach，后台继续运行)
# 重新接入: tmux attach -t mon
```

#### 方法四：远程监控

从开发机 SSH 到板子查看：

```bash
# SSH 进去跑 NPU 监控
ssh orangepi@<opi-ip> 'sudo watch -n1 cat /sys/kernel/debug/rknpu/load'

# SSH 进去跑 htop
ssh -t orangepi@<opi-ip> htop

# 浏览器远程看 YOLO 预览画面
# 打开: http://<opi-ip>:8088/
```

#### 各项指标正常范围参考

| 指标 | 正常范围 | 异常信号 | 处理 |
|------|---------|---------|------|
| NPU Core0/1/2 | 50-95% | <10% 说明没走NPU | 检查模型路径是否为 .rknn |
| A76 大核占用 | <70% | >80% 且持续 | 关闭 debug_image 和 HTTP |
| A55 小核占用 | <50% | >80% | 正常，ROS回调可能调度到小核 |
| soc-thermal | 55-75 C | >85 C | 加风扇/散热片，降低 imgsz |
| 内存 used | <5.5 GB | >6.5 GB | 关 publish_debug_image |
| Swap used | <500 MB | >1 GB | 内存不足，降 QoS depth |

### 9.2 性能基准

| 指标 | 目标值 | 如果达不到的调节手段 |
|------|--------|-------------------|
| `/camera/color/image_raw` | 15 Hz | 降低相机分辨率/帧率 |
| `/yolo/results_json` | ≥5 Hz | 降低 imgsz(320)、提高 frame_skip(2) |
| `/semantic_seed` | ≥3 Hz | 同上 + 检查 TF 是否通畅 |
| NPU 占用 | 70-95% | 正常，说明 NPU 在干活 |
| CPU 占用 | <60% | 如果过高，关闭 debug_image 和 HTTP |
| 内存 | <6 GB | 关闭 publish_debug_image，降低 QoS depth |

### 9.3 如果帧率不够的逐步降级策略

```
阶段1 (推荐): imgsz=416, frame_skip=1, enable_pose=false
  → 预期 ~10-15 FPS 推理

阶段2: imgsz=320, frame_skip=1
  → 预期 ~20-30 FPS 推理

阶段3: imgsz=416, frame_skip=2, enable_pose=false
  → 预期 ~5-7 FPS 推理 (最低可接受)

阶段4: imgsz=320, frame_skip=2, 关闭HTTP, 关闭debug_image
  → 极限模式
```

---

## 10. 常见问题排查

### Q: ai_recorder_node 启动后推理很慢 (几秒一帧)

**原因**：使用了 `.pt` 模型而非 `.rknn` 模型，走 CPU 推理。
**解决**：
```bash
# 确认模型路径
ls -la /home/orangepi/models/
# 确认文件是 .rknn 格式
```

### Q: frontend_bridge_node 不发布 /semantic_seed

**排查步骤**：
```bash
# 1. 检查 /yolo/results_json 是否有数据
ros2 topic echo /yolo/results_json -1

# 2. 检查深度图是否有数据
ros2 topic echo /camera/depth/image_raw --no-arr -1

# 3. 检查 TF 树
ros2 run tf2_tools view_frames
# 确认 camera_depth_optical_frame → map 链存在

# 4. 查看节点日志
ros2 launch --debug my_work_pkg semantic_map.launch.py ...
```

### Q: TF 报错 "Lookup would require extrapolation into the past"

**原因**：Cartographer 还没建好图，或时间同步问题。
**解决**：增大 `tf_timeout_s:=0.5`，确保 Cartographer 已启动。

### Q: 内存持续增长最终 OOM

**解决**：
- `memory_timeout` 不要设为 `inf`，用 `15.0`
- `publish_debug_image:=false`
- 降低 QoS depth: 在代码中把图像 QoS depth 从 5 改为 2

### Q: 相机内参怎么获取

```bash
# 方法1: 从相机 camera_info 话题读取
ros2 topic echo /camera/color/camera_info -1 | grep -A5 "k:"

# K矩阵 [fx, 0, cx, 0, fy, cy, 0, 0, 1]
# 对应 camera_fx=K[0], camera_fy=K[4], camera_cx=K[2], camera_cy=K[5]

# 方法2: 查看 Orbbec 出厂参数文件
cat /path/to/orbbec_camera/config/gemini_336l.yaml
```
