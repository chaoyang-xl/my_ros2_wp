# Semantic Navigation Package

面向 ROS 2 Jazzy 的独立语义导航包。项目将保存的语义对象 JSON 转换为可到达的
Nav2 目标，并提供一套可独立运行的 Fishbot Gazebo Sim、AMCL 和 Nav2 测试环境。

## 功能

- 读取 `my_work_pkg` 生成的 `semantic_objects.json`。
- 只加载 `confirmed` 静态对象，默认过滤 `person`。
- 根据对象尺寸和 OccupancyGrid 在对象周围计算安全接近点。
- 通过 Nav2 `NavigateToPose` action 执行语义导航。
- 发布静态语义对象和 RViz Marker。
- 内置 Fishbot URDF、Gazebo world、TurtleBot3 house 资源和 Nav2 参数。
- 支持一键启动，也支持 Gazebo、Nav2、语义导航分离启动。

## 系统结构

```text
semantic_objects.json
        |
        v
semantic_map_loader_node --------> /semantic_map/objects
        |                           /semantic_map_loader_node/markers
        v
semantic_goal_resolver_node <----- /map + map -> base_link
        |
        v
Nav2 /navigate_to_pose
```

完整仿真链路：

```text
Gazebo Sim -> /scan + /odom + TF + /clock
                         |
saved map -> map_server -> AMCL -> Nav2
semantic JSON --------------------> semantic navigation
```

## 环境要求

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic / Gazebo Sim 8
- Nav2
- `ros_gz_sim`、`ros_gz_bridge`

依赖可通过 rosdep 安装：

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 目录结构

```text
semantic_navigation_pkg/
├── config/nav2_params.yaml
├── launch/
│   ├── fishbot_gazebo.launch.py
│   ├── nav2_localization.launch.py
│   ├── semantic_navigation.launch.py
│   └── gazebo_nav2_semantic.launch.py
├── models/turtlebot3_house/
├── semantic_navigation_pkg/
│   ├── semantic_map.py
│   ├── semantic_map_loader_node.py
│   └── semantic_goal_resolver_node.py
├── test/
├── urdf/fishbot_base.urdf
└── worlds/semantic_house.world
```

## 输入文件

### SLAM 地图

需要标准 map server 文件：

```text
my_map.yaml
my_map.pgm
```

YAML 中的 `image` 相对路径必须能找到对应图像。

### 语义对象 JSON

示例：

```json
{
  "frame_id": "map",
  "count": 1,
  "objects": [
    {
      "id": "chair_0",
      "label": "chair",
      "state": "confirmed",
      "source": "snapped",
      "x": 1.2,
      "y": 2.4,
      "size_x": 0.5,
      "size_y": 0.5,
      "confidence": 0.91
    }
  ]
}
```

SLAM 地图、Gazebo world 和语义 JSON 必须描述同一个环境，并使用一致的 `map`
坐标系。否则激光、地图和语义目标无法同时对齐。

## 构建

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select semantic_navigation_pkg
source install/setup.bash
```

## 一键启动仿真

启动 Gazebo、Fishbot、map server、AMCL、Nav2、RViz 和语义导航：

```bash
ros2 launch semantic_navigation_pkg gazebo_nav2_semantic.launch.py \
  map:=/absolute/path/to/my_map.yaml \
  semantic_map:=/absolute/path/to/semantic_objects.json
```

无界面运行：

```bash
ros2 launch semantic_navigation_pkg gazebo_nav2_semantic.launch.py \
  map:=/absolute/path/to/my_map.yaml \
  semantic_map:=/absolute/path/to/semantic_objects.json \
  headless:=true \
  enable_rviz:=false
```

常用启动参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `world` | 包内 `semantic_house.world` | Gazebo world 路径 |
| `spawn_x`, `spawn_y` | `-2.0`, `1.0` | Gazebo world 出生位置 |
| `spawn_z` | `0.08` | Gazebo world 出生高度 |
| `spawn_yaw` | `0.0` | 出生朝向，弧度 |
| `headless` | `false` | 是否关闭 Gazebo GUI |
| `enable_rviz` | `true` | 是否启动 RViz |
| `rviz_config` | 包内 `semantic_navigation.rviz` | RViz 配置文件路径 |
| `standoff_m` | `0.7` | 目标与语义对象边缘的接近距离 |
| `robot_clearance_m` | `0.20` | 接近点周围最小自由半径 |

## 分离启动

只启动 Gazebo 和 Fishbot：

```bash
ros2 launch semantic_navigation_pkg fishbot_gazebo.launch.py
```

只启动 map server、AMCL、Nav2 和 RViz：

```bash
ros2 launch semantic_navigation_pkg nav2_localization.launch.py \
  map:=/absolute/path/to/my_map.yaml
```

默认 RViz 配置位于 `config/semantic_navigation.rviz`，包含地图、激光、机器人模型、AMCL 粒子云、Nav2 路径与代价地图，以及语义对象 Marker。需要使用自定义配置时：

```bash
ros2 launch semantic_navigation_pkg nav2_localization.launch.py \
  map:=/absolute/path/to/my_map.yaml \
  rviz_config:=/absolute/path/to/custom.rviz
```

只启动语义地图加载和目标解析：

```bash
ros2 launch semantic_navigation_pkg semantic_navigation.launch.py \
  semantic_map_path:=/absolute/path/to/semantic_objects.json \
  use_sim_time:=true
```

## 坐标系说明

Gazebo world 坐标、里程计坐标和 SLAM 地图坐标不是同一个概念：

- Fishbot 默认生成在 Gazebo world `(-2, 1, 0.08)`。
- 差速插件的 `odom -> base_footprint` 从 `(0,0,0)` 开始积分。
- AMCL 默认把机器人初始 map 位姿设为建图起点 `(0,0,0)`。
- TF 主链为 `map -> odom -> base_footprint -> base_link`。

如果激光与地图没有重合，应在 RViz 使用 **2D Pose Estimate** 设置实际初始位姿，
而不是修改 Gazebo world 出生坐标来补偿 map 坐标误差。

## 发起语义导航

查看已加载对象：

```bash
ros2 topic echo /semantic_map/objects --once
```

按对象 ID 导航：

```bash
ros2 topic pub --once /semantic_navigation/goal \
  std_msgs/msg/String "{data: 'chair_0'}"
```

覆盖单次任务参数：

```bash
ros2 topic pub --once /semantic_navigation/goal std_msgs/msg/String \
  "{data: '{\"object_id\":\"chair_0\",\"standoff_m\":0.8,\"clearance_m\":0.25}'}"
```

查看状态：

```bash
ros2 topic echo /semantic_navigation/status
```

状态包括 `sending`、`accepted`、`finished`、`rejected` 和 `error`。

## ROS 接口

| 名称 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/semantic_map/objects` | `std_msgs/String` | 发布 | 过滤后的静态语义地图 JSON |
| `/semantic_navigation/goal` | `std_msgs/String` | 订阅 | 对象 ID 或导航命令 JSON |
| `/semantic_navigation/status` | `std_msgs/String` | 发布 | 语义导航状态 |
| `/semantic_map_loader_node/markers` | `MarkerArray` | 发布 | 静态对象 RViz Marker |
| `/map` | `OccupancyGrid` | 订阅 | 接近点碰撞检查地图 |
| `/navigate_to_pose` | Nav2 action | 客户端 | 最终导航目标 |

## 接近点算法

解析器不会把家具中心直接设为目标。算法会：

1. 根据 `size_x/size_y` 估算对象占用半径。
2. 在对象周围按角度采样候选位姿。
3. 使用 `/map` 排除占用、未知和净空不足的位置。
4. 选择距离机器人当前位置最近的候选点。
5. 设置目标朝向语义对象，并交给 Nav2 做全局路径规划。

候选点通过静态地图检查不代表路径一定连通，最终结果以 Nav2 规划器为准。

## 测试

```bash
cd ~/ros2_ws
colcon test --packages-select semantic_navigation_pkg
colcon test-result --verbose
```

纯算法测试：

```bash
python3 -m pytest src/semantic_navigation_pkg/test/test_semantic_map.py
```

## 常见问题

### Gazebo 中找不到 Fishbot

确认日志出现 `Entity creation successful`，然后在 Gazebo Entity Tree 中选择
`fishbot` 并聚焦。启动前应关闭旧 Gazebo，确保只有一个 `/clock` 发布者。

### 激光与地图错位

- 确认地图与 world 来自同一场景。
- 确认 TF 链完整。
- 在 RViz 使用 `2D Pose Estimate` 重设 AMCL 位姿。

### 没有可用接近点

检查对象坐标是否位于地图内，并适当增大 `standoff_m`。`clearance_m` 不应小于
机器人实际外接半径。

### Nav2 接受目标后规划失败

目标点通过了静态 OccupancyGrid 检查，但仍可能被 Nav2 inflation layer、动态障碍
或不可连通区域拒绝。应结合 global costmap 和 planner 日志判断。

## 许可证

项目代码采用 Apache-2.0。内置 TurtleBot3 house 场景资源来自 ROBOTIS
`turtlebot3_simulations`，详情见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

