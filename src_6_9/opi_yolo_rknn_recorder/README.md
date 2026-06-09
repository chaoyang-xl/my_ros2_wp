# 1、启动

## 键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard

## gazebo环境:
ros2 launch fishbot_description display_rviz2.launch.py

## 检测节点
ros2 launch opi_yolo_rknn_recorder ai_recorder.launch.py \
  image_topic:=/camera/image \
  output_dir:=/home/weiyu/vscode_workspace/yolo_results_snapshot \
  detect_model:=/home/weiyu/vscode_workspace/models/yolo11n.pt \
  pose_model:=/home/weiyu/vscode_workspace/models/yolo11n-pose.pt

 http://127.0.0.1:8088 


## cartographer建图
 ros2 launch cartographer_test cartographer.launch.py 


 ## 语义建图
  ros2 run my_work_pkg semantic_projection_node 
  ros2 run my_work_pkg frontend_bridge_node 


ros2 run my_work_pkg frontend_bridge_node --ros-args -p camera_frame:=camera_optical_link


# 为什么要加camera_frame:=camera_optical_link
因为深度图 header 里的 frame_id 和 TF 树里的帧名不一致：

深度图 header 写的:  fishbot/base_link/rgbd_camera   ← 这个名字
TF 树里实际叫:       camera_link → camera_optical_link  ← 不匹配

ROS 的 tf2 是按名字严格匹配的。_transform_to_map 从 camera_frame 出发查 TF 链到 map，如果第一个名字就不存在，整条链都找不到，变换失败返回 None，种子被丢弃。

camera_frame 参数就是用来覆盖深度图 header 里的错误名字，告诉节点用 TF 树里实际存在的帧名去查。你指定 camera_optical_link 后，TF 链路就通了：


camera_optical_link → camera_link → base_link → odom → map  ✅

# 为什么真机和仿真的步骤不一样
两种方式的区别：

原版：ApproximateTimeSynchronizer 时间同步

深度图 ──┬── [比较时间戳] ── 差距 < 0.1s？ ── 是 → 触发回调
JSON   ──┘                                 ── 否 → 等待/丢弃

它要求两条消息的时间戳在 slop=0.1s 窗口内。

你改的：latest-depth 手动配对

深度图到来 → 存到 self._latest_depth_msg
JSON 到来  → 直接用最新的深度图处理


为什么真机 OK，Gazebo 不行
真机上，深度相机和前端 JSON 都挂在同一台机器上，用同一个时钟（系统墙钟），两个消息的时间戳天然对齐在几十毫秒内，ApproximateTimeSynchronizer 总能配对成功。

Gazebo 里不一样：


深度图 (ros_gz_bridge):  使用 /clock 仿真时间    ← 从 Gazebo 桥接过来
JSON (前端模拟):         可能没有 header 时间戳   ← std_msgs/String 本就没有 header

String 消息没有 header.stamp，即使开了 allow_headerless=True（用到达时间代替），到达时间也是系统墙钟。一个用仿真时间、一个用墙钟，两个时间域根本不在一个尺度上，差距远大于 0.1s，同步器永远不会触发回调。

你的 latest-depth 方案绕过了时间戳比较，所以仿真和真机都能跑。