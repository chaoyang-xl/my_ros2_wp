#!/usr/bin/env python3
"""语义地图投影系统 — 统一启动文件

管道:  前端检测 → 深度反投影 → TF变换 → 栅格吸附 → 记忆追踪 → RViz可视化

用法:
  # 默认: frontend_bridge + semantic_projection + raw_seed_visualizer
  ros2 launch my_work_pkg semantic_map.launch.py

  # 使用回环修正节点替代 semantic_projection_node
  ros2 launch my_work_pkg semantic_map.launch.py use_loop_closure:=true

  # 关闭调试可视化
  ros2 launch my_work_pkg semantic_map.launch.py enable_debug_viz:=false

  # 覆盖单个参数
  ros2 launch my_work_pkg semantic_map.launch.py search_radius_m:=0.8 camera_fx:=609.8

  # 使用标准 ROS Detection2DArray 接入 (替代 frontend_bridge)
  ros2 launch my_work_pkg semantic_map.launch.py frontend:=yolo
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # ======================================================================
    #  全局开关
    # ======================================================================
    arg_frontend = DeclareLaunchArgument(
        "frontend", default_value="frontend",
        description="前端接入方式: 'frontend'(自定义JSON) 或 'yolo'(标准Detection2DArray)",
    )
    arg_use_loop_closure = DeclareLaunchArgument(
        "use_loop_closure", default_value="false",
        description="是否使用带回环修正的节点 (loop_closure_guard_node) 替代 semantic_projection_node",
    )
    arg_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="是否使用 rosbag 或仿真的 /clock",
    )
    arg_enable_debug_viz = DeclareLaunchArgument(
        "enable_debug_viz", default_value="true",
        description="是否启动 raw_seed_visualizer_node (调试用原始种子可视化)",
    )
    arg_enable_seed_tracker = DeclareLaunchArgument(
        "enable_seed_tracker", default_value="false",
        description="是否启动 seed_tracker_node (轻量级种子追踪可视化，无栅格吸附)",
    )
    arg_enable_diag = DeclareLaunchArgument(
        "enable_diag", default_value="true",
        description="是否启动 semantic_diag_node 诊断节点",
    )

    # ======================================================================
    #  相机内参 & 深度图  (frontend_bridge_node / yolo_seed_projection_node 共用)
    # ======================================================================
    arg_camera_fx = DeclareLaunchArgument("camera_fx", default_value="320.0", description="相机焦距 X (像素)")
    arg_camera_fy = DeclareLaunchArgument("camera_fy", default_value="320.0", description="相机焦距 Y (像素)")
    arg_camera_cx = DeclareLaunchArgument("camera_cx", default_value="320.0", description="光心 X (像素), 通常=图像宽/2")
    arg_camera_cy = DeclareLaunchArgument("camera_cy", default_value="240.0", description="光心 Y (像素), 通常=图像高/2")
    arg_depth_scale = DeclareLaunchArgument("depth_scale", default_value="1.0", description="深度值→米 系数: 16UC1(mm)=0.001, 32FC1(m)=1.0")
    arg_target_frame = DeclareLaunchArgument("target_frame", default_value="map", description="TF 目标坐标系")
    arg_tf_timeout_s = DeclareLaunchArgument("tf_timeout_s", default_value="0.1", description="TF 查询超时 (秒)")

    # ======================================================================
    #  frontend_bridge_node 专属参数
    # ======================================================================
    arg_input_topic = DeclareLaunchArgument("input_topic", default_value="/yolo/results_json", description="前端 JSON 检测话题")
    arg_depth_topic = DeclareLaunchArgument("depth_topic", default_value="/camera/depth_image", description="深度图话题")
    arg_camera_frame = DeclareLaunchArgument("camera_frame", default_value="", description="相机坐标系名 (空=使用 depth_msg.header.frame_id)")
    arg_slop = DeclareLaunchArgument("slop", default_value="0.1", description="时间同步容差 (秒)")
    arg_pose_iou_threshold = DeclareLaunchArgument("pose_iou_threshold", default_value="0.3", description="检测框与姿态框 IoU 匹配阈值")
    arg_enable_pose = DeclareLaunchArgument("enable_pose", default_value="true", description="是否输出姿态/动作信息")

    # ======================================================================
    #  yolo_seed_projection_node 专属参数 (仅 frontend:=yolo 时生效)
    # ======================================================================
    arg_yolo_det_topic = DeclareLaunchArgument(
        "yolo_det_topic", default_value="/yolo/detections",
        description="标准 Detection2DArray 话题 (仅 frontend:=yolo)",
    )
    arg_yolo_depth_topic = DeclareLaunchArgument(
        "yolo_depth_topic", default_value="/camera/depth/image_rect_raw",
        description="深度图话题 (仅 frontend:=yolo)",
    )

    # ======================================================================
    #  栅格吸附参数  (semantic_projection_node / loop_closure_guard_node 共用)
    # ======================================================================
    arg_search_radius_m = DeclareLaunchArgument("search_radius_m", default_value="0.2", description="种子周围搜索占用栅格半径 (米)")
    arg_occupied_threshold = DeclareLaunchArgument("occupied_threshold", default_value="80", description="栅格值>=此值视为占用")
    arg_min_island_pixels = DeclareLaunchArgument("min_island_pixels", default_value="2", description="占用岛屿最小像素数")
    arg_max_island_size_m = DeclareLaunchArgument("max_island_size_m", default_value="2.0", description="占用岛屿最大外接尺寸 (米), 超过则视为墙/大结构")
    arg_max_total_pixels = DeclareLaunchArgument("max_total_pixels", default_value="0", description="占用岛屿最大像素数, 0=关闭")

    # ======================================================================
    #  记忆追踪参数  (semantic_projection_node / loop_closure_guard_node / seed_tracker_node 共用)
    # ======================================================================
    arg_match_distance = DeclareLaunchArgument("match_distance", default_value="1.0", description="记忆匹配欧氏距离阈值 (米)")
    arg_smoothing_alpha = DeclareLaunchArgument("smoothing_alpha", default_value="0.3", description="EMA 平滑系数 (0~1, 越小越平滑)")
    arg_memory_timeout = DeclareLaunchArgument("memory_timeout", default_value="5.0", description="对象遗忘时间 (秒), inf=永不过期")
    arg_min_confirmed_seen = DeclareLaunchArgument("min_confirmed_seen", default_value="50", description="对象至少观测几次才进入 confirmed")
    arg_show_candidates = DeclareLaunchArgument("show_candidates", default_value="false", description="是否在 RViz 显示 candidate 对象")
    arg_label_policy_json = DeclareLaunchArgument("label_policy_json", default_value="", description="按类别覆盖策略的 JSON 字符串")
    arg_semantic_objects_topic = DeclareLaunchArgument("semantic_objects_topic", default_value="/semantic_objects", description="语义对象 JSON 发布话题")
    arg_semantic_objects_path = DeclareLaunchArgument("semantic_objects_path", default_value="/tmp/semantic_objects.json", description="语义对象 JSON 快照文件路径, 空字符串=不写文件")

    # ======================================================================
    #  loop_closure_guard_node 专属参数
    # ======================================================================
    arg_lc_check_period = DeclareLaunchArgument("lc_check_period", default_value="1.0", description="TF 监控频率 (秒)")
    arg_lc_translation_threshold = DeclareLaunchArgument("lc_translation_threshold", default_value="0.05", description="回环位移跳变阈值 (米)")
    arg_lc_rotation_threshold = DeclareLaunchArgument("lc_rotation_threshold", default_value="0.02", description="回环旋转跳变阈值 (弧度) ≈1.1°")
    arg_lc_source_frame = DeclareLaunchArgument("lc_source_frame", default_value="odom", description="回环监控参考帧 (通过 lookup_transform('map', 此帧) 监控此帧→map)")
    arg_lc_resnap_enabled = DeclareLaunchArgument("lc_resnap_enabled", default_value="true", description="回环后是否对 snapped 对象在新 /map 上重新小半径吸附")
    arg_lc_resnap_radius_m = DeclareLaunchArgument("lc_resnap_radius_m", default_value="0.3", description="回环后 snapped 对象 re-snap 搜索半径 (米)")
    arg_lc_resnap_max_attempts = DeclareLaunchArgument("lc_resnap_max_attempts", default_value="4", description="跨新地图帧的最大 re-snap 尝试次数")
    arg_lc_resnap_radius_step_m = DeclareLaunchArgument("lc_resnap_radius_step_m", default_value="0.15", description="每次失败后的搜索半径增量 (米)")
    arg_lc_resnap_max_radius_m = DeclareLaunchArgument("lc_resnap_max_radius_m", default_value="0.6", description="re-snap 最大搜索半径 (米)")
    arg_lc_stable_checks = DeclareLaunchArgument("lc_stable_checks", default_value="2", description="re-snap 前 TF 连续稳定检查次数")
    arg_lc_stable_translation_threshold = DeclareLaunchArgument("lc_stable_translation_threshold", default_value="0.02", description="TF 稳定检查最大平移变化 (米)")
    arg_lc_stable_rotation_threshold = DeclareLaunchArgument("lc_stable_rotation_threshold", default_value="0.01", description="TF 稳定检查最大旋转变化 (弧度)")

    # ======================================================================
    #  raw_seed_visualizer_node 参数
    # ======================================================================
    arg_marker_lifetime_s = DeclareLaunchArgument("marker_lifetime_s", default_value="5.0", description="原始种子 Marker 存活时间 (秒)")
    arg_raw_point_scale = DeclareLaunchArgument("raw_point_scale", default_value="0.10", description="原始种子点尺寸 (米)")
    arg_raw_publish_hz = DeclareLaunchArgument("raw_publish_hz", default_value="10.0", description="原始种子发布频率 (Hz)")

    # ======================================================================
    #  seed_tracker_node 专属参数
    # ======================================================================
    arg_tracker_point_scale = DeclareLaunchArgument("tracker_point_scale", default_value="0.12", description="追踪器点尺寸 (米)")
    arg_tracker_publish_hz = DeclareLaunchArgument("tracker_publish_hz", default_value="10.0", description="追踪器发布频率 (Hz)")
    arg_min_times_seen = DeclareLaunchArgument("min_times_seen", default_value="1", description="最少观测几次才显示")

    # ======================================================================
    #  公共参数引用 (减少重复的 LaunchConfiguration)
    # ======================================================================
    lc_camera_fx = LaunchConfiguration("camera_fx")
    lc_camera_fy = LaunchConfiguration("camera_fy")
    lc_camera_cx = LaunchConfiguration("camera_cx")
    lc_camera_cy = LaunchConfiguration("camera_cy")
    lc_depth_scale = LaunchConfiguration("depth_scale")
    lc_target_frame = LaunchConfiguration("target_frame")
    lc_tf_timeout_s = LaunchConfiguration("tf_timeout_s")
    lc_use_loop_closure = LaunchConfiguration("use_loop_closure")
    lc_enable_debug_viz = LaunchConfiguration("enable_debug_viz")
    lc_enable_seed_tracker = LaunchConfiguration("enable_seed_tracker")
    lc_enable_diag = LaunchConfiguration("enable_diag")
    lc_frontend = LaunchConfiguration("frontend")

    # ======================================================================
    #  节点定义
    # ======================================================================

    # --- 前端桥接节点 (frontend_bridge_node) ---
    frontend_bridge = Node(
        package="my_work_pkg",
        executable="frontend_bridge_node",
        name="frontend_bridge_node",
        output="screen",
        parameters=[{
            # 相机内参
            "camera_fx": lc_camera_fx,
            "camera_fy": lc_camera_fy,
            "camera_cx": lc_camera_cx,
            "camera_cy": lc_camera_cy,
            # 深度图
            "depth_scale": lc_depth_scale,
            # TF
            "target_frame": lc_target_frame,
            "tf_timeout_s": lc_tf_timeout_s,
            # 话题
            "input_topic": LaunchConfiguration("input_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_frame": LaunchConfiguration("camera_frame"),
            # 时间同步
            "slop": LaunchConfiguration("slop"),
            # 姿态匹配
            "pose_iou_threshold": LaunchConfiguration("pose_iou_threshold"),
            "enable_pose": LaunchConfiguration("enable_pose"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        condition=IfCondition(
            PythonExpression(["'", lc_frontend, "' == 'frontend'"])
        ),
    )

    # --- 标准 YOLO 接入节点 (yolo_seed_projection_node) ---
    yolo_seed_proj = Node(
        package="my_work_pkg",
        executable="yolo_seed_projection_node",
        name="yolo_seed_projection_node",
        output="screen",
        parameters=[{
            "camera_fx": lc_camera_fx,
            "camera_fy": lc_camera_fy,
            "camera_cx": lc_camera_cx,
            "camera_cy": lc_camera_cy,
            "depth_scale": lc_depth_scale,
            "target_frame": lc_target_frame,
            "tf_timeout_s": lc_tf_timeout_s,
        }],
        condition=IfCondition(
            PythonExpression(["'", lc_frontend, "' == 'yolo'"])
        ),
        remappings=[
            ("/yolo/detections", LaunchConfiguration("yolo_det_topic")),
            ("/camera/depth/image_rect_raw", LaunchConfiguration("yolo_depth_topic")),
        ],
    )

    # --- 语义吸附 + 追踪节点 (semantic_projection_node) ---
    semantic_proj = Node(
        package="my_work_pkg",
        executable="semantic_projection_node",
        name="semantic_projection_node",
        output="screen",
        parameters=[{
            # 栅格吸附
            "search_radius_m": LaunchConfiguration("search_radius_m"),
            "occupied_threshold": LaunchConfiguration("occupied_threshold"),
            "min_island_pixels": LaunchConfiguration("min_island_pixels"),
            "max_island_size_m": LaunchConfiguration("max_island_size_m"),
            "max_total_pixels": LaunchConfiguration("max_total_pixels"),
            # 记忆追踪
            "match_distance": LaunchConfiguration("match_distance"),
            "smoothing_alpha": LaunchConfiguration("smoothing_alpha"),
            "memory_timeout": LaunchConfiguration("memory_timeout"),
            "min_confirmed_seen": LaunchConfiguration("min_confirmed_seen"),
            "show_candidates": LaunchConfiguration("show_candidates"),
            "label_policy_json": LaunchConfiguration("label_policy_json"),
            "semantic_objects_topic": LaunchConfiguration("semantic_objects_topic"),
            "semantic_objects_path": LaunchConfiguration("semantic_objects_path"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        condition=UnlessCondition(lc_use_loop_closure),
    )

    # --- 回环修正节点 (loop_closure_guard_node, 替代 semantic_projection_node) ---
    loop_closure = Node(
        package="my_work_pkg",
        executable="loop_closure_guard_node",
        name="loop_closure_guard_node",
        output="screen",
        parameters=[{
            # 栅格吸附
            "search_radius_m": LaunchConfiguration("search_radius_m"),
            "occupied_threshold": LaunchConfiguration("occupied_threshold"),
            "min_island_pixels": LaunchConfiguration("min_island_pixels"),
            "max_island_size_m": LaunchConfiguration("max_island_size_m"),
            "max_total_pixels": LaunchConfiguration("max_total_pixels"),
            # 记忆追踪
            "match_distance": LaunchConfiguration("match_distance"),
            "smoothing_alpha": LaunchConfiguration("smoothing_alpha"),
            "memory_timeout": LaunchConfiguration("memory_timeout"),
            "min_confirmed_seen": LaunchConfiguration("min_confirmed_seen"),
            "show_candidates": LaunchConfiguration("show_candidates"),
            "label_policy_json": LaunchConfiguration("label_policy_json"),
            "semantic_objects_topic": LaunchConfiguration("semantic_objects_topic"),
            "semantic_objects_path": LaunchConfiguration("semantic_objects_path"),
            # 回环检测
            "lc_check_period": LaunchConfiguration("lc_check_period"),
            "lc_translation_threshold": LaunchConfiguration("lc_translation_threshold"),
            "lc_rotation_threshold": LaunchConfiguration("lc_rotation_threshold"),
            "lc_source_frame": LaunchConfiguration("lc_source_frame"),
            "lc_resnap_enabled": LaunchConfiguration("lc_resnap_enabled"),
            "lc_resnap_radius_m": LaunchConfiguration("lc_resnap_radius_m"),
            "lc_resnap_max_attempts": LaunchConfiguration("lc_resnap_max_attempts"),
            "lc_resnap_radius_step_m": LaunchConfiguration("lc_resnap_radius_step_m"),
            "lc_resnap_max_radius_m": LaunchConfiguration("lc_resnap_max_radius_m"),
            "lc_stable_checks": LaunchConfiguration("lc_stable_checks"),
            "lc_stable_translation_threshold": LaunchConfiguration("lc_stable_translation_threshold"),
            "lc_stable_rotation_threshold": LaunchConfiguration("lc_stable_rotation_threshold"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        condition=IfCondition(lc_use_loop_closure),
    )

    # --- 原始种子可视化节点 (调试用) ---
    raw_viz = Node(
        package="my_work_pkg",
        executable="raw_seed_visualizer_node",
        name="raw_seed_visualizer_node",
        output="screen",
        parameters=[{
            "input_topic": "/semantic_seed",
            "marker_lifetime_s": LaunchConfiguration("marker_lifetime_s"),
            "point_scale": LaunchConfiguration("raw_point_scale"),
            "publish_hz": LaunchConfiguration("raw_publish_hz"),
        }],
        condition=IfCondition(lc_enable_debug_viz),
    )

    # --- 种子追踪可视化节点 (可选) ---
    seed_tracker = Node(
        package="my_work_pkg",
        executable="seed_tracker_node",
        name="seed_tracker_node",
        output="screen",
        parameters=[{
            "input_topic": "/semantic_seed",
            "match_distance": LaunchConfiguration("match_distance"),
            "smoothing_alpha": LaunchConfiguration("smoothing_alpha"),
            "memory_timeout": LaunchConfiguration("memory_timeout"),
            "point_scale": LaunchConfiguration("tracker_point_scale"),
            "publish_hz": LaunchConfiguration("tracker_publish_hz"),
            "min_times_seen": LaunchConfiguration("min_times_seen"),
        }],
        condition=IfCondition(lc_enable_seed_tracker),
    )

    # --- 管线诊断节点 ---
    semantic_diag = Node(
        package="my_work_pkg",
        executable="semantic_diag_node",
        name="semantic_diag_node",
        output="screen",
        parameters=[{
            "json_topic": LaunchConfiguration("input_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "seed_topic": "/semantic_seed",
            "map_topic": "/map",
            "camera_frame": LaunchConfiguration("camera_frame"),
            "target_frame": lc_target_frame,
        }],
        condition=IfCondition(lc_enable_diag),
    )

    # ======================================================================
    #  组装 LaunchDescription
    # ======================================================================
    return LaunchDescription([
        # --- 全局开关 ---
        arg_frontend,
        arg_use_loop_closure,
        arg_use_sim_time,
        arg_enable_debug_viz,
        arg_enable_seed_tracker,
        arg_enable_diag,
        # --- 相机 & 深度 & TF ---
        arg_camera_fx,
        arg_camera_fy,
        arg_camera_cx,
        arg_camera_cy,
        arg_depth_scale,
        arg_target_frame,
        arg_tf_timeout_s,
        # --- frontend_bridge 专属 ---
        arg_input_topic,
        arg_depth_topic,
        arg_camera_frame,
        arg_slop,
        arg_pose_iou_threshold,
        arg_enable_pose,
        # --- yolo_seed_projection 专属 ---
        arg_yolo_det_topic,
        arg_yolo_depth_topic,
        # --- 栅格吸附 ---
        arg_search_radius_m,
        arg_occupied_threshold,
        arg_min_island_pixels,
        arg_max_island_size_m,
        arg_max_total_pixels,
        # --- 记忆追踪 ---
        arg_match_distance,
        arg_smoothing_alpha,
        arg_memory_timeout,
        arg_min_confirmed_seen,
        arg_show_candidates,
        arg_label_policy_json,
        arg_semantic_objects_topic,
        arg_semantic_objects_path,
        # --- 回环修正 ---
        arg_lc_check_period,
        arg_lc_translation_threshold,
        arg_lc_rotation_threshold,
        arg_lc_source_frame,
        arg_lc_resnap_enabled,
        arg_lc_resnap_radius_m,
        arg_lc_resnap_max_attempts,
        arg_lc_resnap_radius_step_m,
        arg_lc_resnap_max_radius_m,
        arg_lc_stable_checks,
        arg_lc_stable_translation_threshold,
        arg_lc_stable_rotation_threshold,
        # --- 原始种子可视化 ---
        arg_marker_lifetime_s,
        arg_raw_point_scale,
        arg_raw_publish_hz,
        # --- seed_tracker ---
        arg_tracker_point_scale,
        arg_tracker_publish_hz,
        arg_min_times_seen,
        # --- 节点 ---
        frontend_bridge,
        yolo_seed_proj,
        semantic_proj,
        loop_closure,
        raw_viz,
        seed_tracker,
        semantic_diag,
    ])
