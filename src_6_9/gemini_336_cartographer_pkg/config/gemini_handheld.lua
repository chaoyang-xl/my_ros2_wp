include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  
  -- 【注意】由于合并脚本直接复制了陀螺仪的 Header，这里的 tracking_frame 
  -- 通常是 "camera_gyro_optical_frame" 或 "camera_imu_optical_frame"。
  -- 如果运行报错说找不到 TF，请根据 ros2 topic echo /camera/imu 里的 frame_id 修改这里。
  tracking_frame = "camera_gyro_optical_frame", 
  
  published_frame = "camera_link",
  odom_frame = "odom",
  provide_odom_frame = true,            -- 手持无底盘，由 Cartographer 自行发布里程计
  publish_frame_projected_to_2d = true, -- 将手持时的上下抖动投影到 2D 平面上
  
  use_pose_extrapolator = true,
  use_odometry = false,                 -- 关掉轮式里程计
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,                  -- 接收 pointcloud_to_laserscan 转换后的 2D 激光
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  -- 新增：ROS 2 较新版本 Cartographer 强制要求的采样率参数
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- ==== 针对 Gemini 336L 的室内 2D 轨迹构建器调优 ====
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 6.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = true 

-- IMU 信任度调整（手持建图必须高度信任 IMU）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40. 

-- 运动滤波（稍微走动距离就更新地图，让局部细节更丰富）
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 2.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)

return options