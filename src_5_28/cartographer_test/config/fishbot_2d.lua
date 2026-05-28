include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 【稳妥修复版配置】 --

-- 1. 先强制关闭 IMU (确保地图能出来)
TRAJECTORY_BUILDER_2D.use_imu_data = true  -- fishbot 有 IMU，可以开启
-- 【核心修改 2】开启 IMU
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- 【核心修改 3】因为有了 IMU，我们可以更信任旋转数据，减少重影
-- 撞墙时，雷达离墙很近。如果 min_range 设为 0.15，
-- 撞墙时墙壁可能就进入盲区了，算法就瞎了。
TRAJECTORY_BUILDER_2D.min_range = 0.05
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 

-- IMU 提供的重力数据权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
-- 增加相关性扫描匹配的权重 (暴力匹配)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
return options