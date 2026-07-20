-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- ============================================================
-- 🎯 这个文件是"后端优化"的核心配置，调参的第二重点！
--
-- 后端负责什么？
--   前端（trajectory_builder_2d.lua）每帧估计一个位姿（node）
--   后端把这些node连接成图，做全局优化，消除累积误差
--
-- 核心工作：
--   1. 回环检测：当机器人回到之前去过的地方，闭合环路
--   2. 全局优化：把所有node和submap一起优化，让地图一致
-- ============================================================

POSE_GRAPH = {
  -- ⭐【重要！】每隔多少个node做一次全局优化
  --   比如=90：每插入90个新node，运行一次后端优化
  --   值小→优化频繁→精度高（漂移被尽早修正）
  --   值大→优化少→速度快（但漂移累积更多）
  --   建议：大场景/长距离=20~30，小场景=60~90
  optimize_every_n_nodes = 70,

  -- ========== 回环约束构建器 ==========

  constraint_builder = {
    sampling_ratio = 0.3,           -- ⭐【重要！】回环检测的采样率
                                    --   每次回环检测时，只检查30%的历史node
                                    --   值大→回环检测更密集→精度高但慢
                                    --   值小→回环检测稀疏→快但可能漏掉回环
                                    --   建议：0.3~0.5

    max_constraint_distance = 15.,  -- ⭐【重要！】回环的最大搜索距离（米）
                                    --   只搜索距离当前15米内的历史submap做回环
                                    --   值大→搜索范围大→能检测远距离回环
                                    --   但太大→计算量暴增+可能误匹配
                                    --   建议：室内10~20，室外20~30

    min_score = 0.55,               -- ⭐【重要！】回环检测的最低分数
                                    --   分数低于此的匹配结果被丢弃
                                    --   值大→回环可靠但可能漏掉
                                    --   值小→回环多但可能有误匹配
                                    --   建议：0.5~0.7

    global_localization_min_score = 0.6,  -- 全局定位模式的最低分数

    loop_closure_translation_weight = 1.1e4,  -- 【一般不改】回环的位移权重
    loop_closure_rotation_weight = 1e5,        -- 【一般不改】回环的旋转权重

    log_matches = true,             -- 【不用改】是否打印回环匹配日志

    -- ========== 快速相关性扫描匹配（回环检测用） ==========
    -- 和前端不一样，这是专门用于回环检测的粗匹配器

    fast_correlative_scan_matcher = {
      linear_search_window = 7.,          -- ⭐【重要！】回环搜索范围（米）
                                           --   在[-7米, +7米]范围内搜索
      angular_search_window = math.rad(30.), -- ⭐【重要！】回环搜索角度范围（度）
      branch_and_bound_depth = 7,         -- 分支定界搜索深度
                                           --   值大→搜索更精细但更慢
                                           --   值小→搜索粗糙但快
    },

    ceres_scan_matcher = {
      -- 回环检测的精匹配（和前端类似，但参数独立）

      occupied_space_weight = 20.,        -- 占据空间权重
      translation_weight = 10.,           -- 位移权重
      rotation_weight = 1.,               -- 旋转权重

      ceres_solver_options = {
        use_nonmonotonic_steps = true,    -- 【不用改】
        max_num_iterations = 10,          -- 迭代次数
        num_threads = 1,
      },
    },

    -- ========== 3D回环检测（不用管，3D才用） ==========
    fast_correlative_scan_matcher_3d = {
      branch_and_bound_depth = 8,
      full_resolution_depth = 3,
      min_rotational_score = 0.77,
      min_low_resolution_score = 0.55,
      linear_xy_search_window = 5.,
      linear_z_search_window = 1.,
      angular_search_window = math.rad(15.),
    },
    ceres_scan_matcher_3d = {
      occupied_space_weight_0 = 5.,
      occupied_space_weight_1 = 30.,
      translation_weight = 10.,
      rotation_weight = 1.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },

  -- ========== 匹配器权重（优化时用） ==========

  matcher_translation_weight = 5e2,    -- 【一般不改】优化的位移权重
  matcher_rotation_weight = 1.6e3,     -- 【一般不改】优化的旋转权重

  -- ========== 优化问题配置 ==========

  optimization_problem = {
    huber_scale = 1e1,                 -- 胡伯损失函数参数（抑制离群点）
                                       --   值越大→对误差越宽容
                                       --   值越小→越苛刻

    acceleration_weight = 1.1e2,       -- 加速度约束权重（平滑轨迹用）
    rotation_weight = 1.6e4,           -- 旋转约束权重

    local_slam_pose_translation_weight = 1e5,  -- 【不用改】前端位姿的位移权重
    local_slam_pose_rotation_weight = 1e5,     -- 【不用改】前端位姿的旋转权重

    odometry_translation_weight = 1e5,  -- 里程计位移权重（如果有里程计）
    odometry_rotation_weight = 1e5,     -- 里程计旋转权重

    fixed_frame_pose_translation_weight = 1e1, -- GPS等固定位姿的位移权重
    fixed_frame_pose_rotation_weight = 1e2,    -- GPS等固定位姿的旋转权重

    fixed_frame_pose_use_tolerant_loss = false,
    fixed_frame_pose_tolerant_loss_param_a = 1,
    fixed_frame_pose_tolerant_loss_param_b = 1,

    log_solver_summary = false,         -- 【不用改】是否打印优化器日志
    use_online_imu_extrinsics_in_3d = true,  -- 【3D用】是否在线标定IMU外参
    fix_z_in_3d = false,                -- 【3D用】是否固定Z轴

    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,          -- 【可调】单次优化的最大迭代次数
                                         --   越大→优化越精细
                                         --   一般30~50
      num_threads = 7,                  -- 【可调】优化线程数
                                         --   建议=CPU核数-1
    },
  },

  -- ========== 最终优化 ==========

  max_num_final_iterations = 300,       -- 【可调】建图完成后的最终全局优化迭代次数
                                         --   值越大→最终地图越精细
                                         --   一般100~300

  -- ⭐【重要！】全局回环采样率
  --   在全局优化中，从所有submap中随机采样做回环检测
  --   值大→回环多→地图好但慢
  --   值小→回环少→快但可能不准
  --   建议：0.003~0.01
  global_sampling_ratio = 0.01,

  log_residual_histograms = true,        -- 【不用改】是否记录残差直方图

  -- ⭐【重要！】全局回环搜索延迟（秒）
  --   机器人启动后过多久才开始搜索全局回环
  --   值大→等建图稳定后再搜索（减少误匹配）
  --   值小→早早开始搜索回环
  global_constraint_search_after_n_seconds = 10.,

  -- submaps修剪器（被注释掉，不需要管）
  -- 用于删除重复覆盖的submap，节省内存
  --  overlapping_submaps_trimmer_2d = {
  --    fresh_submaps_count = 1,
  --    min_covered_area = 2,
  --    min_added_submaps_count = 5,
  --  },
}