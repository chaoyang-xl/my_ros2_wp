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
-- 🎯 这个文件是2D建图的"前端核心配置"，调参的重中之重！
-- 前端负责：把每一帧激光数据匹配到地图上，确定机器人位置
--
-- 核心流程：
--   1. 激光雷达数据进来
--   2. 体素滤波（降采样，减少计算量）
--   3. 扫描匹配（找当前帧对应地图的最佳位置）
--   4. 运动滤波器（判断是否要插入新node）
--   5. 插入到submap中
-- ============================================================

TRAJECTORY_BUILDER_2D = {
  -- ========== 传感器数据预处理 ==========

  use_imu_data = false,         -- 【建议true】是否使用IMU数据
                                --   有IMU=true（能提高旋转估计精度）
                                --   无IMU=false

  min_range = 0.25,              -- 【可调】激光最小有效距离（米）
                                --   低于这个值的数据被忽略（太近可能是噪声）
                                --   室内一般0.，室外可以考虑0.3~1.0

  max_range = 5.,             -- ⭐【重要！】激光最大有效距离（米）
                                --   超过这个距离的数据被忽略
                                --   室内小房间=10，走廊/博物馆=30+
                                --   设太大会引入远处噪声，设太小会丢失结构信息

  min_z = -0.8,                -- 【3D激光用】Z轴最小高度，低于此的忽略
  max_z = 2.,                  -- 【3D激光用】Z轴最大高度，高于此的忽略

  missing_data_ray_length = 2.0, -- 【一般不改】激光没打到物体时的回波长度

  num_accumulated_range_data = 1,  -- ⭐【重要！】一次处理前累加多少帧激光
                                    --   =1：每帧独立处理
                                    --   >1：累加多帧成稠密点云再处理
                                    --   优点：点云更密，匹配更准
                                    --   缺点：运动畸变更大（快速运动时容易糊）
                                    --   backpack_2d.lua中覆写为10

  voxel_filter_size = 0.025,   -- 【可调】体素滤波的大小（米）
                                --   对激光点云做下采样，降低点数
                                --   越小→点越多→越精确但越慢
                                --   越大→点越少→越快但丢失细节
                                --   一般室内0.025~0.05

  -- ========== 自适应体素滤波器 ==========

  adaptive_voxel_filter = {
    max_length = 0.5,          -- 【一般不改】自适应滤波的搜索半径（米）
    min_num_points = 200,      -- 【一般不改】最少保留的点数
    max_range = 50.,           -- 【一般不改】自适应滤波的最大距离
  },

  -- ========== 回环检测用的体素滤波器 ==========

  loop_closure_adaptive_voxel_filter = {
    max_length = 0.9,          -- 【一般不改】回环检测的自适应滤波半径
    min_num_points = 100,      -- 【一般不改】回环检测最少保留点数
    max_range = 50.,           -- 【一般不改】回环检测的最大距离
  },

  -- ========== 扫描匹配（最重要！）==========

  -- ⭐【极重要！】是否开启相关性扫描匹配（CSM）
  --   false：只用Ceres（梯度下降法）做精匹配
  --   优点：速度快
  --   缺点：如果初始猜测不准，容易匹配到错误位置（=毛刺）
  --
  --   true：先做粗匹配（暴力搜索最佳位置）+ 再做精匹配
  --   优点：匹配更鲁棒，不容易跟丢，对无里程计场景尤其重要
  --   缺点：CPU消耗大很多（2~3倍）
  --
  --   建议：有轮式里程计 → false
  --         纯雷达（无里程计）→ true
  use_online_correlative_scan_matching = false,

  real_time_correlative_scan_matcher = {
    -- 下面四个参数只有在 use_online_correlative_scan_matching = true 时才生效

    linear_search_window = 0.1,    -- ⭐【重要！】粗匹配搜索范围（米）
                                   --   Cartographer会在[-0.1米, +0.1米]范围内搜索最佳匹配
                                   --   运动越快/里程计越差 → 设越大（如0.5、1.0）
                                   --   运动慢/有里程计 → 设小（如0.1）
                                   --   太大：计算量暴增  太小：搜不到正确位置

    angular_search_window = math.rad(20.),  -- ⭐【重要！】粗匹配角度搜索范围（度）
                                             --   math.rad(20)=20°，范围越大越能应对大角度旋转
                                             --   建议：20°~60°

    translation_delta_cost_weight = 1e-1,   -- 【一般不改】位移变化的惩罚权重
    rotation_delta_cost_weight = 1e-1,      -- 【一般不改】角度变化的惩罚权重
  },

  ceres_scan_matcher = {
    -- Ceres是精匹配，在粗匹配结果基础上微调

    occupied_space_weight = 1.,   -- 【可调】占据空间匹配权重
                                  --   越大→越要求激光点落在占据格子上

    translation_weight = 10.,     -- 【可调】位移权重
                                  --   越大→匹配时越不愿意移动（更保守）
                                  --   越小→匹配时越容易移动（更灵活）

    rotation_weight = 40.,        -- 【可调】旋转权重
                                  --   越大→匹配时越不愿意旋转
                                  --   越小→匹配时越容易旋转

    ceres_solver_options = {
      use_nonmonotonic_steps = false, -- 【不用改】是否用非单调步长
      max_num_iterations = 20,       -- 【可调】最大迭代次数
                                      --   越大→匹配越精细但越慢
                                      --   一般10~20
      num_threads = 1,               -- 【不用改】Ceres内部线程数
    },
  },

  -- ========== 运动滤波器（控制node插入频率）==========
  -- ⭐【极重要！】决定什么时候插入一个新node到submap中
  -- 每插入一个node = 往地图里添加一帧数据
  -- 条件1：距离上次插入超过了 max_time_seconds 秒
  -- 条件2：移动距离超过 max_distance_meters 米
  -- 条件3：旋转角度超过 max_angle_radians 度
  -- 满足任意条件就插入一个node

  motion_filter = {
    max_time_seconds = 5.,         -- 【可调】最大时间间隔（秒）
                                   --   机器人即使不动，每 N 秒也插入一个node
                                   --   设小→node多→精度高但计算量大
                                   --   设大→node少→计算量小但可能漏掉细节
                                   --   建议：慢速运动=2~3，快速运动=5

    max_distance_meters = 0.2,     -- ⭐【重要！】最大位移间隔（米）
                                   --   机器人每走 N 米插入一个node
                                   --   设小→node密集→小细节更好
                                   --   设大→node稀疏→计算量小
                                   --   一般0.1~0.3

    max_angle_radians = math.rad(1.),  -- ⭐【重要！】最大角度间隔（度）
                                       --   机器人每转 N 度插入一个node
                                       --   设小→旋转时node密集→旋转建图更准
                                       --   设大→旋转时node稀疏
                                       --   math.rad(1)=1°, 一般0.5°~5°
  },

  -- ========== IMU相关 ==========

  -- 以下参数是ROS遗留，一般不改
  imu_gravity_time_constant = 10.,

  -- ========== 位姿外推器 ==========
  -- 在没有新激光数据时，用历史运动趋势预测当前位姿

  pose_extrapolator = {
    use_imu_based = false,         -- 是否用IMU做位姿外推
                                   --   true→更准但需要IMU
                                   --   false→用匀速模型

    constant_velocity = {          -- 匀速运动模型
      imu_gravity_time_constant = 10.,
      pose_queue_duration = 0.001, -- 用过去多久的数据预测
    },

    imu_based = {                  -- IMU运动模型（use_imu_based=true时生效）
      pose_queue_duration = 5.,
      gravity_constant = 9.806,
      pose_translation_weight = 1.,
      pose_rotation_weight = 1.,
      imu_acceleration_weight = 1.,
      imu_rotation_weight = 1.,
      odometry_translation_weight = 1.,
      odometry_rotation_weight = 1.,
      solver_options = {
        use_nonmonotonic_steps = false;
        max_num_iterations = 10;
        num_threads = 1;
      },
    },
  },

  -- ========== Submap（子图）配置 ==========

  submaps = {
    -- ⭐【重要！】每个submap包含多少帧激光数据
    -- submap = 小的局部地图，建好后就固定不变
    --   值小→submap小→优化频繁→精度高
    --   值大→submap大→优化少→速度快
    --   建议：快速运动=30~60，慢速运动=60~90
    num_range_data = 90,

    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",  -- 地图类型（一般用PROBABILITY_GRID）
                                       -- PROBABILITY_GRID = 概率栅格地图
                                       -- TSDF = 截断符号距离场（实验性）
      resolution = 0.05,               -- ⭐【重要！】地图分辨率（米/格）
                                       --   0.05=5厘米/格（常规）
                                       --   0.025=2.5厘米/格（精细但慢）
                                       --   0.1=10厘米/格（粗糙但快）
    },

    range_data_inserter = {
      -- 激光数据如何写入地图

      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",

      probability_grid_range_data_inserter = {
        insert_free_space = true,    -- 是否更新空闲区域（建议true）

        hit_probability = 0.55,      -- 激光打中栅格的概率增加值
                                     --   越大→地图更新越快，但容易过信
                                     --   建议：0.55~0.7

        miss_probability = 0.49,     -- 激光穿过栅格的概率减少值
                                     --   建议：0.4~0.49
      },

      -- TSDF相关（用不到就不用管）
      tsdf_range_data_inserter = {
        truncation_distance = 0.3,
        maximum_weight = 10.,
        update_free_space = false,
        normal_estimation_options = {
          num_normal_samples = 4,
          sample_radius = 0.5,
        },
        project_sdf_distance_to_scan_normal = true,
        update_weight_range_exponent = 0,
        update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5,
        update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,
      },
    },
  },
}