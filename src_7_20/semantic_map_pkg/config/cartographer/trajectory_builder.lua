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
-- 🎯 这个文件是"前端框架"的配置
-- 主要作用：
--   1. 引入 trajectory_builder_2d.lua（2D前端配置）
--   2. 引入 trajectory_builder_3d.lua（3D前端配置）
--   3. 组装 TRAJECTORY_BUILDER 对象（包含前后端）
-- ============================================================

-- 引入2D前端的详细配置
include "trajectory_builder_2d.lua"
-- 引入3D前端的详细配置（如果不用3D可以忽略）
include "trajectory_builder_3d.lua"

-- ╔══════════════════════════════════════════════════════════╗
-- ║  TRAJECTORY_BUILDER：轨迹构建器                       ║
-- ║  这里只是把前端配置组装起来，本身没有太多参数         ║
-- ║  真正的调参在 trajectory_builder_2d.lua 和           ║
-- ║  pose_graph.lua 里进行                                 ║
-- ╚══════════════════════════════════════════════════════════╝
TRAJECTORY_BUILDER = {
  -- 指向 trajectory_builder_2d.lua 里定义的完整配置
  trajectory_builder_2d = TRAJECTORY_BUILDER_2D,
  -- 指向 trajectory_builder_3d.lua 里定义的完整配置
  trajectory_builder_3d = TRAJECTORY_BUILDER_3D,

  -- 纯定位模式（被注释掉了，不需要动）
  -- pure_localization_trimmer = {
  --   max_submaps_to_keep = 3,
  -- },

  collate_fixed_frame = true,   -- 【不用改】是否收集固定帧数据（如GPS）
  collate_landmarks = false,    -- 【不用改】是否收集路标数据
}