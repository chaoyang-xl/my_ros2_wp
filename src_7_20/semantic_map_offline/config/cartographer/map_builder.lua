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
-- 🎯 这个文件是"后端框架"的配置
-- 主要管两件事：
--   1. 全局地图构建的框架设置
--   2. 引入pose_graph.lua（真正的后端优化配置）
-- ============================================================

-- 引入后端核心：位姿图优化配置
include "pose_graph.lua"

-- ╔══════════════════════════════════════════════════════════╗
-- ║  MAP_BUILDER：全局地图构建器                           ║
-- ║  它负责管理整个建图过程的框架                           ║
-- ╚══════════════════════════════════════════════════════════╝
MAP_BUILDER = {
  use_trajectory_builder_2d = false,   -- 是否用2D建图
                                       --   默认false，由backpack_2d.lua覆写为true
  use_trajectory_builder_3d = false,   -- 是否用3D建图（和2D只能选一个）

  num_background_threads = 4,          -- 【可调】后端优化的线程数
                                       --   CPU核数多就调大（如8核→6或8）
                                       --   调大可以让后端优化更快完成

  pose_graph = POSE_GRAPH,             -- 指向pose_graph.lua里定义的配置

  collate_by_trajectory = false,       -- 【不用改】多轨迹时是否按轨迹分组
}