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

-- ============backpack2d.lua============= 
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = true,
  num_laser_scans = 1,  --启用路标
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 1e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10

TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 2
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
--  TEST 
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2e2
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
MAP_BUILDER.num_background_threads = 4


--  TEST
POSE_GRAPH.optimize_every_n_nodes = 50
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.90 --
POSE_GRAPH.global_sampling_ratio = 0.001
POSE_GRAPH.constraint_builder.sampling_ratio = 0.001
POSE_GRAPH.constraint_builder.loop_closure_translation_weight =1e5

return options
-- ============backpack2d.lua============= 
-- 基于Cartographer官方backpack_2d.lua修改的纯激光雷达配置
-- include "map_builder.lua"
-- include "trajectory_builder.lua"

-- options = {
--   map_builder = MAP_BUILDER,
--   trajectory_builder = TRAJECTORY_BUILDER,
  
--   -- 坐标系配置
--   map_frame = "map",
--   tracking_frame = "base_link",     -- 雷达数据对应的基坐标系
--   published_frame = "base_link",    -- 发布的坐标系
--   odom_frame = "odom",             -- 里程计坐标系
--   provide_odom_frame = false,       -- 不提供odom坐标系
  
--   -- 传感器配置
--   use_odometry = false,            -- 禁用里程计
--   use_nav_sat = false,             -- 禁用GPS
--   use_landmarks = false,           -- 禁用路标
--   use_pose_extrapolator = false,   -- 禁用位姿推断
  
--   -- 激光雷达配置
--   num_laser_scans = 1,             -- 使用单线雷达
--   num_multi_echo_laser_scans = 0,  -- 禁用多回波雷达
--   num_subdivisions_per_laser_scan = 1, -- 不细分扫描数据
--   num_point_clouds = 0,            -- 禁用点云
  
--   -- 话题配置
--   laser_scan_topic = "/scan",      -- 雷达话题
  
--   -- 性能参数
--   lookup_transform_timeout_sec = 0.2,
--   submap_publish_period_sec = 0.3,
--   pose_publish_period_sec = 0.005,
--   trajectory_publish_period_sec = 0.03,
  
--   -- 采样率（保持默认）
--   rangefinder_sampling_ratio = 1.0,
-- }

-- -- 2D地图构建器配置
-- MAP_BUILDER.use_trajectory_builder_2d = true

-- -- 2D轨迹构建器参数优化
-- TRAJECTORY_BUILDER_2D = {
--   min_range = 0.3,                  -- 最小有效距离（根据雷达性能调整）
--   max_range = 12.,                  -- 最大有效距离（根据N10P参数设置）
--   missing_data_ray_length = 5.,     -- 缺失数据射线长度
  
--   use_imu_data = false,             -- 禁用IMU
  
--   motion_filter = {
--     max_time_seconds = 5.0,
--     max_distance_meters = 0.2,
--     max_angle_radians = math.rad(1.),
--   },
  
--   submaps = {
--     grid_options_2d = {
--       grid_type = "PROBABILITY_GRID",
--       resolution = 0.05,            -- 地图分辨率（可调）
--     },
--     num_range_data = 90,            -- 每个子图包含的扫描次数
--   },
  
--   -- 实时相关性扫描匹配器（提高建图精度）
--   real_time_correlative_scan_matcher = {
--     linear_search_window = 0.1,
--     angular_search_window = math.rad(20.),
--     translation_delta_cost_weight = 1e-1,
--     rotation_delta_cost_weight = 1e-1,
--   }
-- }

-- return options

