include "ano_imu_2d.lua"  -- 包含您原来的建图配置

-- 纯定位核心设置
pure_localization = true
pure_localization_trimmer = { max_submaps_to_keep = 3 }

-- 性能优化设置
MAP_BUILDER.num_background_threads = 1  -- 设置为CPU核心数

POSE_GRAPH.optimize_every_n_nodes = 0

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.0

return options