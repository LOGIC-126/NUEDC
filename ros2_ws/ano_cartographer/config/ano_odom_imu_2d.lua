include "map_builder.lua"  -- 包含地图构建模块
include "trajectory_builder.lua"  -- 包含轨迹构建模块

-- 全局配置选项表
options = {
  map_builder = MAP_BUILDER,                -- 地图构建器配置
  trajectory_builder = TRAJECTORY_BUILDER,  -- 轨迹构建器配置
  map_frame = "map",                        -- 地图坐标系名称 (字符串)
  tracking_frame = "imu_link",              -- 传感器数据关联的坐标系 (字符串)
  published_frame = "base_link",            -- 发布位姿的坐标系 (字符串)
  odom_frame = "odom",                      -- 里程计坐标系名称 (字符串)
  provide_odom_frame = false,               -- 是否发布odom到map的TF (布尔值)
  publish_frame_projected_to_2d = true,     -- 是否将3D位姿投影到2D (布尔值)
  use_pose_extrapolator = false,             -- 是否使用位姿推测器 (布尔值)
  use_odometry = true,                     -- 是否使用里程计数据 (布尔值)
  use_nav_sat = false,                      -- 是否使用GPS数据 (布尔值)
  use_landmarks = false,                    -- 是否使用地标数据 (布尔值)
  num_laser_scans = 1,                      -- 使用的激光扫描话题数量 (整数)
  num_multi_echo_laser_scans = 0,           -- 多回波激光扫描话题数量 (整数)
  num_subdivisions_per_laser_scan = 1,      -- 每帧扫描分割次数 (整数)
  num_point_clouds = 0,                     -- 点云话题数量 (整数)
  lookup_transform_timeout_sec = 0.2,       -- TF查询超时时间 (秒)
  submap_publish_period_sec = 0.3,          -- 子图发布周期 (秒)
  pose_publish_period_sec = 1e-3,           -- 位姿发布周期 (秒)
  trajectory_publish_period_sec = 15e-3,    -- 轨迹发布周期 (秒)
  rangefinder_sampling_ratio = 1.,          -- 测距仪采样率 (浮点数)
  odometry_sampling_ratio = 1.,             -- 里程计采样率 (浮点数)
  fixed_frame_pose_sampling_ratio = 1.,     -- 固定帧位姿采样率 (浮点数)
  imu_sampling_ratio = 1.,                  -- IMU采样率 (浮点数)
  landmarks_sampling_ratio = 1.,            -- 地标采样率 (浮点数)
}
-- 里程计专用配置

TRAJECTORY_BUILDER_2D.use_odometry = true            -- 启用里程计数据
TRAJECTORY_BUILDER_2D.odom_translation_weight = 5e2  -- 位置权重 (较高表示更信任里程计)
TRAJECTORY_BUILDER_2D.odom_rotation_weight = 5e2     -- 旋转权重

-- 位姿图优化中的里程计权重
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e4

-- 地图构建器参数配置
MAP_BUILDER.use_trajectory_builder_2d = true        -- 启用2D轨迹构建器 (布尔值)

-- 轨迹构建器参数配置 (2D)
TRAJECTORY_BUILDER_2D.min_range = 0.3               -- 最小有效测距距离 (米)
TRAJECTORY_BUILDER_2D.max_range = 3.5                -- 最大有效测距距离 (米)
TRAJECTORY_BUILDER_2D.min_z = 0.2
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.  -- 缺失数据射线长度 (米)
TRAJECTORY_BUILDER_2D.use_imu_data = true           -- 是否使用IMU数据 (布尔值)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 是否使用在线相关扫描匹配 (布尔值)

-- 运动滤波器参数 (减少冗余位姿)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.8             -- 最大时间间隔 (秒)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2         -- 最大移动距离 (米)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)  -- 最大旋转角度 (弧度)
TRAJECTORY_BUILDER_2D.motion_filter.min_movement_speed = 0.1           -- 最小移动速度 (m/s)
TRAJECTORY_BUILDER_2D.motion_filter.hover_detection_duration = 1.0     -- 悬停检测持续时间 (秒)
-- 扫描匹配优化参数
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2e2                      -- 平移优化权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20  -- 优化器最大迭代次数
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1                                   -- 累积的扫描数据数量
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.02                                         -- 体素滤波器尺寸 (米)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90                                      -- 每个子图包含的扫描次数
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 1                                    -- IMU重力时间常数 (秒)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1e3                         -- 旋转优化权重

-- 并行处理参数
MAP_BUILDER.num_background_threads = 6  -- 后台处理线程数

-- 位姿图优化参数 (全局优化)
POSE_GRAPH.optimize_every_n_nodes = 180                                -- 每N个节点全局优化一次
POSE_GRAPH.constraint_builder.min_score = 0.48                        -- 闭环检测最小分数阈值
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60    -- 全局定位最小分数阈值
POSE_GRAPH.global_sampling_ratio = 0.001                              -- 全局约束采样率
POSE_GRAPH.constraint_builder.sampling_ratio = 0.001                  -- 约束采样率
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e5   -- 闭环平移约束权重

return options  -- 返回配置表