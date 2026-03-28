include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "vehicle_blue/chassis",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
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
  
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 1. BALANCED LOCAL MATCHING
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0 -- Lowered! Trust the wheels for forward distance
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0   -- Keep high to prevent the map from spinning

-- 2. RESTORE GLOBAL ODOMETRY TRUST
-- We restore the default weights so Cartographer stops warping the map
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

-- 3. SENSOR TUNING
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 25.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 26.0

-- 4. CRISP SUBMAPS
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90 -- Let it collect more data before finalizing a chunk of the map
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- 5. RELAX THE POSE GRAPH
-- Stop the algorithm from constantly tearing the map apart trying to fix it
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.65

return options
