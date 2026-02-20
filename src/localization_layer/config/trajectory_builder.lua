TRAJECTORY_BUILDER = {
  max_submaps_to_keep = 3,
  submaps_options_2d = {
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",
    },
    range_data_inserter = {
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
    },
  },
  motion_filter = {
    max_time_seconds = 5.,
    max_distance_meters = 0.2,
    max_angle_radians = 0.017453,
  },
}

TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,
  use_online_correlative_scan_matching = true,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.1,
    angular_search_window = 0.017453,
    translation_delta_cost_weight = 1.0,
    rotation_delta_cost_weight = 1.0,
  },
  ceres_scan_matcher = {
    occupied_space_weight = 1.0,
    translation_weight = 10.0,
    rotation_weight = 40.0,
  },
  submaps = {
    num_range_data = 90,
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",
    },
  },
}

return TRAJECTORY_BUILDER