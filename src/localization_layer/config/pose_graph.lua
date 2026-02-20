POSE_GRAPH = {
  optimization_problem = {
    huber_scale = 1e1,
    acceleration_weight = 1.1e2,
    relative_pose_translation_weight = 5.e5,
    relative_pose_rotation_weight = 1.6e4,
    odometry_translation_weight = 0.1,
    odometry_rotation_weight = 0.1,
    fixed_frame_pose_translation_weight = 1.e5,
    fixed_frame_pose_rotation_weight = 1.e5,
    log_solver_variance = true,
    use_online_imu_extrinsics_in_3d = true,
  },
  constraint_builder = {
    sampling_ratio = 0.3,
    max_constraint_distance = 15.,
    min_score = 0.55,
    global_localization_min_score = 0.6,
    loop_closure_translation_weight = 1.1e4,
    loop_closure_rotation_weight = 1.e5,
  },
  matcher_translation_weight = 5.e2,
  matcher_rotation_weight = 1.6e4,
  optimization_every_n_nodes = 90,
}

return POSE_GRAPH