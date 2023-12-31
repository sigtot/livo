#include "global_params.h"

GlobalParams& GlobalParams::GetInstance()
{
  static GlobalParams instance;
  return instance;
}

template <class T>
void GlobalParams::ReadVariable(const ros::NodeHandle& nh, const std::string& variable_name, T& variable)
{
  if (!nh.getParam(variable_name, variable))
  {
    ROS_WARN_STREAM("WARN: Could not read param " << variable_name
                                                  << " from parameter server, so using "
                                                     "default value "
                                                  << variable);
  }
  else
  {
    ROS_INFO_STREAM("Read param " << variable_name << " from parameter server: " << variable);
  }
}

template <class T>
void GlobalParams::ReadVectorVariable(const ros::NodeHandle& nh, const std::string& variable_name,
                                      std::vector<T>& variable)
{
  if (!nh.getParam(variable_name, variable))
  {
    ROS_WARN_STREAM("WARN: Could not read param " << variable_name
                                                  << " from parameter server, so using "
                                                     "default value");
    std::cout << "Default value: ";
    for (auto& x : variable)
    {
      std::cout << x << " ";
    }
    std::cout << std::endl;
  }
  else
  {
    ROS_INFO_STREAM("Read param " << variable_name << " from parameter server: ");
    std::cout << "value ";
    for (auto& x : variable)
    {
      std::cout << x << " ";
    }
    std::cout << std::endl;
  }
}

void GlobalParams::LoadParams(const ros::NodeHandle& nh)
{
  // Add ReadVariable calls here
  ReadVariable(nh, "/orb_test_node/imu_sub_topic", GetInstance().imu_sub_topic_);
  ReadVariable(nh, "/orb_test_node/cam_sub_topic", GetInstance().cam_sub_topic_);
  ReadVariable(nh, "/orb_test_node/lidar_sub_topic", GetInstance().lidar_sub_topic_);
  ReadVariable(nh, "/orb_test_node/lidar_frame_for_publish", GetInstance().lidar_frame_for_publish_);

  ReadVariable(nh, "/orb_test_node/full_path_export_file_prefix", GetInstance().full_path_export_file_prefix_);

  ReadVariable(nh, "/orb_test_node/count_features_per_cell", GetInstance().count_features_per_cell_);
  ReadVariable(nh, "/orb_test_node/min_features_per_cell", GetInstance().min_features_per_cell_);
  ReadVariable(nh, "/orb_test_node/max_features_per_cell", GetInstance().max_features_per_cell_);
  ReadVariable(nh, "/orb_test_node/max_features", GetInstance().max_features_);
  ReadVariable(nh, "/orb_test_node/grid_cells_x", GetInstance().grid_cells_x_);
  ReadVariable(nh, "/orb_test_node/grid_cells_y", GetInstance().grid_cells_y_);
  ReadVariable(nh, "/orb_test_node/resize_factor", GetInstance().resize_factor_);
  ReadVariable(nh, "/orb_test_node/pre_processing/pre_process", GetInstance().pre_process_);
  ReadVariable(nh, "/orb_test_node/pre_processing/contrast_alpha", GetInstance().contrast_alpha_);
  ReadVariable(nh, "/orb_test_node/pre_processing/contrast_beta", GetInstance().contrast_beta_);

  ReadVariable(nh, "/orb_test_node/klt/max_iterations", GetInstance().klt_max_iterations_);
  ReadVariable(nh, "/orb_test_node/klt/convergence_epsilon", GetInstance().klt_convergence_epsilon_);
  ReadVariable(nh, "/orb_test_node/klt/win_size", GetInstance().klt_win_size_);
  ReadVariable(nh, "/orb_test_node/klt/pyramids", GetInstance().klt_pyramids_);

  ReadVariable(nh, "/orb_test_node/feature_extraction/quality_level", GetInstance().feature_extraction_quality_level_);
  ReadVariable(nh, "/orb_test_node/feature_extraction/min_distance", GetInstance().feature_extraction_min_distance_);
  ReadVariable(nh, "/orb_test_node/feature_extraction/min_eigen_value",
               GetInstance().feature_extraction_min_eigen_value_);

  ReadVariable(nh, "/orb_test_node/landmark_culling_frame_count", GetInstance().landmark_culling_frame_count_);
  ReadVariable(nh, "/orb_test_node/landmark_culling_observation_percentage",
               GetInstance().landmark_culling_observation_percentage_);
  ReadVariable(nh, "/orb_test_node/landmark_matching_window", GetInstance().landmark_matching_window_);
  ReadVariable(nh, "/orb_test_node/ground_truth_file", GetInstance().ground_truth_file_);
  ReadVariable(nh, "/orb_test_node/ground_truth_provider", GetInstance().ground_truth_provider_);
  ReadVariable(nh, "/orb_test_node/lidar_time_offset_file", GetInstance().lidar_time_offset_file_);
  ReadVariable(nh, "/orb_test_node/lidar_time_offset_type", GetInstance().lidar_time_offset_type_);
  ReadVariable(nh, "/orb_test_node/lidar_frame_manager_timestamp_thresh",
               GetInstance().lidar_frame_manager_timestamp_thresh_);
  ReadVariable(nh, "/orb_test_node/init_on_ground_truth", GetInstance().init_on_ground_truth_);
  ReadVariable(nh, "/orb_test_node/match_max_distance", GetInstance().match_max_distance_);
  ReadVariable(nh, "/orb_test_node/init_keyframe_interval", GetInstance().init_keyframe_interval_);
  ReadVariable(nh, "/orb_test_node/num_good_keyframes_for_initialization",
               GetInstance().num_good_keyframes_for_initialization_);
  ReadVariable(nh, "/orb_test_node/add_essential_matrix_constraints", GetInstance().add_essential_matrix_constraints_);
  ReadVariable(nh, "/orb_test_node/min_keyframe_feature_inlier_ratio",
               GetInstance().min_keyframe_feature_inlier_ratio_);
  ReadVariable(nh, "/orb_test_node/min_parallax_for_smoothing", GetInstance().min_parallax_for_smoothing_);
  ReadVariable(nh, "/orb_test_node/min_parallax_for_smoothing_depth", GetInstance().min_parallax_for_smoothing_depth_);
  ReadVariable(nh, "/orb_test_node/max_parallax_rotation_compensation",
               GetInstance().max_parallax_rotation_compensation_);
  ReadVariable(nh, "/orb_test_node/num_high_parallax_points_for_keyframe",
               GetInstance().num_high_parallax_points_for_keyframe_);
  ReadVariable(nh, "/orb_test_node/min_active_track_count", GetInstance().min_active_track_count_);
  ReadVariable(nh, "/orb_test_node/use_angle_parallax", GetInstance().use_angle_parallax_);
  ReadVariable(nh, "/orb_test_node/max_depth_for_smoothing", GetInstance().max_depth_for_smoothing_);
  ReadVariable(nh, "/orb_test_node/min_depth_for_smoothing", GetInstance().min_depth_for_smoothing_);

  ReadVariable(nh, "/orb_test_node/use_parallax_keyframes", GetInstance().use_parallax_keyframes_);
  ReadVariable(nh, "/orb_test_node/min_parallax_for_keyframe", GetInstance().min_parallax_for_keyframe_);
  ReadVariable(nh, "/orb_test_node/temporal_keyframe_interval", GetInstance().temporal_keyframe_interval_);
  ReadVariable(nh, "/orb_test_node/frame_interval", GetInstance().frame_interval_);
  ReadVariable(nh, "/orb_test_node/second_ransac_n_frames", GetInstance().second_ransac_n_frames_);

  ReadVariable(nh, "/orb_test_node/use_isam", GetInstance().use_isam_);
  ReadVariable(nh, "/orb_test_node/use_dogleg", GetInstance().use_dogleg_);
  ReadVariable(nh, "/orb_test_node/isam_relinearize_thresh", GetInstance().isam_relinearize_thresh_);
  ReadVariable(nh, "/orb_test_node/isam_relinearize_skip", GetInstance().isam_relinearize_skip_);
  ReadVariable(nh, "/orb_test_node/save_factor_graphs_to_file", GetInstance().save_factor_graphs_to_file_);
  ReadVariable(nh, "/orb_test_node/init_range_factor_length", GetInstance().init_range_factor_length_);
  ReadVariable(nh, "/orb_test_node/min_keyframes_for_nominal", GetInstance().min_keyframes_for_nominal_);
  ReadVariable(nh, "/orb_test_node/use_fixed_lag", GetInstance().use_fixed_lag_);
  ReadVariable(nh, "/orb_test_node/smoother_lag", GetInstance().smoother_lag_);
  ReadVariable(nh, "/orb_test_node/enable_smart_factors", GetInstance().enable_smart_factors_);

  ReadVariable(nh, "/orb_test_node/isam_relin_thresh/x_rotation", GetInstance().isam_relin_thresh_x_rotation_);
  ReadVariable(nh, "/orb_test_node/isam_relin_thresh/x_translation", GetInstance().isam_relin_thresh_x_translation_);
  ReadVariable(nh, "/orb_test_node/isam_relin_thresh/v", GetInstance().isam_relin_thresh_v_);
  ReadVariable(nh, "/orb_test_node/isam_relin_thresh/b", GetInstance().isam_relin_thresh_b_);
  ReadVariable(nh, "/orb_test_node/isam_relin_thresh/l", GetInstance().isam_relin_thresh_l_);

  ReadVariable(nh, "/orb_test_node/noise_params/prior_X_yaw", GetInstance().prior_noise_X_yaw_);
  ReadVariable(nh, "/orb_test_node/noise_params/prior_X_roll_pitch", GetInstance().prior_noise_X_roll_pitch_);
  ReadVariable(nh, "/orb_test_node/noise_params/prior_X_translation", GetInstance().prior_noise_X_translation_);
  ReadVariable(nh, "/orb_test_node/noise_params/prior_gyro", GetInstance().prior_noise_gyro_);
  ReadVariable(nh, "/orb_test_node/noise_params/prior_accel", GetInstance().prior_noise_accel_);
  ReadVariable(nh, "/orb_test_node/noise_params/prior_velocity", GetInstance().prior_noise_velocity);
  ReadVariable(nh, "/orb_test_node/noise_params/between_rotation", GetInstance().noise_between_rotation_);
  ReadVariable(nh, "/orb_test_node/noise_params/between_translation", GetInstance().noise_between_translation_);
  ReadVariable(nh, "/orb_test_node/noise_params/between_rotation_keyframe",
               GetInstance().noise_between_rotation_keyframe_);
  ReadVariable(nh, "/orb_test_node/noise_params/between_translation_keyframe",
               GetInstance().noise_between_translation_keyframe_);
  ReadVariable(nh, "/orb_test_node/noise_params/feature", GetInstance().noise_feature_);
  ReadVariable(nh, "/orb_test_node/noise_params/range", GetInstance().noise_range_);
  ReadVariable(nh, "/orb_test_node/noise_params/robust_feature_k", GetInstance().robust_feature_k_);
  ReadVariable(nh, "/orb_test_node/noise_params/robust_range_k", GetInstance().robust_range_k_);

  ReadVariable(nh, "/orb_test_node/feature_extraction_interval", GetInstance().feature_extraction_interval_);
  ReadVariable(nh, "/orb_test_node/track_count_lower_thresh", GetInstance().track_count_lower_thresh_);
  ReadVariable(nh, "/orb_test_node/track_nms_squared_dist_thresh", GetInstance().track_nms_squared_dist_thresh_);
  ReadVariable(nh, "/orb_test_node/min_track_length_for_smoothing", GetInstance().min_track_length_for_smoothing_);
  ReadVariable(nh, "/orb_test_node/min_track_length_for_smoothing_depth",
               GetInstance().min_track_length_for_smoothing_depth_);
  ReadVariable(nh, "/orb_test_node/min_depth_measurements_for_smoothing", GetInstance().min_depth_measurements_for_smoothing_);
  ReadVariable(nh, "/orb_test_node/image_edge_padding_percent", GetInstance().image_edge_padding_percent_);
  ReadVariable(nh, "/orb_test_node/stationary_thresh", GetInstance().stationary_thresh_);

  ReadVariable(nh, "/orb_test_node/timeshift_cam_imu", GetInstance().timeshift_cam_imu_);
  ReadVariable(nh, "/orb_test_node/imu_max_messages_retained", GetInstance().imu_max_messages_retained_);
  ReadVariable(nh, "/orb_test_node/imu_g", GetInstance().imu_g_);
  ReadVariable(nh, "/orb_test_node/imu_n_gravity_x", GetInstance().imu_n_gravity_x_);
  ReadVariable(nh, "/orb_test_node/imu_n_gravity_y", GetInstance().imu_n_gravity_y_);
  ReadVariable(nh, "/orb_test_node/imu_n_gravity_z", GetInstance().imu_n_gravity_z_);
  ReadVariable(nh, "/orb_test_node/imu_accel_noise_density", GetInstance().imu_accel_noise_density_);
  ReadVariable(nh, "/orb_test_node/imu_gyro_noise_density", GetInstance().imu_gyro_noise_density_);
  ReadVariable(nh, "/orb_test_node/imu_accel_random_walk", GetInstance().imu_accel_random_walk_);
  ReadVariable(nh, "/orb_test_node/imu_gyro_random_walk", GetInstance().imu_gyro_random_walk_);
  ReadVectorVariable(nh, "/orb_test_node/imu_init_bias_accel", GetInstance().imu_init_bias_accel_);
  ReadVectorVariable(nh, "/orb_test_node/imu_init_bias_gyro", GetInstance().imu_init_bias_gyro_);

  ReadVariable(nh, "/orb_test_node/do_initial_gravity_alignment", GetInstance().do_initial_gravity_alignment_);
  ReadVariable(nh, "/orb_test_node/dynamic_outlier_rejection_threshold",
               GetInstance().dynamic_outlier_rejection_threshold_);
  ReadVariable(nh, "/orb_test_node/landmark_distance_threshold", GetInstance().landmark_distance_threshold_);
  ReadVariable(nh, "/orb_test_node/proj_landmark_init_dist_thresh", GetInstance().proj_landmark_init_dist_thresh_);
  ReadVariable(nh, "/orb_test_node/extra_isam2_update_steps", GetInstance().extra_isam2_update_steps_);
  ReadVariable(nh, "/orb_test_node/triangulation_rank_tol", GetInstance().triangulation_rank_tol_);
  ReadVariable(nh, "/orb_test_node/feature_prediction_outlier_thresh",
               GetInstance().feature_prediction_outlier_thresh_);

  ReadVectorVariable(nh, "/orb_test_node/body_p_cam_quat", GetInstance().body_p_cam_quat_);
  ReadVectorVariable(nh, "/orb_test_node/body_p_cam_vec", GetInstance().body_p_cam_vec_);
  ReadVectorVariable(nh, "/orb_test_node/body_p_imu_quat", GetInstance().body_p_imu_quat_);
  ReadVectorVariable(nh, "/orb_test_node/body_p_imu_vec", GetInstance().body_p_imu_vec_);
  ReadVectorVariable(nh, "/orb_test_node/body_p_lidar_quat", GetInstance().body_p_lidar_quat_);
  ReadVectorVariable(nh, "/orb_test_node/body_p_lidar_vec", GetInstance().body_p_lidar_vec_);

  ReadVariable(nh, "/orb_test_node/image_width", GetInstance().image_width_);
  ReadVariable(nh, "/orb_test_node/image_height", GetInstance().image_height_);
  ReadVariable(nh, "/orb_test_node/cam_fx", GetInstance().cam_fx_);
  ReadVariable(nh, "/orb_test_node/cam_fy", GetInstance().cam_fy_);
  ReadVariable(nh, "/orb_test_node/cam_u0", GetInstance().cam_u0_);
  ReadVariable(nh, "/orb_test_node/cam_v0", GetInstance().cam_v0_);
  ReadVariable(nh, "/orb_test_node/color_image", GetInstance().color_image_);

  ReadVectorVariable(nh, "/orb_test_node/distortion_model/coeffs", GetInstance().distortion_coeffs_);
  ReadVariable(nh, "/orb_test_node/distortion_model/type", GetInstance().distortion_model_);

  ReadVariable(nh, "/orb_test_node/lidar_depth/enabled", GetInstance().lidar_depth_enabled_);
  ReadVariable(nh, "/orb_test_node/lidar_depth/calc_mode", GetInstance().lidar_depth_calc_mode_);
  ReadVariable(nh, "/orb_test_node/lidar_depth/search_window_width", GetInstance().lidar_depth_search_window_width_);
  ReadVariable(nh, "/orb_test_node/lidar_depth/search_window_height", GetInstance().lidar_depth_search_window_height_);
  ReadVariable(nh, "/orb_test_node/lidar_depth/min_non_zero_neighbors",
               GetInstance().lidar_depth_min_non_zero_neighbors_);
  ReadVariable(nh, "/orb_test_node/lidar_depth/max_allowed_feature_distance",
               GetInstance().lidar_depth_max_allowed_feature_distance_);
  ReadVariable(nh, "/orb_test_node/lidar_depth/std_dev_tol_factor", GetInstance().lidar_depth_std_dev_tol_factor_);
  ReadVariable(nh, "/orb_test_node/lidar_depth/max_messages_retained", GetInstance().lidar_max_messages_retained_);

  ReadVariable(nh, "/orb_test_node/loam_between_factors_enabled", GetInstance().loam_between_factors_enabled_);
  ReadVariable(nh, "/orb_test_node/loam_imu_only", GetInstance().loam_imu_only_);
  ReadVariable(nh, "/orb_test_node/frame_between_factors", GetInstance().frame_between_factors_);
  ReadVariable(nh, "/orb_test_node/keyframe_between_factors", GetInstance().keyframe_between_factors_);
  ReadVariable(nh, "/orb_test_node/loam_world_frame", GetInstance().loam_world_frame_);
  ReadVariable(nh, "/orb_test_node/loam_sensor_frame", GetInstance().loam_sensor_frame_);
  ReadVariable(nh, "/orb_test_node/loam_degeneracy_topic", GetInstance().loam_degeneracy_topic_);
  ReadVariable(nh, "/orb_test_node/loam_between_factor_interval", GetInstance().loam_between_factor_interval_);

  ReadVariable(nh, "/orb_test_node/visualization/enabled", GetInstance().visualization_enabled_);
  ReadVariable(nh, "/orb_test_node/visualization/draw_only_in_smoother_landmarks",
               GetInstance().draw_only_in_smoother_landmarks_);
  ReadVariable(nh, "/orb_test_node/visualization/draw_lidar_lines", GetInstance().draw_lidar_lines_);

  ReadVariable(nh, "/orb_test_node/landmark_removal/high_delta", GetInstance().landmark_removal_high_delta_);
  ReadVariable(nh, "/orb_test_node/landmark_removal/high_depth_difference",
               GetInstance().landmark_removal_high_depth_distance_);
  ReadVariable(nh, "/orb_test_node/landmark_removal/max_depth_difference",
               GetInstance().max_depth_difference_before_removal_);
}

// Implement parameter accessors here
std::string GlobalParams::IMUSubTopic()
{
  return GetInstance().imu_sub_topic_;
}
std::string GlobalParams::CameraSubTopic()
{
  return GetInstance().cam_sub_topic_;
}
std::string GlobalParams::LidarSubTopic()
{
  return GetInstance().lidar_sub_topic_;
}
std::string GlobalParams::LidarFrameForPublish()
{
  return GetInstance().lidar_frame_for_publish_;
}

bool GlobalParams::CountFeaturesPerCell()
{
  return GetInstance().count_features_per_cell_;
}
int GlobalParams::MinFeaturesPerCell()
{
  return GetInstance().min_features_per_cell_;
}
int GlobalParams::MaxFeaturesPerCell()
{
  return GetInstance().max_features_per_cell_;
}
int GlobalParams::MaxFeatures()
{
  return GetInstance().max_features_;
}
int GlobalParams::GridCellsX()
{
  return GetInstance().grid_cells_x_;
}
int GlobalParams::GridCellsY()
{
  return GetInstance().grid_cells_y_;
}
double GlobalParams::ResizeFactor()
{
  return GetInstance().resize_factor_;
}

int GlobalParams::KLTMaxIterations()
{
  return GetInstance().klt_max_iterations_;
}
double GlobalParams::KLTConvergenceEpsilon()
{
  return GetInstance().klt_convergence_epsilon_;
}
int GlobalParams::KLTWinSize()
{
  return GetInstance().klt_win_size_;
}
int GlobalParams::KLTPyramids()
{
  return GetInstance().klt_pyramids_;
}

double GlobalParams::FeatureExtractionQualityLevel()
{
  return GetInstance().feature_extraction_quality_level_;
}
double GlobalParams::FeatureExtractionMinDistance()
{
  return GetInstance().feature_extraction_min_distance_;
}
double GlobalParams::FeatureExtractionMinEigenValue()
{
  return GetInstance().feature_extraction_min_eigen_value_;
}

int GlobalParams::LandmarkCullingFrameCount()
{
  return GetInstance().landmark_culling_frame_count_;
}
double GlobalParams::LandmarkCullingObservationPercentage()
{
  return GetInstance().landmark_culling_observation_percentage_;
}
int GlobalParams::LandmarkMatchingWindow()
{
  return GetInstance().landmark_matching_window_;
}
std::string GlobalParams::GroundTruthFile()
{
  return GetInstance().ground_truth_file_;
}
std::string GlobalParams::GroundTruthProvider()
{
  return GetInstance().ground_truth_provider_;
}
std::string GlobalParams::LidarTimeOffsetFile()
{
  return GetInstance().lidar_time_offset_file_;
}
std::string GlobalParams::LidarTimeOffsetType()
{
  return GetInstance().lidar_time_offset_type_;
}
double GlobalParams::LidarFrameManagerTimestampThresh()
{
  return GetInstance().lidar_frame_manager_timestamp_thresh_;
}
bool GlobalParams::InitOnGroundTruth()
{
  return GetInstance().init_on_ground_truth_;
}

int GlobalParams::FeatureExtractionInterval()
{
  return GetInstance().feature_extraction_interval_;
}
// When the number of tracks goes below this threshold, we extract new features.
int GlobalParams::TrackCountLowerThresh()
{
  return GetInstance().track_count_lower_thresh_;
}
// Distance threshold for suppressing tracks in non-max suppression scheme.
// Features closer than this threshold (in squared euclidean distance) to another point will be suppressed
// if the other features has a longer track.
double GlobalParams::TrackNMSSquaredDistThresh()
{
  return GetInstance().track_nms_squared_dist_thresh_;
}
double GlobalParams::StationaryThresh()
{
  return GetInstance().stationary_thresh_;
}

double GlobalParams::TimeshiftCamImu()
{
  return GetInstance().timeshift_cam_imu_;
}
int GlobalParams::IMUMaxMessagesRetained()
{
  return GetInstance().imu_max_messages_retained_;
}

double GlobalParams::IMUAccelNoiseDensity()
{
  return GetInstance().imu_accel_noise_density_;
}
double GlobalParams::IMUGyroNoiseDensity()
{
  return GetInstance().imu_gyro_noise_density_;
}
double GlobalParams::IMUAccelRandomWalk()
{
  return GetInstance().imu_accel_random_walk_;
}
double GlobalParams::IMUGyroRandomWalk()
{
  return GetInstance().imu_gyro_random_walk_;
}
double GlobalParams::IMUG()
{
  return GetInstance().imu_g_;
}
double GlobalParams::IMUNGravityX()
{
  return GetInstance().imu_n_gravity_x_;
}
double GlobalParams::IMUNGravityY()
{
  return GetInstance().imu_n_gravity_y_;
}
double GlobalParams::IMUNGravityZ()
{
  return GetInstance().imu_n_gravity_z_;
}

bool GlobalParams::DoInitialGravityAlignment()
{
  return GetInstance().do_initial_gravity_alignment_;
}
double GlobalParams::DynamicOutlierRejectionThreshold()
{
  return GetInstance().dynamic_outlier_rejection_threshold_;
}
double GlobalParams::LandmarkDistanceThreshold()
{
  return GetInstance().landmark_distance_threshold_;
}
double GlobalParams::ProjLandmarkInitDistanceThresh()
{
  return GetInstance().proj_landmark_init_dist_thresh_;
}
int GlobalParams::ExtraISAM2UpdateSteps()
{
  return GetInstance().extra_isam2_update_steps_;
}

std::vector<double> GlobalParams::BodyPCamQuat()
{
  return GetInstance().body_p_cam_quat_;
}
std::vector<double> GlobalParams::BodyPCamVec()
{
  return GetInstance().body_p_cam_vec_;
}
std::vector<double> GlobalParams::BodyPImuQuat()
{
  return GetInstance().body_p_imu_quat_;
}
std::vector<double> GlobalParams::BodyPImuVec()
{
  return GetInstance().body_p_imu_vec_;
}
std::vector<double> GlobalParams::BodyPLidarQuat()
{
  return GetInstance().body_p_lidar_quat_;
}
std::vector<double> GlobalParams::BodyPLidarVec()
{
  return GetInstance().body_p_lidar_vec_;
}

int GlobalParams::ImageHeight()
{
  return GetInstance().image_height_;
}
int GlobalParams::ImageWidth()
{
  return GetInstance().image_width_;
}
double GlobalParams::CamFx()
{
  return GetInstance().cam_fx_;
}
double GlobalParams::CamFy()
{
  return GetInstance().cam_fy_;
}
double GlobalParams::CamU0()
{
  return GetInstance().cam_u0_;
}
double GlobalParams::CamV0()
{
  return GetInstance().cam_v0_;
}
bool GlobalParams::ColorImage()
{
  return GetInstance().color_image_;
}
std::vector<double> GlobalParams::DistortionCoefficients()
{
  return GetInstance().distortion_coeffs_;
}
std::string GlobalParams::DistortionModel()
{
  return GetInstance().distortion_model_;
}
double GlobalParams::MatchMaxDistance()
{
  return GetInstance().match_max_distance_;
}
int GlobalParams::InitKeyframeInterval()
{
  return GetInstance().init_keyframe_interval_;
}
int GlobalParams::NumGoodKeyframesForInitialization()
{
  return GetInstance().num_good_keyframes_for_initialization_;
}
int GlobalParams::MinTrackLengthForSmoothing()
{
  return GetInstance().min_track_length_for_smoothing_;
}
int GlobalParams::MinTrackLengthForSmoothingDepth()
{
  return GetInstance().min_track_length_for_smoothing_depth_;
}
int GlobalParams::MinDepthMeasurementsForSmoothing()
{
  return GetInstance().min_depth_measurements_for_smoothing_;
}
double GlobalParams::ImageEdgePaddingPercent()
{
  return GetInstance().image_edge_padding_percent_;
}
bool GlobalParams::AddEssentialMatrixConstraints()
{
  return GetInstance().add_essential_matrix_constraints_;
}
double GlobalParams::MinKeyframeFeatureInlierRatio()
{
  return GetInstance().min_keyframe_feature_inlier_ratio_;
}
double GlobalParams::MinParallaxForSmoothing()
{
  return GetInstance().min_parallax_for_smoothing_;
}
double GlobalParams::MinParallaxForSmoothingDepth()
{
  return GetInstance().min_parallax_for_smoothing_depth_;
}
double GlobalParams::NumHighParallaxPointsForKeyframe()
{
  return GetInstance().num_high_parallax_points_for_keyframe_;
}
int GlobalParams::MinActiveTrackCount()
{
  return GetInstance().min_active_track_count_;
}
double GlobalParams::MaxParallaxRotationCompensation()
{
  return GetInstance().max_parallax_rotation_compensation_;
}
bool GlobalParams::UseAngleParallax()
{
  return GetInstance().use_angle_parallax_;
}
double GlobalParams::MaxDepthForSmoothing()
{
  return GetInstance().max_depth_for_smoothing_;
}
double GlobalParams::MinDepthForSmoothing()
{
  return GetInstance().min_depth_for_smoothing_;
}

bool GlobalParams::UseIsam()
{
  return GetInstance().use_isam_;
}
double GlobalParams::IsamRelinearizeThresh()
{
  return GetInstance().isam_relinearize_thresh_;
}
bool GlobalParams::SaveFactorGraphsToFile()
{
  return GetInstance().save_factor_graphs_to_file_;
}
double GlobalParams::InitRangeFactorLength()
{
  return GetInstance().init_range_factor_length_;
}
bool GlobalParams::UseDogLeg()
{
  return GetInstance().use_dogleg_;
}
int GlobalParams::MinKeyframesForNominal()
{
  return GetInstance().min_keyframes_for_nominal_;
}
bool GlobalParams::UseFixedLag()
{
  return GetInstance().use_fixed_lag_;
}
double GlobalParams::SmootherLag()
{
  return GetInstance().smoother_lag_;
}
bool GlobalParams::EnableSmartFactors()
{
  return GetInstance().enable_smart_factors_;
}

double GlobalParams::MinParallaxForKeyframe()
{
  return GetInstance().min_parallax_for_keyframe_;
}
bool GlobalParams::UseParallaxKeyframes()
{
  return GetInstance().use_parallax_keyframes_;
}
int GlobalParams::TemporalKeyframeInterval()
{
  return GetInstance().temporal_keyframe_interval_;
}
int GlobalParams::FrameInterval()
{
  return GetInstance().frame_interval_;
}
int GlobalParams::SecondRANSACNFrames()
{
  return GetInstance().second_ransac_n_frames_;
}

double GlobalParams::IsamRelinThreshXRotation()
{
  return GetInstance().isam_relin_thresh_x_rotation_;
}
double GlobalParams::IsamRelinThreshXTranslation()
{
  return GetInstance().isam_relin_thresh_x_translation_;
}
double GlobalParams::IsamRelinThreshV()
{
  return GetInstance().isam_relin_thresh_v_;
}
double GlobalParams::IsamRelinThreshB()
{
  return GetInstance().isam_relin_thresh_b_;
}
double GlobalParams::IsamRelinThreshL()
{
  return GetInstance().isam_relin_thresh_l_;
}

double GlobalParams::PriorNoiseXYaw()
{
  return GetInstance().prior_noise_X_yaw_;
}
double GlobalParams::PriorNoiseXRollPitch()
{
  return GetInstance().prior_noise_X_roll_pitch_;
}
double GlobalParams::PriorNoiseXTranslation()
{
  return GetInstance().prior_noise_X_translation_;
}
double GlobalParams::PriorNoiseGyro()
{
  return GetInstance().prior_noise_gyro_;
}
double GlobalParams::PriorNoiseAccel()
{
  return GetInstance().prior_noise_accel_;
}
double GlobalParams::PriorNoiseVelocity()
{
  return GetInstance().prior_noise_velocity;
}
double GlobalParams::NoiseBetweenRotation()
{
  return GetInstance().noise_between_rotation_;
}
double GlobalParams::NoiseBetweenTranslation()
{
  return GetInstance().noise_between_translation_;
}
double GlobalParams::NoiseBetweenRotationKeyframe()
{
  return GetInstance().noise_between_rotation_keyframe_;
}
double GlobalParams::NoiseBetweenTranslationKeyframe()
{
  return GetInstance().noise_between_translation_keyframe_;
}
double GlobalParams::NoiseFeature()
{
  return GetInstance().noise_feature_;
}
double GlobalParams::NoiseRange()
{
  return GetInstance().noise_range_;
}
double GlobalParams::RobustFeatureK()
{
  return GetInstance().robust_feature_k_;
}
double GlobalParams::RobustRangeK()
{
  return GetInstance().robust_range_k_;
}
bool GlobalParams::LidarDepthEnabled()
{
  return GetInstance().lidar_depth_enabled_;
}
int GlobalParams::LidarDepthCalcMode()
{
  return GetInstance().lidar_depth_calc_mode_;
}
int GlobalParams::LidarDepthSearchWindowWidth()
{
  return GetInstance().lidar_depth_search_window_width_;
}
int GlobalParams::LidarDepthSearchWindowHeight()
{
  return GetInstance().lidar_depth_search_window_height_;
}
int GlobalParams::LidarDepthMinNonZeroNeighbors()
{
  return GetInstance().lidar_depth_min_non_zero_neighbors_;
}
double GlobalParams::LidarDepthMaxAllowedFeatureDistance()
{
  return GetInstance().lidar_depth_max_allowed_feature_distance_;
}
double GlobalParams::LidarDepthStdDevTolFactor()
{
  return GetInstance().lidar_depth_std_dev_tol_factor_;
}
int GlobalParams::LidarMaxMessagesRetained()
{
  return GetInstance().lidar_max_messages_retained_;
}

int GlobalParams::IsamRelinearizeSkip()
{
  return GetInstance().isam_relinearize_skip_;
}

bool GlobalParams::LoamBetweenFactorsEnabled()
{
  return GetInstance().loam_between_factors_enabled_;
}
bool GlobalParams::LoamIMUOnly()
{
  return GetInstance().loam_imu_only_;
}
bool GlobalParams::FrameBetweenFactors()
{
  return GetInstance().frame_between_factors_;
}
bool GlobalParams::KeyframeBetweenFactors()
{
  return GetInstance().keyframe_between_factors_;
}
std::string GlobalParams::LoamWorldFrame()
{
  return GetInstance().loam_world_frame_;
}
std::string GlobalParams::LoamSensorFrame()
{
  return GetInstance().loam_sensor_frame_;
}
std::string GlobalParams::LoamDegeneracySubTopic()
{
  return GetInstance().loam_degeneracy_topic_;
}
int GlobalParams::LoamBetweenFactorInterval()
{
  return GetInstance().loam_between_factor_interval_;
}

bool GlobalParams::DrawLidarLines()
{
  return GetInstance().draw_lidar_lines_;
}

bool GlobalParams::LandmarkRemovalHighDelta()
{
  return GetInstance().landmark_removal_high_delta_;
}
bool GlobalParams::LandmarkRemovalHighDepthDifference()
{
  return GetInstance().landmark_removal_high_depth_distance_;
}
double GlobalParams::MaxDepthDifferenceBeforeRemoval()
{
  return GetInstance().max_depth_difference_before_removal_;
}
std::vector<double> GlobalParams::IMUInitBiasAccel()
{
  return GetInstance().imu_init_bias_accel_;
}
std::vector<double> GlobalParams::IMUInitBiasGyro()
{
  return GetInstance().imu_init_bias_gyro_;
}
std::string GlobalParams::FullPathExportFilePrefix()
{
  return GetInstance().full_path_export_file_prefix_;
}
double GlobalParams::TriangulationRankTol()
{
  return GetInstance().triangulation_rank_tol_;
}
bool GlobalParams::VisualizationEnabled()
{
  return GetInstance().visualization_enabled_;
}
bool GlobalParams::DrawOnlyInSmootherLandmarks()
{
  return GetInstance().draw_only_in_smoother_landmarks_;
}
double GlobalParams::FeaturePredictionOutlierThresh()
{
  return GetInstance().feature_prediction_outlier_thresh_;
}
bool GlobalParams::PreProcess()
{
  return GetInstance().pre_process_;
}
double GlobalParams::ContrastAlpha()
{
  return GetInstance().contrast_alpha_;
}
double GlobalParams::ContrastBeta()
{
  return GetInstance().contrast_beta_;
}
