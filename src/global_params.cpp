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

  ReadVariable(nh, "/orb_test_node/max_features_per_cell", GetInstance().max_features_per_cell_);
  ReadVariable(nh, "/orb_test_node/resize_factor", GetInstance().resize_factor_);
  ReadVariable(nh, "/orb_test_node/landmark_culling_frame_count", GetInstance().landmark_culling_frame_count_);
  ReadVariable(nh, "/orb_test_node/landmark_culling_observation_percentage",
               GetInstance().landmark_culling_observation_percentage_);
  ReadVariable(nh, "/orb_test_node/landmark_matching_window", GetInstance().landmark_matching_window_);
  ReadVariable(nh, "/orb_test_node/ground_truth_file", GetInstance().ground_truth_file_);
  ReadVariable(nh, "/orb_test_node/ground_truth_provider", GetInstance().ground_truth_provider_);
  ReadVariable(nh, "/orb_test_node/init_on_ground_truth", GetInstance().init_on_ground_truth_);
  ReadVariable(nh, "/orb_test_node/match_max_distance", GetInstance().match_max_distance_);
  ReadVariable(nh, "/orb_test_node/init_keyframe_interval", GetInstance().init_keyframe_interval_);
  ReadVariable(nh, "/orb_test_node/num_good_keyframes_for_initialization",
               GetInstance().num_good_keyframes_for_initialization_);
  ReadVariable(nh, "/orb_test_node/add_essential_matrix_constraints", GetInstance().add_essential_matrix_constraints_);
  ReadVariable(nh, "/orb_test_node/min_keyframe_feature_inlier_ratio",
               GetInstance().min_keyframe_feature_inlier_ratio_);
  ReadVariable(nh, "/orb_test_node/min_parallax_for_keyframe", GetInstance().min_parallax_for_keyframe_);
  ReadVariable(nh, "/orb_test_node/min_parallax_for_smoothing", GetInstance().min_parallax_for_smoothing_);
  ReadVariable(nh, "/orb_test_node/max_parallax_rotation_compensation",
               GetInstance().max_parallax_rotation_compensation_);
  ReadVariable(nh, "/orb_test_node/num_high_parallax_points_for_keyframe",
               GetInstance().num_high_parallax_points_for_keyframe_);

  ReadVariable(nh, "/orb_test_node/use_isam", GetInstance().use_isam_);
  ReadVariable(nh, "/orb_test_node/use_dogleg", GetInstance().use_dogleg_);
  ReadVariable(nh, "/orb_test_node/isam_relinearize_thresh", GetInstance().isam_relinearize_thresh_);
  ReadVariable(nh, "/orb_test_node/save_factor_graphs_to_file", GetInstance().save_factor_graphs_to_file_);
  ReadVariable(nh, "/orb_test_node/init_range_factor_length", GetInstance().init_range_factor_length_);
  ReadVariable(nh, "/orb_test_node/min_keyframes_for_nominal", GetInstance().min_keyframes_for_nominal_);

  ReadVariable(nh, "/orb_test_node/noise_params/prior_X_yaw", GetInstance().prior_noise_X_yaw_);
  ReadVariable(nh, "/orb_test_node/noise_params/prior_X_roll_pitch", GetInstance().prior_noise_X_roll_pitch_);
  ReadVariable(nh, "/orb_test_node/noise_params/prior_X_translation", GetInstance().prior_noise_X_translation_);
  ReadVariable(nh, "/orb_test_node/noise_params/prior_gyro", GetInstance().prior_noise_gyro_);
  ReadVariable(nh, "/orb_test_node/noise_params/prior_accel", GetInstance().prior_noise_accel_);
  ReadVariable(nh, "/orb_test_node/noise_params/prior_velocity", GetInstance().prior_noise_velocity);
  ReadVariable(nh, "/orb_test_node/noise_params/feature", GetInstance().noise_feature_);

  ReadVariable(nh, "/orb_test_node/feature_extraction_interval", GetInstance().feature_extraction_interval_);
  ReadVariable(nh, "/orb_test_node/track_count_lower_thresh", GetInstance().track_count_lower_thresh_);
  ReadVariable(nh, "/orb_test_node/track_nms_squared_dist_thresh", GetInstance().track_nms_squared_dist_thresh_);
  ReadVariable(nh, "/orb_test_node/min_track_length_for_smoothing", GetInstance().min_track_length_for_smoothing_);
  ReadVariable(nh, "/orb_test_node/image_edge_padding_percent", GetInstance().image_edge_padding_percent_);
  ReadVariable(nh, "/orb_test_node/stationary_thresh", GetInstance().stationary_thresh_);

  ReadVariable(nh, "/orb_test_node/timeshift_cam_imu", GetInstance().timeshift_cam_imu_);
  ReadVariable(nh, "/orb_test_node/imu_g", GetInstance().imu_g_);
  ReadVariable(nh, "/orb_test_node/imu_n_gravity_x", GetInstance().imu_n_gravity_x_);
  ReadVariable(nh, "/orb_test_node/imu_n_gravity_y", GetInstance().imu_n_gravity_y_);
  ReadVariable(nh, "/orb_test_node/imu_n_gravity_z", GetInstance().imu_n_gravity_z_);
  ReadVariable(nh, "/orb_test_node/imu_accel_noise_density", GetInstance().imu_accel_noise_density_);
  ReadVariable(nh, "/orb_test_node/imu_gyro_noise_density", GetInstance().imu_gyro_noise_density_);
  ReadVariable(nh, "/orb_test_node/imu_accel_random_walk", GetInstance().imu_accel_random_walk_);
  ReadVariable(nh, "/orb_test_node/imu_gyro_random_walk", GetInstance().imu_gyro_random_walk_);

  ReadVariable(nh, "/orb_test_node/do_initial_gravity_alignment", GetInstance().do_initial_gravity_alignment_);
  ReadVariable(nh, "/orb_test_node/dynamic_outlier_rejection_threshold",
               GetInstance().dynamic_outlier_rejection_threshold_);

  ReadVectorVariable(nh, "/orb_test_node/body_p_cam_quat", GetInstance().body_p_cam_quat_);
  ReadVectorVariable(nh, "/orb_test_node/body_p_cam_vec", GetInstance().body_p_cam_vec_);
  ReadVectorVariable(nh, "/orb_test_node/body_p_imu_quat", GetInstance().body_p_imu_quat_);
  ReadVectorVariable(nh, "/orb_test_node/body_p_imu_vec", GetInstance().body_p_imu_vec_);
  ReadVectorVariable(nh, "/orb_test_node/body_p_lidar_quat", GetInstance().body_p_lidar_quat_);
  ReadVectorVariable(nh, "/orb_test_node/body_p_lidar_vec", GetInstance().body_p_lidar_vec_);

  ReadVariable(nh, "/orb_test_node/cam_fx", GetInstance().cam_fx_);
  ReadVariable(nh, "/orb_test_node/cam_fy", GetInstance().cam_fy_);
  ReadVariable(nh, "/orb_test_node/cam_u0", GetInstance().cam_u0_);
  ReadVariable(nh, "/orb_test_node/cam_v0", GetInstance().cam_v0_);

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
int GlobalParams::MaxFeaturesPerCell()
{
  return GetInstance().max_features_per_cell_;
}
double GlobalParams::ResizeFactor()
{
  return GetInstance().resize_factor_;
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
double GlobalParams::MinParallaxForKeyframe()
{
  return GetInstance().min_parallax_for_keyframe_;
}
double GlobalParams::MinParallaxForSmoothing()
{
  return GetInstance().min_parallax_for_smoothing_;
}
double GlobalParams::NumHighParallaxPointsForKeyframe()
{
  return GetInstance().num_high_parallax_points_for_keyframe_;
}
double GlobalParams::MaxParallaxRotationCompensation()
{
  return GetInstance().max_parallax_rotation_compensation_;
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
double GlobalParams::NoiseFeature()
{
  return GetInstance().noise_feature_;
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
