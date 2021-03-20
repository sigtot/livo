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

void GlobalParams::LoadParams(const ros::NodeHandle& nh)
{
  // Add ReadVariable calls here
  ReadVariable(nh, "/orb_test_node/max_features_per_cell", GetInstance().max_features_per_cell_);
  ReadVariable(nh, "/orb_test_node/resize_factor", GetInstance().resize_factor_);
  ReadVariable(nh, "/orb_test_node/landmark_culling_frame_count", GetInstance().landmark_culling_frame_count_);
  ReadVariable(nh, "/orb_test_node/landmark_culling_observation_percentage",
               GetInstance().landmark_culling_observation_percentage_);
  ReadVariable(nh, "/orb_test_node/landmark_matching_window", GetInstance().landmark_matching_window_);
  ReadVariable(nh, "/orb_test_node/ground_truth_file", GetInstance().ground_truth_file_);
  ReadVariable(nh, "/orb_test_node/match_max_distance", GetInstance().match_max_distance_);
  ReadVariable(nh, "/orb_test_node/init_keyframe_interval", GetInstance().init_keyframe_interval_);
  ReadVariable(nh, "/orb_test_node/num_good_keyframes_for_initialization",
               GetInstance().num_good_keyframes_for_initialization_);

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

  ReadVariable(nh, "/orb_test_node/cam_fx", GetInstance().cam_fx_);
  ReadVariable(nh, "/orb_test_node/cam_fy", GetInstance().cam_fy_);
  ReadVariable(nh, "/orb_test_node/cam_u0", GetInstance().cam_u0_);
  ReadVariable(nh, "/orb_test_node/cam_v0", GetInstance().cam_v0_);
}

// Implement parameter accessors here
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
