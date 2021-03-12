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
  ReadVariable(nh, "/orb_test_node/feature_extraction_interval", GetInstance().feature_extraction_interval_);
  ReadVariable(nh, "/orb_test_node/track_count_lower_thresh", GetInstance().track_count_lower_thresh_);
  ReadVariable(nh, "/orb_test_node/track_nms_squared_dist_thresh", GetInstance().track_nms_squared_dist_thresh_);

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
