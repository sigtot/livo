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
    std::cout << "WARN: Could not read param " << variable_name
              << " from parameter server, so using "
                 "default value "
              << variable << std::endl;
  }
  else
  {
    std::cout << "Read param " << variable_name << " from parameter server: " << variable << std::endl;
  }
}

void GlobalParams::LoadParams(const ros::NodeHandle& nh)
{
  // Add ReadVariable calls here
  ReadVariable(nh, "max_features_per_cell", GetInstance().max_features_per_cell_);
  ReadVariable(nh, "resize_factor", GetInstance().resize_factor_);
  ReadVariable(nh, "landmark_culling_frame_count", GetInstance().landmark_culling_frame_count_);
  ReadVariable(nh, "landmark_culling_observation_percentage", GetInstance().landmark_culling_observation_percentage_);
  ReadVariable(nh, "landmark_matching_window", GetInstance().landmark_matching_window_);
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
