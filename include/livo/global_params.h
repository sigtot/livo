#ifndef ORB_TEST__GLOBAL_PARAMS_H_
#define ORB_TEST__GLOBAL_PARAMS_H_

#include <ros/ros.h>

class GlobalParams
{
private:
  GlobalParams() = default;
  static GlobalParams& GetInstance();
  template <class T>
  static void ReadVariable(const ros::NodeHandle& nh, const std::string& variable_name, T& variable);

  // Add parameters here
  int max_features_per_cell_ = 10;
  double resize_factor_ = 1.f;
  int landmark_culling_frame_count_ = 20;
  double landmark_culling_observation_percentage_ = .40;
  int landmark_matching_window_ = 5;

public:
  static void LoadParams(const ros::NodeHandle& nh);
  GlobalParams(GlobalParams const&) = delete;
  void operator=(GlobalParams const&) = delete;

  // Add parameter accessors here
  static int MaxFeaturesPerCell();
  static double ResizeFactor();
  static int LandmarkCullingFrameCount();
  static double LandmarkCullingObservationPercentage();
  static int LandmarkMatchingWindow();
};

#endif
