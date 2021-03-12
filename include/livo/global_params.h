#ifndef ORB_TEST__GLOBAL_PARAMS_H_
#define ORB_TEST__GLOBAL_PARAMS_H_

#include <ros/ros.h>
#include <string>

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
  std::string ground_truth_file_ = "/path/to/registered_poses.csv";
  double match_max_distance_ = 20;

  int feature_extraction_interval_ = 5;
  int track_count_lower_thresh_ = 100;

  double cam_fx_ = 431.38739114;
  double cam_fy_ = 430.24961762;
  double cam_u0_ = 427.4407802;
  double cam_v0_ = 238.52694868;

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
  static std::string GroundTruthFile();
  static double MatchMaxDistance();

  static int FeatureExtractionInterval();
  static int TrackCountLowerThresh();

  static double CamFx();
  static double CamFy();
  static double CamU0();
  static double CamV0();
};

#endif
