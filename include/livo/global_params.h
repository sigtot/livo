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
  double track_nms_squared_dist_thresh_ = 16.;
  int min_track_length_for_smoothing_ = 15;
  double image_edge_padding_percent_ = 0.05;
  double stationary_thresh_ = 0.5;
  int init_keyframe_interval_ = 6;
  int num_good_keyframes_for_initialization_ = 20;

  double timeshift_cam_imu_ = 0.01379378638037798;

  // For explanation and units, see https://github.com/borglab/gtsam/issues/213
  double imu_g_ = 9.81000;
  double imu_n_gravity_x_ = 0.;
  double imu_n_gravity_y_ = 0.;
  double imu_n_gravity_z_ = -9.81;
  double imu_accel_noise_density_ = 0.1;
  double imu_gyro_noise_density_ = 0.1;
  double imu_accel_random_walk_ = 0.01;
  double imu_gyro_random_walk_ = 0.01;

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
  static int InitKeyframeInterval();
  static int NumGoodKeyframesForInitialization();

  static int FeatureExtractionInterval();
  static int TrackCountLowerThresh();
  static double TrackNMSSquaredDistThresh();
  static int MinTrackLengthForSmoothing();
  static double ImageEdgePaddingPercent();
  static double StationaryThresh();

  static double TimeshiftCamImu(); // t_imu = t_cam + shift
  static double IMUG();
  static double IMUNGravityX();
  static double IMUNGravityY();
  static double IMUNGravityZ();
  static double IMUAccelNoiseDensity();
  static double IMUGyroNoiseDensity();
  static double IMUAccelRandomWalk();
  static double IMUGyroRandomWalk();

  static double CamFx();
  static double CamFy();
  static double CamU0();
  static double CamV0();
};

#endif
