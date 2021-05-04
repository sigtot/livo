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
  template <class T>
  static void ReadVectorVariable(const ros::NodeHandle& nh, const std::string& variable_name, std::vector<T>& variable);

  // Add parameters here
  std::string imu_sub_topic_ = "/imu";
  std::string cam_sub_topic_ = "/camera";
  std::string lidar_sub_topic_ = "/os1_cloud_node/points";

  int max_features_per_cell_ = 10;
  double resize_factor_ = 1.f;
  int landmark_culling_frame_count_ = 20;
  double landmark_culling_observation_percentage_ = .40;
  int landmark_matching_window_ = 5;
  std::string ground_truth_file_ = "/path/to/registered_poses.csv";
  std::string ground_truth_provider_ = "newer_college";  // Possible values: newer_college, euroc
  bool init_on_ground_truth_ = false;
  double match_max_distance_ = 20;
  double min_parallax_for_keyframe_ =
      10.;  // Need n points with higher than parallax than this to insert a new keyframe
  double min_parallax_for_smoothing_ = 5.;  // Points need higher parallax than this to be added to the smoother
  double max_parallax_rotation_compensation_ = 30.;
  int num_high_parallax_points_for_keyframe_ = 20;

  int feature_extraction_interval_ = 5;
  int track_count_lower_thresh_ = 100;
  double track_nms_squared_dist_thresh_ = 16.;
  int min_track_length_for_smoothing_ = 15;
  double image_edge_padding_percent_ = 0.05;
  double stationary_thresh_ = 0.5;
  int init_keyframe_interval_ = 6;
  int num_good_keyframes_for_initialization_ = 20;
  bool add_essential_matrix_constraints_ = false;
  double min_keyframe_feature_inlier_ratio_ = .75;

  bool use_isam_ = true;
  bool use_dogleg_ = false;  // Only applies when using ISAM2. true = DogLeg, false = GN
  double isam_relinearize_thresh_ = 0.1;
  bool save_factor_graphs_to_file_ = false;
  double init_range_factor_length_ = 10.;  // Give negative value to use value calculated from estimate
  int min_keyframes_for_nominal_ = 20;

  double prior_noise_X_yaw_ = 0.001;  // Low variance for yaw since it is unobservable
  double prior_noise_X_roll_pitch_ = 0.1;
  double prior_noise_X_translation_ = 0.01;
  double prior_noise_gyro_ = 0.1;
  double prior_noise_accel_ = 0.1;
  double prior_noise_velocity = 0.1;
  double noise_feature_ = 1.0;
  double noise_range_ = 0.5;  // meters

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

  bool do_initial_gravity_alignment_ = false;
  double dynamic_outlier_rejection_threshold_ = 8.0;

  std::vector<double> body_p_cam_quat_ = { 0., 0., 0., 1. };
  std::vector<double> body_p_cam_vec_ = { 0., 0., 0. };

  std::vector<double> body_p_imu_quat_ = { 0., 0., 0., 1. };
  std::vector<double> body_p_imu_vec_ = { 0., 0., 0. };

  std::vector<double> body_p_lidar_quat_ = { 0., 0., 0., 1. };
  std::vector<double> body_p_lidar_vec_ = { 0., 0., 0.};

  int image_width_ = 480;
  int image_height_ = 848;
  double cam_fx_ = 431.38739114;
  double cam_fy_ = 430.24961762;
  double cam_u0_ = 427.4407802;
  double cam_v0_ = 238.52694868;

  bool lidar_depth_enabled_ = true;
  int lidar_depth_calc_mode_ = 0; // 0 = patch, 1 = line
  int lidar_depth_search_window_width_ = 7;
  int lidar_depth_search_window_height_ = 7;
  int lidar_depth_min_non_zero_neighbors_ = 3;
  double lidar_depth_max_allowed_feature_distance_ = 20.;


  std::vector<double> distortion_coeffs_ = { 0., 0., 0., 0. };
  std::string distortion_model_ = "radtan";

public:
  static void LoadParams(const ros::NodeHandle& nh);
  GlobalParams(GlobalParams const&) = delete;
  void operator=(GlobalParams const&) = delete;

  // Add parameter accessors here
  static std::string IMUSubTopic();
  static std::string CameraSubTopic();
  static std::string LidarSubTopic();

  static int MaxFeaturesPerCell();
  static double ResizeFactor();
  static int LandmarkCullingFrameCount();
  static double LandmarkCullingObservationPercentage();
  static int LandmarkMatchingWindow();
  static std::string GroundTruthFile();
  static std::string GroundTruthProvider();
  static bool InitOnGroundTruth();
  static double MatchMaxDistance();
  static int InitKeyframeInterval();
  static int NumGoodKeyframesForInitialization();
  static bool AddEssentialMatrixConstraints();
  static double MinKeyframeFeatureInlierRatio();
  static double MinParallaxForKeyframe();
  static double MinParallaxForSmoothing();
  static double MaxParallaxRotationCompensation();
  static double NumHighParallaxPointsForKeyframe();

  static bool UseIsam();
  static bool UseDogLeg();
  static double IsamRelinearizeThresh();
  static bool SaveFactorGraphsToFile();
  static double InitRangeFactorLength();
  static int MinKeyframesForNominal();

  static int FeatureExtractionInterval();
  static int TrackCountLowerThresh();
  static double TrackNMSSquaredDistThresh();
  static int MinTrackLengthForSmoothing();
  static double ImageEdgePaddingPercent();
  static double StationaryThresh();

  static double PriorNoiseXYaw();
  static double PriorNoiseXRollPitch();
  static double PriorNoiseXTranslation();
  static double PriorNoiseGyro();
  static double PriorNoiseAccel();
  static double PriorNoiseVelocity();
  static double NoiseFeature();
  static double NoiseRange();

  static double TimeshiftCamImu();  // t_imu = t_cam + shift
  static double IMUG();
  static double IMUNGravityX();
  static double IMUNGravityY();
  static double IMUNGravityZ();
  static double IMUAccelNoiseDensity();
  static double IMUGyroNoiseDensity();
  static double IMUAccelRandomWalk();
  static double IMUGyroRandomWalk();

  static bool DoInitialGravityAlignment();
  static double DynamicOutlierRejectionThreshold();

  static std::vector<double> BodyPCamQuat();
  static std::vector<double> BodyPCamVec();

  static std::vector<double> BodyPImuQuat();
  static std::vector<double> BodyPImuVec();

  static std::vector<double> BodyPLidarQuat();
  static std::vector<double> BodyPLidarVec();

  static int ImageHeight();
  static int ImageWidth();
  static double CamFx();
  static double CamFy();
  static double CamU0();
  static double CamV0();

  static std::vector<double> DistortionCoefficients();
  static std::string DistortionModel();

  static bool LidarDepthEnabled();
  static int LidarDepthCalcMode();
  static int LidarDepthSearchWindowWidth();
  static int LidarDepthSearchWindowHeight();
  static int LidarDepthMinNonZeroNeighbors();
  static double LidarDepthMaxAllowedFeatureDistance();
};

#endif
