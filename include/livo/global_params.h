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

  std::string full_path_export_file_prefix_ = "/tmp/livo_full_path_";  // Will be concatenated with current ISO-time

  // If true, will extract new features per cell when the per-cell count falls below min_features_per_cell_
  // If false, will instead extract new features when the total count falls below track_count_lower_thresh_
  bool count_features_per_cell_ = false;
  int min_features_per_cell_ = 3;  // Note: only used if count_features_per_cell_ is true
  int max_features_per_cell_ = 10;
  int feature_extraction_interval_ = 5;
  int track_count_lower_thresh_ = 100;
  double track_nms_squared_dist_thresh_ = 16.;
  int max_features_ = 100;
  int grid_cells_x_ = 9;
  int grid_cells_y_ = 7;
  double resize_factor_ = 1.f;

  int klt_max_iterations_ = 10;
  double klt_convergence_epsilon_ = 0.03;
  int klt_win_size_ = 15;
  int klt_pyramids_ = 2;

  double feature_extraction_quality_level_ = 0.3; // quality_level param for cv::goodFeaturesToTrack
  double feature_extraction_min_distance_ = 7; // min_distance param for cv::goodFeaturesToTrack
  double feature_extraction_min_eigen_value_ = 1e-4; // all feature eigen values must be at least larger than this value

  int landmark_culling_frame_count_ = 20;
  double landmark_culling_observation_percentage_ = .40;
  int landmark_matching_window_ = 5;
  std::string ground_truth_file_ = "/path/to/registered_poses.csv";
  std::string ground_truth_provider_ = "newer_college";  // Possible values: newer_college, euroc
  std::string lidar_time_offset_file_ = "/path/to/time_offsets.csv";
  std::string lidar_time_offset_type_ = "newer_college";  // Possible values: newer_college, zero
  double lidar_frame_manager_timestamp_thresh_ = 0.05;
  bool init_on_ground_truth_ = false;
  double match_max_distance_ = 20;
  double min_parallax_for_smoothing_ = 5.;        // Points need higher parallax than this to be added to the smoother
  double min_parallax_for_smoothing_depth_ = 5.;  // Depth features need higher parallax than this for smoothing
  double max_parallax_rotation_compensation_ = 30.;
  int num_high_parallax_points_for_keyframe_ = 20;
  bool use_angle_parallax_ = true;  // true: use angle parallax as in ORB-SLAM, false: pixel parallax as in VINS-mono

  bool landmark_removal_high_delta_ = true;
  bool landmark_removal_high_depth_distance_ = true;
  double max_depth_difference_before_removal_ = 10.;

  int second_ransac_n_frames_ = 30; // Perform a second fundamentalMat RANSAC outlier rejection with the nth prev frame

  bool use_parallax_keyframes_ = false;  // false: temporal keyframes, true: insert keyframes when enough parallax
  // If parallax keyframes: Need n points with higher than parallax than this to insert a new keyframe
  double min_parallax_for_keyframe_ = 10.;
  // If temporal keyframes: Initialize a keyframe every n frames
  int temporal_keyframe_interval_ = 60;
  int frame_interval_ = 1;  // Initialize a frame in the backend every n frames

  int min_track_length_for_smoothing_ = 15;
  int min_track_length_for_smoothing_depth_ = 15.;
  int min_depth_measurements_for_smoothing_ = 3;  // Need more than n depth measurements to add as depth feat
  double image_edge_padding_percent_ = 0.05;
  double stationary_thresh_ = 0.5;
  int init_keyframe_interval_ = 6;
  int num_good_keyframes_for_initialization_ = 20;
  bool add_essential_matrix_constraints_ = false;
  double min_keyframe_feature_inlier_ratio_ = .75;
  int min_active_track_count_ = 20;

  bool use_isam_ = true;
  bool use_dogleg_ = false;  // Only applies when using ISAM2. true = DogLeg, false = GN
  double isam_relinearize_thresh_ = 0.1;
  int isam_relinearize_skip_ = 10;
  bool save_factor_graphs_to_file_ = false;
  double init_range_factor_length_ = 10.;  // Give negative value to use value calculated from estimate
  int min_keyframes_for_nominal_ = 20;
  bool use_fixed_lag_ = true;         // False: ISAM2 without any marginalization, true: IncrementalFixedLagSmoother
  double smoother_lag_ = 10.;         // Seconds
  bool enable_smart_factors_ = true;  // False: Proj factors for all lmks, true: smart factors for range-less lmks

  double isam_relin_thresh_x_rotation_ = 0.1;
  double isam_relin_thresh_x_translation_ = 0.5;
  double isam_relin_thresh_v_ = 0.1;
  double isam_relin_thresh_b_ = 0.1;
  double isam_relin_thresh_l_ = 1.0;

  double prior_noise_X_yaw_ = 0.001;  // Low variance for yaw since it is unobservable
  double prior_noise_X_roll_pitch_ = 0.1;
  double prior_noise_X_translation_ = 0.01;
  double prior_noise_gyro_ = 0.1;
  double prior_noise_accel_ = 0.1;
  double prior_noise_velocity = 0.1;
  double noise_between_rotation_ = 0.001;
  double noise_between_translation_ = 0.005;
  double noise_between_rotation_keyframe_ = 0.1;
  double noise_between_translation_keyframe_ = 0.2;
  double noise_feature_ = 1.0;
  double noise_range_ = 0.5;  // meters
  double robust_feature_k_ = 15;
  double robust_range_k_ = 15;

  double timeshift_cam_imu_ = 0.01379378638037798;
  int imu_max_messages_retained_ = 10000;

  // For explanation and units, see https://github.com/borglab/gtsam/issues/213
  double imu_g_ = 9.81000;
  double imu_n_gravity_x_ = 0.;
  double imu_n_gravity_y_ = 0.;
  double imu_n_gravity_z_ = -9.81;
  double imu_accel_noise_density_ = 0.1;
  double imu_gyro_noise_density_ = 0.1;
  double imu_accel_random_walk_ = 0.01;
  double imu_gyro_random_walk_ = 0.01;
  std::vector<double> imu_init_bias_accel_ = std::vector<double> {0., 0., 0.};
  std::vector<double> imu_init_bias_gyro_ = std::vector<double> {0., 0., 0.};

  bool do_initial_gravity_alignment_ = false;
  double dynamic_outlier_rejection_threshold_ = 8.0;
  double landmark_distance_threshold_ = 15.;    // Smart factors with larger distance than this are marked degenerate
  double proj_landmark_init_dist_thresh_ = 6.;  // Triangulated proj factors are only added if closer than this dist
  int extra_isam2_update_steps_ = 0;
  double triangulation_rank_tol_ = 0.1;

  std::vector<double> body_p_cam_quat_ = { 0., 0., 0., 1. };
  std::vector<double> body_p_cam_vec_ = { 0., 0., 0. };

  std::vector<double> body_p_imu_quat_ = { 0., 0., 0., 1. };
  std::vector<double> body_p_imu_vec_ = { 0., 0., 0. };

  std::vector<double> body_p_lidar_quat_ = { 0., 0., 0., 1. };
  std::vector<double> body_p_lidar_vec_ = { 0., 0., 0. };

  int image_width_ = 480;
  int image_height_ = 848;
  double cam_fx_ = 431.38739114;
  double cam_fy_ = 430.24961762;
  double cam_u0_ = 427.4407802;
  double cam_v0_ = 238.52694868;
  bool color_image_ = false;  // False: 8UC1, True: 8UC3

  bool lidar_depth_enabled_ = true;
  int lidar_depth_calc_mode_ = 0;  // 0 = patch, 1 = line
  int lidar_depth_search_window_width_ = 7;
  int lidar_depth_search_window_height_ = 7;
  int lidar_depth_min_non_zero_neighbors_ = 3;
  double lidar_depth_max_allowed_feature_distance_ = 20.;
  double lidar_depth_std_dev_tol_factor_ = 0.1;  // Depth rejected if stddev > tol_factor * depth
  int lidar_max_messages_retained_ = 100;

  std::vector<double> distortion_coeffs_ = { 0., 0., 0., 0. };
  std::string distortion_model_ = "radtan";

  bool loam_between_factors_enabled_ = true;
  bool loam_imu_only_ = false;  // Turns off landmarks leaving only LOAM and IMU
  bool frame_between_factors_ = true;
  bool keyframe_between_factors_ = true;
  std::string loam_world_frame_ = "unknown";
  std::string loam_sensor_frame_ = "unknown";
  std::string loam_degeneracy_topic_ = "";  // Use empty string if no degeneracy topic available

  bool draw_only_in_smoother_landmarks_ = false;
  // If true, will draw lidar depth lines published in landmarks image.
  // When lidar depth is not available, no landmarks image will be published.
  bool draw_lidar_lines_ = true;

public:
  static void LoadParams(const ros::NodeHandle& nh);
  GlobalParams(GlobalParams const&) = delete;
  void operator=(GlobalParams const&) = delete;

  // Add parameter accessors here
  static std::string IMUSubTopic();
  static std::string CameraSubTopic();
  static std::string LidarSubTopic();

  static std::string FullPathExportFilePrefix();

  static bool CountFeaturesPerCell();
  static int MinFeaturesPerCell();
  static int MaxFeaturesPerCell();
  static int MaxFeatures();
  static int GridCellsX();
  static int GridCellsY();
  static double ResizeFactor();

  static int KLTMaxIterations();
  static double KLTConvergenceEpsilon();
  static int KLTWinSize();
  static int KLTPyramids();

  static double FeatureExtractionQualityLevel();
  static double FeatureExtractionMinDistance();
  static double FeatureExtractionMinEigenValue();

  static int LandmarkCullingFrameCount();
  static double LandmarkCullingObservationPercentage();
  static int LandmarkMatchingWindow();
  static std::string GroundTruthFile();
  static std::string GroundTruthProvider();
  static std::string LidarTimeOffsetFile();
  static std::string LidarTimeOffsetType();
  static double LidarFrameManagerTimestampThresh();
  static bool InitOnGroundTruth();
  static double MatchMaxDistance();
  static int InitKeyframeInterval();
  static int NumGoodKeyframesForInitialization();
  static bool AddEssentialMatrixConstraints();
  static double MinKeyframeFeatureInlierRatio();
  static double MinParallaxForSmoothing();
  static double MinParallaxForSmoothingDepth();
  static double MaxParallaxRotationCompensation();
  static double NumHighParallaxPointsForKeyframe();
  static int MinActiveTrackCount();
  static bool UseAngleParallax();

  static int SecondRANSACNFrames();

  static bool UseParallaxKeyframes();
  static double MinParallaxForKeyframe();
  static int TemporalKeyframeInterval();
  static int FrameInterval();

  static bool UseIsam();
  static bool UseDogLeg();
  static double IsamRelinearizeThresh();
  static int IsamRelinearizeSkip();
  static bool SaveFactorGraphsToFile();
  static double InitRangeFactorLength();
  static int MinKeyframesForNominal();
  static bool UseFixedLag();
  static double SmootherLag();
  static bool EnableSmartFactors();

  static double IsamRelinThreshXRotation();
  static double IsamRelinThreshXTranslation();
  static double IsamRelinThreshV();
  static double IsamRelinThreshB();
  static double IsamRelinThreshL();

  static int FeatureExtractionInterval();
  static int TrackCountLowerThresh();
  static double TrackNMSSquaredDistThresh();
  static int MinTrackLengthForSmoothing();
  static int MinTrackLengthForSmoothingDepth();
  static int MinDepthMeasurementsForSmoothing();
  static double ImageEdgePaddingPercent();
  static double StationaryThresh();

  static double PriorNoiseXYaw();
  static double PriorNoiseXRollPitch();
  static double PriorNoiseXTranslation();
  static double PriorNoiseGyro();
  static double PriorNoiseAccel();
  static double PriorNoiseVelocity();
  static double NoiseBetweenRotation();
  static double NoiseBetweenTranslation();
  static double NoiseBetweenRotationKeyframe();
  static double NoiseBetweenTranslationKeyframe();
  static double NoiseFeature();
  static double NoiseRange();
  static double RobustFeatureK();
  static double RobustRangeK();

  static double TimeshiftCamImu();  // t_imu = t_cam + shift
  static int IMUMaxMessagesRetained();
  static double IMUG();
  static double IMUNGravityX();
  static double IMUNGravityY();
  static double IMUNGravityZ();
  static double IMUAccelNoiseDensity();
  static double IMUGyroNoiseDensity();
  static double IMUAccelRandomWalk();
  static double IMUGyroRandomWalk();
  static std::vector<double> IMUInitBiasAccel();
  static std::vector<double> IMUInitBiasGyro();

  static bool DoInitialGravityAlignment();
  static double DynamicOutlierRejectionThreshold();
  static double LandmarkDistanceThreshold();
  static double ProjLandmarkInitDistanceThresh();
  static int ExtraISAM2UpdateSteps();
  static double TriangulationRankTol();

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
  static bool ColorImage();

  static std::vector<double> DistortionCoefficients();
  static std::string DistortionModel();

  static bool LidarDepthEnabled();
  static int LidarDepthCalcMode();
  static int LidarDepthSearchWindowWidth();
  static int LidarDepthSearchWindowHeight();
  static int LidarDepthMinNonZeroNeighbors();
  static double LidarDepthMaxAllowedFeatureDistance();
  static double LidarDepthStdDevTolFactor();
  static int LidarMaxMessagesRetained();

  static std::string LoamWorldFrame();
  static std::string LoamSensorFrame();
  static std::string LoamDegeneracySubTopic();
  static bool LoamBetweenFactorsEnabled();
  static bool LoamIMUOnly();
  static bool FrameBetweenFactors();
  static bool KeyframeBetweenFactors();

  static bool LandmarkRemovalHighDelta();
  static bool LandmarkRemovalHighDepthDifference();
  static double MaxDepthDifferenceBeforeRemoval();

  static bool DrawOnlyInSmootherLandmarks();
  static bool DrawLidarLines();
};

#endif
