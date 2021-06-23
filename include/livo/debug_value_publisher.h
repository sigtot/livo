#ifndef ORB_TEST_INCLUDE_LIVO_DEBUG_VALUE_PUBLISHER_H_
#define ORB_TEST_INCLUDE_LIVO_DEBUG_VALUE_PUBLISHER_H_

#include <ros/ros.h>

class DebugValuePublisher
{
private:
  ros::Publisher nonlinear_error_pub_;
  ros::Publisher relinearized_cliques_pub_;
  ros::Publisher reeliminated_cliques_pub_;
  ros::Publisher total_cliques_pub_;
  ros::Publisher update_duration_pub_;
  ros::Publisher extra_updates_duration_pub_;
  ros::Publisher frontend_duration_pub_;
  ros::Publisher feature_extraction_duration_pub_;
  ros::Publisher klt_duration_pub_;
  ros::Publisher publish_image_duration_pub_;
  ros::Publisher img_queue_size_pub_;
  ros::Publisher abs_gt_error_pub_;

  ros::Publisher n_cells_repopulated_;

  ros::Publisher bias_acc_x_pub_;
  ros::Publisher bias_acc_y_pub_;
  ros::Publisher bias_acc_z_pub_;

  ros::Publisher bias_gyro_x_pub_;
  ros::Publisher bias_gyro_y_pub_;
  ros::Publisher bias_gyro_z_pub_;

  ros::Publisher velocity_norm_average_pub_;

  ros::Publisher frame_id_pub_;

  DebugValuePublisher() = default;
  static DebugValuePublisher& GetInstance();

  static void PublishDoubleValue(double value, ros::Publisher& publisher);
  static void PublishIntValue(int value, ros::Publisher& publisher);

public:
  DebugValuePublisher(DebugValuePublisher const&) = delete;
  void operator=(DebugValuePublisher const&) = delete;

  static void SetPublishers(ros::NodeHandle& nh);

  static void PublishNonlinearError(double nonlinear_error);
  static void PublishRelinearizedCliques(int relinearized_cliques);
  static void PublishReeliminatedCliques(int relinearized_cliques);
  static void PublishTotalCliques(int total_cliques);
  static void PublishUpdateDuration(int duration);
  static void PublishExtraUpdatesDuration(int duration);
  static void PublishFrontendDuration(double duration);
  static void PublishFeatureExtractionDuration(double duration);
  static void PublishKLTDuration(double duration);
  static void PublishImagePublishDuration(double duration);
  static void PublishImageQueueSize(int size);
  static void PublishAbsoluteGroundTruthError(double err);

  static void PublishNCellsRepopulated(int n);

  static void PublishBias(const std::vector<double>& acc_bias, const std::vector<double>& gyro_bias);
  static void PublishVelocityNormAverage(double velocity_norm_average);
  static void PublishFrameId(int frame_id);
};

#endif  // ORB_TEST_INCLUDE_LIVO_DEBUG_VALUE_PUBLISHER_H_
