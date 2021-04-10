#ifndef ORB_TEST_INCLUDE_LIVO_DEBUG_VALUE_PUBLISHER_H_
#define ORB_TEST_INCLUDE_LIVO_DEBUG_VALUE_PUBLISHER_H_

#include <ros/ros.h>

class DebugValuePublisher
{
private:
  ros::Publisher nonlinear_error_pub_;
  ros::Publisher relinearized_cliques_pub_;
  ros::Publisher total_cliques_pub_;

  ros::Publisher bias_acc_x_pub_;
  ros::Publisher bias_acc_y_pub_;
  ros::Publisher bias_acc_z_pub_;

  ros::Publisher bias_gyro_x_pub_;
  ros::Publisher bias_gyro_y_pub_;
  ros::Publisher bias_gyro_z_pub_;

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
  static void PublishTotalCliques(int total_cliques);
  static void PublishBias(const std::vector<double>& acc_bias, const std::vector<double>& gyro_bias);
};

#endif  // ORB_TEST_INCLUDE_LIVO_DEBUG_VALUE_PUBLISHER_H_