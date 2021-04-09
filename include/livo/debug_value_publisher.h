#ifndef ORB_TEST_INCLUDE_LIVO_DEBUG_VALUE_PUBLISHER_H_
#define ORB_TEST_INCLUDE_LIVO_DEBUG_VALUE_PUBLISHER_H_

#include <ros/ros.h>

class DebugValuePublisher
{
private:
  ros::Publisher nonlinear_error_pub_;
  ros::Publisher relinearized_cliques_pub_;
  ros::Publisher total_cliques_pub_;

  DebugValuePublisher() = default;
  static DebugValuePublisher& GetInstance();

public:
  DebugValuePublisher(DebugValuePublisher const&) = delete;
  void operator=(DebugValuePublisher const&) = delete;

  static void SetPublishers(ros::NodeHandle& nh);

  static void PublishNonlinearError(double nonlinear_error);
  static void PublishRelinearizedCliques(int relinearized_cliques);
  static void PublishTotalCliques(int total_cliques);
};

#endif  // ORB_TEST_INCLUDE_LIVO_DEBUG_VALUE_PUBLISHER_H_
