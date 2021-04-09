#include "debug_value_publisher.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

DebugValuePublisher& DebugValuePublisher::GetInstance()
{
  static DebugValuePublisher instance;
  return instance;
}

void DebugValuePublisher::SetPublishers(ros::NodeHandle& nh)
{
  GetInstance().nonlinear_error_pub_ = nh.advertise<std_msgs::Float64>("/nonlinear_error", 1000);
  GetInstance().relinearized_cliques_pub_ = nh.advertise<std_msgs::Int32>("/relinearized_cliques", 1000);
  GetInstance().total_cliques_pub_ = nh.advertise<std_msgs::Int32>("/total_cliques", 1000);
}

void DebugValuePublisher::PublishNonlinearError(double nonlinear_error)
{
  std_msgs::Float64 msg;
  msg.data = nonlinear_error;
  GetInstance().nonlinear_error_pub_.publish(msg);
}

void DebugValuePublisher::PublishRelinearizedCliques(int relinearized_cliques)
{
  std_msgs::Int32 msg;
  msg.data = relinearized_cliques;
  GetInstance().relinearized_cliques_pub_.publish(msg);
}

void DebugValuePublisher::PublishTotalCliques(int total_cliques)
{
  std_msgs::Int32 msg;
  msg.data = total_cliques;
  GetInstance().total_cliques_pub_.publish(msg);
}
