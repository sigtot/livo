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
  GetInstance().reeliminated_cliques_pub_ = nh.advertise<std_msgs::Int32>("/reeliminated_cliques", 1000);
  GetInstance().total_cliques_pub_ = nh.advertise<std_msgs::Int32>("/total_cliques", 1000);
  GetInstance().update_duration_pub_ = nh.advertise<std_msgs::Int32>("/update_duration", 1000);
  GetInstance().extra_updates_duration_pub_ = nh.advertise<std_msgs::Int32>("/extra_updates_duration", 1000);

  GetInstance().bias_acc_x_pub_ = nh.advertise<std_msgs::Float64>("/bias/acc_x", 1000);
  GetInstance().bias_acc_y_pub_ = nh.advertise<std_msgs::Float64>("/bias/acc_y", 1000);
  GetInstance().bias_acc_z_pub_ = nh.advertise<std_msgs::Float64>("/bias/acc_z", 1000);

  GetInstance().bias_gyro_x_pub_ = nh.advertise<std_msgs::Float64>("/bias/gyro_x", 1000);
  GetInstance().bias_gyro_y_pub_ = nh.advertise<std_msgs::Float64>("/bias/gyro_y", 1000);
  GetInstance().bias_gyro_z_pub_ = nh.advertise<std_msgs::Float64>("/bias/gyro_z", 1000);

  GetInstance().velocity_norm_average_pub_ = nh.advertise<std_msgs::Float64>("/velocity_norm_average", 1000);

  GetInstance().frame_id_pub_ = nh.advertise<std_msgs::Int32>("/frame_id", 1000);
}

void DebugValuePublisher::PublishNonlinearError(double nonlinear_error)
{
  DebugValuePublisher::PublishDoubleValue(nonlinear_error, GetInstance().nonlinear_error_pub_);
}

void DebugValuePublisher::PublishRelinearizedCliques(int relinearized_cliques)
{
  DebugValuePublisher::PublishIntValue(relinearized_cliques, GetInstance().relinearized_cliques_pub_);
}

void DebugValuePublisher::PublishReeliminatedCliques(int reeliminated_cliques)
{
  DebugValuePublisher::PublishIntValue(reeliminated_cliques, GetInstance().reeliminated_cliques_pub_);
}

void DebugValuePublisher::PublishTotalCliques(int total_cliques)
{
  DebugValuePublisher::PublishIntValue(total_cliques, GetInstance().total_cliques_pub_);
}

void DebugValuePublisher::PublishUpdateDuration(int duration)
{
  DebugValuePublisher::PublishIntValue(duration, GetInstance().update_duration_pub_);
}

void DebugValuePublisher::PublishDoubleValue(double value, ros::Publisher& publisher)
{
  std_msgs::Float64 msg;
  msg.data = value;
  publisher.publish(msg);
}

void DebugValuePublisher::PublishIntValue(int value, ros::Publisher& publisher)
{
  std_msgs::Int32 msg;
  msg.data = value;
  publisher.publish(msg);
}

void DebugValuePublisher::PublishBias(const std::vector<double>& acc_bias, const std::vector<double>& gyro_bias)
{
  DebugValuePublisher::PublishDoubleValue(acc_bias[0], GetInstance().bias_acc_x_pub_);
  DebugValuePublisher::PublishDoubleValue(acc_bias[1], GetInstance().bias_acc_y_pub_);
  DebugValuePublisher::PublishDoubleValue(acc_bias[2], GetInstance().bias_acc_z_pub_);

  DebugValuePublisher::PublishDoubleValue(gyro_bias[0], GetInstance().bias_gyro_x_pub_);
  DebugValuePublisher::PublishDoubleValue(gyro_bias[1], GetInstance().bias_gyro_y_pub_);
  DebugValuePublisher::PublishDoubleValue(gyro_bias[2], GetInstance().bias_gyro_z_pub_);
}

void DebugValuePublisher::PublishVelocityNormAverage(double velocity_norm_average)
{
  DebugValuePublisher::PublishDoubleValue(velocity_norm_average, GetInstance().velocity_norm_average_pub_);
}

void DebugValuePublisher::PublishFrameId(int frame_id)
{
  DebugValuePublisher::PublishIntValue(frame_id, GetInstance().frame_id_pub_);
}
void DebugValuePublisher::PublishExtraUpdatesDuration(int duration)
{
  DebugValuePublisher::PublishIntValue(duration, GetInstance().extra_updates_duration_pub_);
}
