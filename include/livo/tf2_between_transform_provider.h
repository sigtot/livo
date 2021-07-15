#ifndef ORB_TEST_SRC_TF2_BETWEEN_TRANSFORM_PROVIDER_H_
#define ORB_TEST_SRC_TF2_BETWEEN_TRANSFORM_PROVIDER_H_

#include "between_transform_provider.h"
#include <orb_test/ForceDegeneracy.h>

#include <memory>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ros/subscriber.h>
#include <std_msgs/Header.h>
#include <boost/optional.hpp>
#include <mutex>

class TF2BetweenTransformProvider : public BetweenTransformProvider
{
private:
  // Configuration
  std::shared_ptr<gtsam::Pose3> body_p_sensor_;
  std::string world_frame_;
  std::string sensor_frame_;

  // Members
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  std::map<double, unsigned int> degeneracy_buffer_;
  std::mutex degeneracy_buffer_mu_;
  bool force_degenerate_ = false;

  bool IsDegenerate(double timestamp);

public:
  TF2BetweenTransformProvider(const gtsam::Pose3& body_p_sensor, std::string world_frame, std::string sensor_frame);

  /**
   * Constructor taking std::vectors for body_p_sensor.
   *
   * @param body_p_sensor_quat vector with order [x, y, z, w]
   * @param body_p_sensor_vector vector with order [x, y, z]
   * @param world_frame
   * @param sensor_frame
   */
  TF2BetweenTransformProvider(const std::vector<double>& body_p_sensor_quat,
                              const std::vector<double>& body_p_sensor_vector, std::string world_frame,
                              std::string sensor_frame);

  bool ForceDegeneracy(orb_test::ForceDegeneracy::Request& req, orb_test::ForceDegeneracy::Response& res);

  boost::optional<gtsam::Pose3> GetBetweenTransform(double timestamp1, double timestamp2) override;
  bool CanTransform(double timestamp) override;

  void AddDegeneracyMessage(const std_msgs::Header& msg);
};

#endif  // ORB_TEST_SRC_TF2_BETWEEN_TRANSFORM_PROVIDER_H_
