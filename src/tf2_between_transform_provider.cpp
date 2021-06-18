#include "tf2_between_transform_provider.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>

#include <tf2_ros/buffer.h>

#include <utility>

TF2BetweenTransformProvider::TF2BetweenTransformProvider(const std::vector<double>& body_p_sensor_quat,
                                                         const std::vector<double>& body_p_sensor_vector,
                                                         std::string world_frame, std::string sensor_frame)
  : TF2BetweenTransformProvider(
        gtsam::Pose3(gtsam::Rot3::Quaternion(body_p_sensor_quat[3], body_p_sensor_quat[0], body_p_sensor_quat[1],
                                             body_p_sensor_quat[2]),
                     gtsam::Point3(body_p_sensor_vector[0], body_p_sensor_vector[1], body_p_sensor_vector[2])),
        std::move(world_frame), std::move(sensor_frame))
{
}

TF2BetweenTransformProvider::TF2BetweenTransformProvider(const gtsam::Pose3& body_p_sensor, std::string world_frame,
                                                         std::string sensor_frame)
  : body_p_sensor_(std::make_shared<gtsam::Pose3>(body_p_sensor))
  , world_frame_(std::move(world_frame))
  , sensor_frame_(std::move(sensor_frame))
  , tf_buffer(ros::Duration(60.))
  , tf_listener(tf_buffer)
{
}

boost::optional<gtsam::Pose3> TF2BetweenTransformProvider::GetBetweenTransform(double timestamp1, double timestamp2)
{
  try
  {
    auto tf_1 = tf_buffer.lookupTransform(world_frame_, sensor_frame_, ros::Time(timestamp1)).transform;
    auto tf_2 = tf_buffer.lookupTransform(world_frame_, sensor_frame_, ros::Time(timestamp2)).transform;

    gtsam::Pose3 loam_T_l1(gtsam::Rot3::Quaternion(tf_1.rotation.w, tf_1.rotation.x, tf_1.rotation.y, tf_1.rotation.z),
                        gtsam::Point3(tf_1.translation.x, tf_1.translation.y, tf_1.translation.z));
    gtsam::Pose3 loam_T_l2(gtsam::Rot3::Quaternion(tf_2.rotation.w, tf_2.rotation.x, tf_2.rotation.y, tf_2.rotation.z),
                        gtsam::Point3(tf_2.translation.x, tf_2.translation.y, tf_2.translation.z));

    auto l1_T_l2 = loam_T_l1.inverse() * loam_T_l2;
    auto b1_T_b2 = *body_p_sensor_ * l1_T_l2 * body_p_sensor_->inverse();

    return b1_T_b2;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
  return boost::none;
}

bool TF2BetweenTransformProvider::CanTransform(double timestamp)
{
  return tf_buffer.canTransform(world_frame_, sensor_frame_, ros::Time(timestamp));
}
