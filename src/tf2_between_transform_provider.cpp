#include "tf2_between_transform_provider.h"

#include <gtsam/geometry/Pose3.h>

#include <utility>

TF2BetweenTransformProvider::TF2BetweenTransformProvider(const gtsam::Pose3& body_p_sensor, std::string world_frame,
                                                         std::string sensor_frame)
  : body_p_sensor_(std::make_shared<gtsam::Pose3>(body_p_sensor))
  , world_frame_(std::move(world_frame))
  , sensor_frame_(std::move(sensor_frame))
{
}

gtsam::Pose3 TF2BetweenTransformProvider::GetBetweenTransform(double timestamp1, double timestamp2)
{
  return gtsam::Pose3();
}
