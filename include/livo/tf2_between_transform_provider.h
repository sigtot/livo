#ifndef ORB_TEST_SRC_TF2_BETWEEN_TRANSFORM_PROVIDER_H_
#define ORB_TEST_SRC_TF2_BETWEEN_TRANSFORM_PROVIDER_H_

#include "between_transform_provider.h"

#include <memory>

class TF2BetweenTransformProvider : public BetweenTransformProvider
{
private:
public:
  TF2BetweenTransformProvider(const gtsam::Pose3& body_p_sensor, std::string  world_frame,
                              std::string  sensor_frame);

private:
  std::shared_ptr<gtsam::Pose3> body_p_sensor;
  std::string world_frame;
  std::string sensor_frame;

public:
  gtsam::Pose3 GetBetweenTransform(double timestamp1, double timestamp2) override;
};

#endif  // ORB_TEST_SRC_TF2_BETWEEN_TRANSFORM_PROVIDER_H_
