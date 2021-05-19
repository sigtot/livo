#ifndef ORB_TEST_INCLUDE_LIVO_BETWEEN_TRANSFORM_PROVIDER_H_
#define ORB_TEST_INCLUDE_LIVO_BETWEEN_TRANSFORM_PROVIDER_H_

namespace gtsam
{
class Pose3;
}

class BetweenTransformProvider
{
public:
  virtual gtsam::Pose3 GetBetweenTransform(double timestamp1, double timestamp2) = 0;
};

#endif  // ORB_TEST_INCLUDE_LIVO_BETWEEN_TRANSFORM_PROVIDER_H_
