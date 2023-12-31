#ifndef ORB_TEST_INCLUDE_LIVO_BETWEEN_TRANSFORM_PROVIDER_H_
#define ORB_TEST_INCLUDE_LIVO_BETWEEN_TRANSFORM_PROVIDER_H_

#include <boost/optional.hpp>

namespace gtsam
{
class Pose3;
}

class BetweenTransformProvider
{
public:
  virtual bool IsDegenerate(double timestamp) = 0;
  virtual boost::optional<gtsam::Pose3> GetBetweenTransform(double timestamp1, double timestamp2) = 0;
  virtual bool CanTransform(double timestamp) = 0;
};

#endif  // ORB_TEST_INCLUDE_LIVO_BETWEEN_TRANSFORM_PROVIDER_H_
