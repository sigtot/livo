#ifndef ORB_TEST_INCLUDE_LIVO_SMART_FACTOR_IN_SMOOTHER_H_
#define ORB_TEST_INCLUDE_LIVO_SMART_FACTOR_IN_SMOOTHER_H_

#include <boost/shared_ptr.hpp>
namespace gtsam
{
class Cal3_S2;

template <class T>
class SmartProjectionPoseFactor;

typedef std::uint64_t FactorIndex;

namespace noiseModel
{
class Base;
class Isotropic;
}
}  // namespace gtsam

typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> SmartFactor;

struct SmartFactorInSmoother
{
  boost::shared_ptr<SmartFactor> smart_factor;
  gtsam::FactorIndex idx_in_new_factors;
  gtsam::FactorIndex idx_in_isam; // TODO Make optional
  bool in_isam = false; // TODO Remove and condition on idx_in_isam instead :)
};

#endif  // ORB_TEST_INCLUDE_LIVO_SMART_FACTOR_IN_SMOOTHER_H_
