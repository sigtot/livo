#ifndef ORB_TEST_INCLUDE_LIVO_LANDMARK_IN_SMOOTHER_H_
#define ORB_TEST_INCLUDE_LIVO_LANDMARK_IN_SMOOTHER_H_

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

namespace gtsam
{
class Cal3_S2;

template <class T>
class SmartProjectionPoseFactor;

namespace noiseModel
{
class Base;
class Isotropic;
}
}  // namespace gtsam

typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> SmartFactor;

struct LandmarkInSmoother
{
  boost::shared_ptr<gtsam::noiseModel::Isotropic> noise_model;
  boost::optional<boost::shared_ptr<gtsam::noiseModel::Base>> robust_noise_model; // Optional for proj factors
  boost::optional<boost::shared_ptr<SmartFactor>> smart_factor;
  int newest_frame_id_seen; // Assumes frame id is strictly increasing
  int first_frame_id_seen; // Assumes frame id is strictly increasing
  double init_timestamp; // Timestamp at which the landmark was initialized
};

#endif  // ORB_TEST_INCLUDE_LIVO_LANDMARK_IN_SMOOTHER_H_
