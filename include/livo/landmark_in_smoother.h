#ifndef ORB_TEST_INCLUDE_LIVO_LANDMARK_IN_SMOOTHER_H_
#define ORB_TEST_INCLUDE_LIVO_LANDMARK_IN_SMOOTHER_H_

#include "smart_factor_in_smoother.h"

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

namespace gtsam
{
namespace noiseModel
{
class Base;
class Isotropic;
}
}  // namespace gtsam

struct LandmarkInSmoother
{
  boost::shared_ptr<gtsam::noiseModel::Isotropic> noise_model;
  boost::optional<boost::shared_ptr<gtsam::noiseModel::Base>> robust_noise_model; // Optional for proj factors
  boost::optional<SmartFactorInSmoother> smart_factor_in_smoother;
  int newest_frame_id_seen; // Assumes frame id is strictly increasing
  int first_frame_id_seen; // Assumes frame id is strictly increasing
  double init_timestamp; // Timestamp at which the landmark was initialized
};

#endif  // ORB_TEST_INCLUDE_LIVO_LANDMARK_IN_SMOOTHER_H_
