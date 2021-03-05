#ifndef ORB_TEST_INCLUDE_LIVO_LANDMARK_MATCH_H_
#define ORB_TEST_INCLUDE_LIVO_LANDMARK_MATCH_H_

#include "landmark.h"
#include "key_point_observation.h"
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

struct LandmarkMatch
{
  std::shared_ptr<Landmark> landmark_;
  std::shared_ptr<KeyPointObservation> observation_;
  double distance_;
};

#endif  // ORB_TEST_INCLUDE_LIVO_LANDMARK_MATCH_H_
