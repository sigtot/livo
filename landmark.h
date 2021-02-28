#ifndef ORB_TEST__LANDMARK_H_
#define ORB_TEST__LANDMARK_H_

#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <memory>
#include "key_point_observation.h"

struct KeyPointObservation;

struct Landmark {
  std::vector<std::shared_ptr<KeyPointObservation>> keypoint_observations{};
  int id{};
};
#endif  // ORB_TEST__LANDMARK_H_
