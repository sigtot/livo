#ifndef ORB_TEST_FEATUREEXTRACTOR_CPP_KEY_POINT_OBSERVATION_H_
#define ORB_TEST_FEATUREEXTRACTOR_CPP_KEY_POINT_OBSERVATION_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

struct Frame;
struct Landmark;

struct KeyPointObservation {
  cv::KeyPoint keypoint;
  cv::Mat descriptor;
  std::weak_ptr<Landmark> landmark;
  std::weak_ptr<Frame> frame;
};

#endif
