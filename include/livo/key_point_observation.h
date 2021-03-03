#ifndef ORB_TEST_FEATUREEXTRACTOR_CPP_KEY_POINT_OBSERVATION_H_
#define ORB_TEST_FEATUREEXTRACTOR_CPP_KEY_POINT_OBSERVATION_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <memory>

struct Frame;
struct Landmark;

struct KeyPointObservation
{
  KeyPointObservation(cv::KeyPoint keypoint, cv::Mat descriptor, std::shared_ptr<Frame> frame);
  const cv::KeyPoint keypoint;
  const cv::Mat descriptor;
  std::shared_ptr<Frame> frame;
  std::weak_ptr<Landmark> landmark;
};

#endif
