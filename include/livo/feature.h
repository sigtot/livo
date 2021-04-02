#ifndef ORB_TEST_INCLUDE_LIVO_FEATURE_H_
#define ORB_TEST_INCLUDE_LIVO_FEATURE_H_

#include <utility>

#include "frame.h"
struct Feature
{
  Feature(std::shared_ptr<Frame> frame, const cv::Point2f& pt) : frame(std::move(frame)), pt(pt)
  {
  }
  std::shared_ptr<Frame> frame;
  cv::Point2f pt;
};

#endif  // ORB_TEST_INCLUDE_LIVO_FEATURE_H_
