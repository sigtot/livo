#ifndef ORB_TEST_INCLUDE_LIVO_FEATURE_H_
#define ORB_TEST_INCLUDE_LIVO_FEATURE_H_

#include "frame.h"
struct Feature
{
  std::shared_ptr<Frame> frame;
  cv::Point2f pt;
};

#endif  // ORB_TEST_INCLUDE_LIVO_FEATURE_H_
