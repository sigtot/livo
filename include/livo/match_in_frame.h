#ifndef ORB_TEST_FEATUREEXTRACTOR_CPP_MATCH_IN_FRAME_H_
#define ORB_TEST_FEATUREEXTRACTOR_CPP_MATCH_IN_FRAME_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "frame.h"

struct MatchInFrame
{
  cv::DMatch match;
  std::shared_ptr<Frame> frame;
};

#endif
