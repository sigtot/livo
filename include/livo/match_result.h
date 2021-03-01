#ifndef ORB_TEST_FEATUREEXTRACTOR_CPP_MATCH_RESULT_H_
#define ORB_TEST_FEATUREEXTRACTOR_CPP_MATCH_RESULT_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

#include "frame.h"

struct MatchResult {
  std::vector<cv::DMatch> matches{};
  std::vector<uchar> inliers{};
};

#endif
