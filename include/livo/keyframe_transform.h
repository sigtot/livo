#ifndef ORB_TEST_INCLUDE_LIVO_KEYFRAME_TRANSFORM_H_
#define ORB_TEST_INCLUDE_LIVO_KEYFRAME_TRANSFORM_H_

#include <utility>
#include <vector>
#include <opencv2/core/core.hpp>

#include "frame.h"

struct KeyframeTransform
{
  const std::shared_ptr<Frame> frame1;
  const std::shared_ptr<Frame> frame2;

  cv::Mat F;
  double S_H;
  double S_F;
  double R_H;

  KeyframeTransform(std::shared_ptr<Frame> frame1, std::shared_ptr<Frame> frame2, cv::Mat F, double S_H, double S_F,
                    double R_H)
    : frame1(std::move(frame1)), frame2(std::move(frame2)), F(std::move(F)), S_H(S_H), S_F(S_F), R_H(R_H)
  {
  }

  bool FundamentalMatGood() const {
    return R_H < 0.45;
  }
};

#endif  // ORB_TEST_INCLUDE_LIVO_KEYFRAME_TRANSFORM_H_
