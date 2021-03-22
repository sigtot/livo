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
  cv::Mat E;
  cv::Mat R;
  std::vector<double> t;  // Scale is not observable, only direction.
  double S_H;
  double S_F;
  double R_H;

  KeyframeTransform(std::shared_ptr<Frame> frame1, std::shared_ptr<Frame> frame2, cv::Mat F, cv::Mat E, cv::Mat R,
                    std::vector<double>  t, double S_H, double S_F, double R_H)
    : frame1(std::move(frame1))
    , frame2(std::move(frame2))
    , F(std::move(F))
    , E(std::move(E))
    , R(std::move(R))
    , t(std::move(t))
    , S_H(S_H)
    , S_F(S_F)
    , R_H(R_H)
  {
  }

  bool FundamentalMatGood() const
  {
    return Valid() && R_H < 0.45;
  }

  bool Valid() const
  {
    return S_H != -1 && S_F != -1 && R_H != -1;
  }

  cv::Mat GetEssentialMat() const
  {
    return E;
  }

  cv::Mat GetRotation() const
  {
    return R;
  }

  std::vector<double> GetTranslation() const
  {
    return t;
  }
};

#endif  // ORB_TEST_INCLUDE_LIVO_KEYFRAME_TRANSFORM_H_
