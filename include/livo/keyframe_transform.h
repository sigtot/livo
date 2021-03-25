#ifndef ORB_TEST_INCLUDE_LIVO_KEYFRAME_TRANSFORM_H_
#define ORB_TEST_INCLUDE_LIVO_KEYFRAME_TRANSFORM_H_

#include <utility>
#include <vector>
#include <opencv2/core/core.hpp>
#include <boost/optional.hpp>

#include "frame.h"
#include "essential_matrix_decomposition_result.h"
#include "homography_decomposition_result.h"

struct KeyframeTransform
{
  const std::shared_ptr<Frame> frame1;
  const std::shared_ptr<Frame> frame2;

  cv::Mat F;
  double S_H;
  double S_F;
  double R_H;
  boost::optional<EssentialMatrixDecompositionResult> essential_matrix_decomposition_result;
  boost::optional<HomographyDecompositionResult> homography_decomposition_result;

  KeyframeTransform(std::shared_ptr<Frame> frame1, std::shared_ptr<Frame> frame2, cv::Mat F, double S_H, double S_F,
                    double R_H)
    : frame1(std::move(frame1)), frame2(std::move(frame2)), F(std::move(F)), S_H(S_H), S_F(S_F), R_H(R_H)
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

  boost::optional<cv::Mat> GetEssentialMat() const
  {
    return essential_matrix_decomposition_result ? boost::optional<cv::Mat>(essential_matrix_decomposition_result->E) :
                                                   boost::none;
  }

  boost::optional<cv::Mat> GetRotation() const
  {
    if (essential_matrix_decomposition_result)
    {
      return boost::optional<cv::Mat>(essential_matrix_decomposition_result->R);
    }
    else if (homography_decomposition_result)
    {
      return boost::optional<cv::Mat>(homography_decomposition_result->GetRotation());
    }
    return boost::none;
  }

  boost::optional<std::vector<double>> GetTranslation() const
  {
    if (essential_matrix_decomposition_result)
    {
      return boost::optional<std::vector<double>>(essential_matrix_decomposition_result->t);
    }
    else if (homography_decomposition_result)
    {
      return boost::optional<std::vector<double>>(homography_decomposition_result->GetTranslation());
    }
    return boost::none;
  }
};

#endif  // ORB_TEST_INCLUDE_LIVO_KEYFRAME_TRANSFORM_H_
