#ifndef ORB_TEST_INCLUDE_LIVO_KEYFRAME_TRANSFORM_H_
#define ORB_TEST_INCLUDE_LIVO_KEYFRAME_TRANSFORM_H_

#include <utility>
#include <vector>
#include <opencv2/core/core.hpp>
#include <boost/optional.hpp>

#include "frame.h"
#include "feature.h"
#include "track.h"
#include "essential_matrix_decomposition_result.h"
#include "homography_decomposition_result.h"

struct KeyframeTransform
{
  const std::shared_ptr<Frame> frame1;
  const std::shared_ptr<Frame> frame2;

  bool stationary;

  std::vector<FeatureMatch> feature_matches;
  boost::optional<std::vector<uchar>> inlier_mask;

  cv::Mat F;
  double S_H;
  double S_F;
  double R_H;
  boost::optional<EssentialMatrixDecompositionResult> essential_matrix_decomposition_result;
  boost::optional<HomographyDecompositionResult> homography_decomposition_result;

  KeyframeTransform(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                    std::vector<FeatureMatch> feature_matches, cv::Mat F, double S_H, double S_F, double R_H,
                    boost::optional<std::vector<uchar>> inlier_mask = boost::none)
    : frame1(frame1)
    , frame2(frame2)
    , feature_matches(std::move(feature_matches))
    , F(std::move(F))
    , S_H(S_H)
    , S_F(S_F)
    , R_H(R_H)
    , stationary(frame1->stationary && frame2->stationary)
    , inlier_mask(std::move(inlier_mask))
  {
    if (inlier_mask)
    {
      assert(inlier_mask->size() == feature_matches.size());
    }
  }

  static KeyframeTransform Invalid(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2)
  {
    return KeyframeTransform(frame1, frame2, frame1->GetFeatureMatches(frame2), cv::Mat(), -1, -1, -1);
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

  void UpdateTrackInlierOutlierCounts()
  {
    if (inlier_mask)
    {
      for (int i = 0; i < inlier_mask->size(); ++i)
      {
        auto feature = feature_matches[i].first.lock();
        if (feature)
        {
          auto track = feature->track.lock();
          if (track)
          {
            if ((*inlier_mask)[i])
            {
              track->inlier_count++;
            }
            else
            {
              track->outlier_count++;
            }
          }
        }
      }
    }
  }

  static bool EnoughMatches(size_t match_count)
  {
    return match_count >= 8;
  }

  bool HaveEnoughMatches() const
  {
    return EnoughMatches(feature_matches.size());
  }
};

#endif  // ORB_TEST_INCLUDE_LIVO_KEYFRAME_TRANSFORM_H_
