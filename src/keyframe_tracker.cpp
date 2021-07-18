#include "keyframe_tracker.h"
#include "global_params.h"
#include "feature_helpers.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include <Initializer.h>

void KeyframeTracker::TryAddFrameSafe(const std::shared_ptr<Frame>& frame2,
                                      const std::vector<std::shared_ptr<Track>>& tracks)
{
  auto frame1 = keyframe_transforms_.empty() ? first_frame_ : keyframe_transforms_.back().frame2;
  TryAddFrame(frame1, frame2, tracks);
}

void KeyframeTracker::TryAddFrame(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                                  const std::vector<std::shared_ptr<Track>>& tracks)
{
  auto transform = TryMakeKeyframeTransform(frame1, frame2, tracks);
  if (!transform.HaveEnoughMatches())
  {
    // Too few matches likely means high feature loss, so we need to initialize a new keyframe to keep tracking
    // In this case, a visual only solution could yield wrong results, so hopefully, IMU or LiDAR data can help us
    std::cout << " Not enough matches " << std::endl;
    keyframe_transforms_.push_back(transform);
    return;
  }
  if (transform.Valid())
  {
    std::cout << "Adding new keyframe transform " << frame1->id << " -> " << frame2->id << std::endl;
    frame1->is_keyframe = true;
    frame2->is_keyframe = true;
    transform.UpdateTrackInlierOutlierCounts();
    if (!keyframe_transforms_.empty() && keyframe_transforms_.back().Valid())
    {
      ChooseBestHomographyDecomposition(transform, keyframe_transforms_.back());
    }
    keyframe_transforms_.push_back(transform);
  }
}

KeyframeTracker::KeyframeTracker(std::shared_ptr<Frame> frame1) : first_frame_(std::move(frame1))
{
}

void KeyframeTracker::ChooseBestHomographyDecomposition(KeyframeTransform& transform,
                                                        KeyframeTransform& reference_transform)
{
  if (!transform.homography_decomposition_result || !reference_transform.homography_decomposition_result)
  {
    return;  // Nothing to do
  }
  auto reference_selected_index = transform.homography_decomposition_result->selected_index;
  std::vector<cv::Mat> reference_normals =
      reference_selected_index ?
          reference_transform.homography_decomposition_result->normals[*reference_selected_index] :
          reference_transform.homography_decomposition_result->normals;
  int best_idx = 0;
  int best_reference_idx = 0;
  double largest_dot_product = -1.;
  for (int i = 0; i < transform.homography_decomposition_result->normals.size(); ++i)
  {
    for (int j = 0; j < reference_normals.size(); ++j)
    {
      cv::Mat dot_product_mat = transform.homography_decomposition_result->normals[i].t() *
                                reference_transform.homography_decomposition_result->normals[j];
      double dot_product = dot_product_mat.at<double>(0);
      if (abs(dot_product) > abs(largest_dot_product))
      {
        largest_dot_product = dot_product;
        best_idx = i;
        best_reference_idx = j;
      }
    }
  }
  transform.homography_decomposition_result->selected_index = boost::optional<int>(best_idx);
  if (!reference_selected_index)
  {
    reference_transform.homography_decomposition_result->selected_index = boost::optional<int>(best_reference_idx);
  }
}

KeyframeTransform KeyframeTracker::TryMakeKeyframeTransform(const std::shared_ptr<Frame>& frame1,
                                                            const std::shared_ptr<Frame>& frame2,
                                                            const std::vector<std::shared_ptr<Track>>& tracks)
{
  if (frame1->stationary && frame2->stationary)
  {
    return KeyframeTransform::Invalid(frame1, frame2);
  }
  std::vector<uchar> inlier_mask;

  auto feature_matches = frame1->GetFeatureMatches(frame2);
  std::vector<cv::Point2f> points1;
  std::vector<cv::Point2f> points2;
  for (const auto& match : feature_matches)
  {
    auto feature1 = match.first.lock();
    auto feature2 = match.second.lock();
    if (feature1 && feature2)
    {
      points1.push_back(feature1->pt);
      points2.push_back(feature2->pt);
    }
  }
  assert(points1.size() == points2.size());

  if (!KeyframeTransform::EnoughMatches(feature_matches.size()))
  {
    return KeyframeTransform::Invalid(frame1, frame2);
  }

  auto F = cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 3., 0.99, inlier_mask);
  assert(inlier_mask.size() == points1.size());
  auto H = cv::findHomography(points1, points2, cv::RANSAC, 3);

  std::vector<bool> F_check_inliers;
  double S_F = ORB_SLAM::CheckFundamental(F, F_check_inliers, points1, points2);

  std::vector<bool> H_check_inliers;
  double S_H = ORB_SLAM::CheckHomography(H, H.inv(), H_check_inliers, points1, points2);

  double R_H = S_H / (S_H + S_F);

  // Update inlier_mask with inliers from F check
  for (int i = 0; i < inlier_mask.size(); ++i)
  {
    inlier_mask[i] = inlier_mask[i] && F_check_inliers[i];
  }

  cv::Mat K = (cv::Mat_<double>(3, 3) << GlobalParams::CamFx(), 0., GlobalParams::CamU0(), 0., GlobalParams::CamFy(),
               GlobalParams::CamV0(), 0., 0., 1.);

  auto transform = KeyframeTransform(frame1, frame2, feature_matches, F, S_H, S_F, R_H, inlier_mask);
  if (!transform.FundamentalMatGood())
  {
    return KeyframeTransform::Invalid(frame1, frame2);
  }

  // Essential matrix and recovered R and t represent transformation from cam 2 to cam 1 in cam 2 frame
  // As such, we compute it in reverse order, so that we obtain the transformation from frame1 to frame2 in frame1
  // frame
  auto E = cv::findEssentialMat(points2, points1, K, cv::RANSAC);
  cv::Mat R;
  std::vector<double> t;
  cv::recoverPose(E, points2, points1, K, R, t);

  EssentialMatrixDecompositionResult result(E, R, t);
  transform.essential_matrix_decomposition_result = boost::optional<EssentialMatrixDecompositionResult>(result);

  // We compute the homography also if a fundamental matrix is a better fit, to compare normals with later frames
  std::vector<cv::Mat> Rs, ts, normals;
  cv::decomposeHomographyMat(H, K, Rs, ts, normals);
  std::vector<cv::Mat> valid_Rs, valid_ts, valid_normals;
  int least_invalids_idx = 0;
  int least_invalids = 999999;
  for (int i = 0; i < Rs.size(); ++i)
  {
    int invalids = NumPointsBehindCamera(points2, normals[i], K.inv());
    if (invalids == 0)
    {
      valid_Rs.push_back(Rs[i]);
      valid_ts.push_back(ts[i]);
      valid_normals.push_back(normals[i]);
      least_invalids = invalids;
      least_invalids_idx = i;
    }
    else if (invalids < least_invalids)
    {
      least_invalids = invalids;
      least_invalids_idx = i;
    }
  }
  if (valid_Rs.empty())  //
  {
    std::cout << "WARN: May be using R and t decomposed from a degenerate homography!" << std::endl;
    std::cout << "Some (" << least_invalids << "/" << points2.size() << ") points were projected to be behind camera "
              << std::endl;
    valid_Rs.push_back(Rs[least_invalids_idx]);
    valid_ts.push_back(ts[least_invalids_idx]);
    valid_normals.push_back(normals[least_invalids_idx]);
  }
  assert(valid_Rs.size() <= 2);
  HomographyDecompositionResult homography_decomp_result(H, valid_Rs, valid_ts, valid_normals);
  transform.homography_decomposition_result = boost::optional<HomographyDecompositionResult>(homography_decomp_result);

  auto R12 = transform.GetRotation();
  if (!R12)
  {
    return KeyframeTransform::Invalid(frame1, frame2);
  }
  std::vector<double> rotation_compensated_parallaxes;
  std::vector<double> uncompensated_parallaxes;
  ComputePointParallaxes(points1, points2, *R12, K, GlobalParams::MinParallaxForKeyframe(),
                         rotation_compensated_parallaxes);
  auto num_high_motion = ComputePointParallaxes(points1, points2, cv::Mat::eye(3, 3, CV_64F), K,
                                                GlobalParams::MinParallaxForKeyframe(), uncompensated_parallaxes);

  for (int i = 0; i < rotation_compensated_parallaxes.size(); ++i)
  {
    if (inlier_mask[i])
    {
      // Simple check to avoid bad rotation estimates (e.g. due to very low motion):
      // Rotation compensation should only correct the parallax by a reasonable amount
      if (std::abs(uncompensated_parallaxes[i] - rotation_compensated_parallaxes[i]) >
          GlobalParams::MaxParallaxRotationCompensation())
      {
        return KeyframeTransform::Invalid(frame1, frame2);
      }
      auto feature = feature_matches[i].second.lock();
      if (feature)
      {
        auto track = feature->track.lock();
        if (track)
        {
          track->max_parallax = std::max(track->max_parallax, rotation_compensated_parallaxes[i]);
        }
      }
    }
  }

  if (num_high_motion <= GlobalParams::NumHighParallaxPointsForKeyframe())
  {
    return KeyframeTransform::Invalid(frame1, frame2);
  }

  int inlier_count = 0;
  int outlier_count = 0;
  for (const auto& inlier : inlier_mask)
  {
    if (inlier)
    {
      ++inlier_count;
    }
    else
    {
      ++outlier_count;
    }
  }
  std::cout << inlier_count << "/" << outlier_count << " inliers/outliers" << std::endl;

  return transform;
}

int KeyframeTracker::ComputePointParallaxes(const std::vector<cv::Point2f>& points1,
                                            const std::vector<cv::Point2f>& points2, const cv::Mat& R12,
                                            const cv::Mat& K, double min_parallax_for_keyframe,
                                            std::vector<double>& parallaxes)
{
  int num_high_parallax = 0;
  parallaxes.resize(points1.size());
  auto H_rot_only = R12.inv();  // Quite spooky that we need to invert this: Need to take a better look at this
  cv::Mat K_inv = K.inv();
  for (int i = 0; i < points1.size(); ++i)
  {
    cv::Point2f _;
    double dist = ComputePointParallax(points1[i], points2[i], R12, K, K_inv, _);
    parallaxes[i] = dist;
    if (dist > min_parallax_for_keyframe)
    {
      num_high_parallax++;
    }
  }
  return num_high_parallax;
}

int KeyframeTracker::NumPointsBehindCamera(const std::vector<cv::Point2f>& points, const cv::Mat& n,
                                           const cv::Mat& K_inv)
{
  int num_invalid = 0;
  for (const auto& point : points)
  {
    cv::Mat p = (cv::Mat_<double>(3, 1) << point.x, point.y, 1.);
    cv::Mat m = K_inv * p;
    cv::Mat res = m.t() * n;
    if (res.at<double>(0) <= 0)
    {
      num_invalid++;
    }
  }
  return num_invalid;
}

int KeyframeTracker::GetMostRecentBadTransformIdx() const
{
  int i = static_cast<int>(keyframe_transforms_.size()) - 1;
  for (; i >= 0 && keyframe_transforms_[i].Valid(); --i)
  {
  }
  return i;
}

int KeyframeTracker::GetNumberOfGoodTransforms() const
{
  return static_cast<int>(keyframe_transforms_.size()) - GetMostRecentBadTransformIdx() - 1;
}

std::vector<KeyframeTransform> KeyframeTracker::GetKeyframeTransforms() const
{
  return keyframe_transforms_;
}

bool KeyframeTracker::GoodForInitialization() const
{
  if (GetNumberOfGoodTransforms() < GlobalParams::NumGoodKeyframesForInitialization())
  {
    return false;
  }

  // The fundamental matrix should be good in the last few frames to enable proper triangulation
  for (int i = static_cast<int>(keyframe_transforms_.size()) - 1;
       i > keyframe_transforms_.size() - GlobalParams::NumGoodKeyframesForInitialization(); --i)
  {
    if (!keyframe_transforms_[i].FundamentalMatGood())
    {
      return false;
    }
  }
  return true;
}

KeyframeTransform KeyframeTracker::GetNewestKeyframeTransform() const
{
  return keyframe_transforms_.back();
}
