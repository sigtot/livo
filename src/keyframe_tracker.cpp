#include "keyframe_tracker.h"
#include "global_params.h"
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
                                  const std::vector<std::shared_ptr<Track>>& tracks, bool init)
{
  if (!HaveEnoughMatches(frame1, frame2, tracks))
  {
    std::cout << "Lost track: Reinitializing keyframe tracker on frame " << frame2->id << std::endl;
    first_frame_ = frame2;
    keyframe_transforms_.clear();
  }
  std::vector<std::shared_ptr<Track>> valid_tracks;
  OnlyValidTracks(frame1, frame2, tracks, valid_tracks);
  std::vector<uchar> inlier_mask;
  auto transform = TryMakeKeyframeTransform(frame1, frame2, tracks, inlier_mask, init);
  if (transform.Valid())
  {
    std::cout << "Adding new keyframe transform " << frame1->id << " -> " << frame2->id << std::endl;
    UpdateTrackInlierOutlierCounts(valid_tracks, inlier_mask);
    if (!keyframe_transforms_.empty() && keyframe_transforms_.back().Valid())
    {
      ChooseBestHomographyDecomposition(transform, keyframe_transforms_.back());
    }
    keyframe_transforms_.push_back(transform);
  }
}

KeyframeTracker::KeyframeTracker(std::shared_ptr<Frame> frame1)
: first_frame_(std::move(frame1))
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

void KeyframeTracker::GetPoints(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                                const std::vector<std::shared_ptr<Track>>& tracks, std::vector<cv::Point2f>& points1,
                                std::vector<cv::Point2f>& points2, bool init)
{
  for (auto& track : tracks)
  {
    int num_features = static_cast<int>(track->features.size());
    std::shared_ptr<Feature> feat1 = nullptr;
    std::shared_ptr<Feature> feat2 = nullptr;
    for (int i = num_features - 1; i >= 0; --i)
    {
      if (track->features[i]->frame->id == frame1->id)
      {
        feat1 = track->features[i];
      }
      if (track->features[i]->frame->id == frame2->id)
      {
        feat2 = track->features[i];
      }
    }
    assert(feat1 != nullptr);
    assert(feat2 != nullptr);
    if (init)
    {
      track->key_features.push_back(feat1);
    }
    track->key_features.push_back(feat2);

    points1.push_back(feat1->pt);
    points2.push_back(feat2->pt);
  }
}

void KeyframeTracker::GetPointsSafe(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                                    const std::vector<std::shared_ptr<Track>>& tracks,
                                    std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2)
{
  for (const auto& track : tracks)
  {
    if (track->features.empty())
    {
      continue;
    }
    int num_features = static_cast<int>(track->features.size());
    std::shared_ptr<Feature> feat1 = nullptr;
    std::shared_ptr<Feature> feat2 = nullptr;
    for (int i = num_features - 1; i >= 0; --i)
    {
      if (track->features[i]->frame->id == frame1->id)
      {
        feat1 = track->features[i];
      }
      if (track->features[i]->frame->id == frame2->id)
      {
        feat2 = track->features[i];
      }
    }
    if (!feat1 || !feat2)
    {
      continue;
    }

    points1.push_back(feat1->pt);
    points2.push_back(feat2->pt);
  }
}

KeyframeTransform KeyframeTracker::TryMakeKeyframeTransform(const std::shared_ptr<Frame>& frame1,
                                                            const std::shared_ptr<Frame>& frame2,
                                                            const std::vector<std::shared_ptr<Track>>& tracks,
                                                            std::vector<uchar>& inlier_mask, bool init)
{
  if (frame1->stationary && frame2->stationary)
  {
    return KeyframeTransform::Invalid(frame1, frame2);
  }
  std::vector<std::shared_ptr<Track>> valid_tracks;
  OnlyValidTracks(frame1, frame2, tracks, valid_tracks);

  std::vector<cv::Point2f> points1;
  std::vector<cv::Point2f> points2;
  GetPoints(frame1, frame2, valid_tracks, points1, points2, init);
  assert(valid_tracks.size() == points1.size());

  assert(points1.size() == points2.size());
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

  auto transform = KeyframeTransform(frame1, frame2, F, S_H, S_F, R_H);
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
  if (R12)
  {
    std::vector<double> parallaxes;
    int num_high_parallax = ComputePointParallaxes(points1, points2, *R12, K, GlobalParams::MinParallax(), parallaxes);
    for (int i = 0; i < valid_tracks.size(); ++i)
    {
      if (inlier_mask[i])
      {
        valid_tracks[i]->max_parallax = std::max(valid_tracks[i]->max_parallax, parallaxes[i]);
      }
    }
    if (num_high_parallax > GlobalParams::NumHighParallaxPointsForKeyframe())
    {
      return transform;
    }
  }

  return KeyframeTransform::Invalid(frame1, frame2);
}

int KeyframeTracker::ComputePointParallaxes(const std::vector<cv::Point2f>& points1,
                                            const std::vector<cv::Point2f>& points2, const cv::Mat& R12,
                                            const cv::Mat& K, double min_parallax, std::vector<double>& parallaxes)
{
  int num_high_parallax = 0;
  parallaxes.resize(points1.size());
  auto H_rot_only = R12.inv(); // Quite spooky that we need to invert this: Need to take a better look at this
  cv::Mat K_inv = K.inv();
  for (int i = 0; i < points1.size(); ++i)
  {
    cv::Mat point1 = (cv::Mat_<double>(3, 1) << points1[i].x, points1[i].y, 1.);
    cv::Mat point2 = (cv::Mat_<double>(3, 1) << points2[i].x, points2[i].y, 1.);
    cv::Mat point2_comp = K * H_rot_only * K_inv * point1;

    point2_comp /= point2_comp.at<double>(2, 0);  // Normalize homogeneous coordinate
    cv::Mat point2_delta = point2 - point2_comp;
    double dist = std::sqrt(point2_delta.dot(point2_delta));  // This works because last coordinate is zero
    parallaxes[i] = dist;
    if (dist > min_parallax)
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

std::vector<KeyframeTransform> KeyframeTracker::GetGoodKeyframeTransforms() const
{
  std::vector<KeyframeTransform> transforms;
  if (GetNumberOfGoodTransforms() > 0)
  {
    int i = GetMostRecentBadTransformIdx();
    std::copy(keyframe_transforms_.begin() + i + 1, keyframe_transforms_.end(), std::back_inserter(transforms));
  }
  return transforms;
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

bool KeyframeTracker::HaveEnoughMatches(const std::shared_ptr<Frame>& frame1,
                                              const std::shared_ptr<Frame>& frame2,
                                              const std::vector<std::shared_ptr<Track>>& tracks)
{
  std::vector<std::shared_ptr<Track>> valid_tracks;
  OnlyValidTracks(frame1, frame2, tracks, valid_tracks);

  std::vector<cv::Point2f> points1;
  std::vector<cv::Point2f> points2;
  GetPointsSafe(frame1, frame2, valid_tracks, points1, points2);
  return points1.size() >= 8 && points2.size() >= 8;
}

void KeyframeTracker::OnlyValidTracks(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                                      const std::vector<std::shared_ptr<Track>>& tracks,
                                      std::vector<std::shared_ptr<Track>>& valid_tracks)
{
  for (auto& track : tracks)
  {
    if (track->features.empty())
    {
      continue;
    }
    bool have_point1 = false;
    bool have_point2 = false;
    int num_features = static_cast<int>(track->features.size());
    for (int i = num_features - 1; i >= 0 && !(have_point1 && have_point2); --i)
    {
      if (track->features[i]->frame->id == frame1->id)
      {
        have_point1 = true;
      }
      if (track->features[i]->frame->id == frame2->id)
      {
        have_point2 = true;
      }
    }
    if (have_point1 && have_point2)
    {
      valid_tracks.push_back(track);
    }
  }
}

void KeyframeTracker::UpdateTrackInlierOutlierCounts(const std::vector<std::shared_ptr<Track>>& tracks,
                                                     const std::vector<uchar>& inlier_mask)
{
  assert(tracks.size() == inlier_mask.size());
  for (int i = 0; i < tracks.size(); ++i)
  {
    if (inlier_mask[i])
    {
      tracks[i]->inlier_count++;
    }
    else
    {
      tracks[i]->outlier_count++;
    }
  }
}
