#include "keyframe_tracker.h"
#include "global_params.h"
#include <opencv2/calib3d.hpp>
#include <Initializer.h>

void KeyframeTracker::AddFrameSafe(const std::shared_ptr<Frame>& frame2, const std::vector<std::shared_ptr<Track>>& tracks)
{
  auto frame1 = keyframe_transforms_.back().frame2;
  if (SafeToAddFrame(frame1, frame2, tracks))
  {
    AddFrame(frame1, frame2, tracks);
  }
  else
  {
    keyframe_transforms_.emplace_back(frame1, frame2, cv::Mat(), -1, -1, -1);
  }
}

void KeyframeTracker::AddFrame(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                          const std::vector<std::shared_ptr<Track>>& tracks, bool init)
{
  std::vector<std::shared_ptr<Track>> valid_tracks;
  OnlyValidTracks(frame1, frame2, tracks, valid_tracks);
  std::vector<uchar> inlier_mask;
  keyframe_transforms_.push_back(MakeKeyframeTransform(frame1, frame2, tracks, inlier_mask, init));

  UpdateTrackInlierOutlierCounts(valid_tracks, inlier_mask);
}

KeyframeTracker::KeyframeTracker(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                                 const std::shared_ptr<Frame>& frame3,
                                 const std::vector<std::shared_ptr<Track>>& tracks)
{
  std::vector<std::shared_ptr<Track>> valid_tracks;
  OnlyValidTracks(frame1, frame2, tracks, valid_tracks);
  std::vector<uchar> inlier_mask;
  auto transform_1 = MakeKeyframeTransform(frame1, frame2, valid_tracks, inlier_mask, true);
  UpdateTrackInlierOutlierCounts(valid_tracks, inlier_mask);

  valid_tracks.clear();
  inlier_mask.clear();

  OnlyValidTracks(frame2, frame3, tracks, valid_tracks);
  auto transform_2 = MakeKeyframeTransform(frame2, frame3, valid_tracks, inlier_mask);
  UpdateTrackInlierOutlierCounts(valid_tracks, inlier_mask);

  // TODO Calculate R and t in case homography is best

  keyframe_transforms_.push_back(transform_1);
  keyframe_transforms_.push_back(transform_2);
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

KeyframeTransform KeyframeTracker::MakeKeyframeTransform(const std::shared_ptr<Frame>& frame1,
                                                         const std::shared_ptr<Frame>& frame2,
                                                         const std::vector<std::shared_ptr<Track>>& tracks,
                                                         std::vector<uchar>& inlier_mask,
                                                         bool init)
{
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

  if (transform.FundamentalMatGood()) {
    // Essential matrix and recovered R and t represent transformation from cam 2 to cam 1 in cam 2 frame
    // As such, we compute it in reverse order, so that we obtain the transformation from frame1 to frame2 in frame1 frame
    auto E = cv::findEssentialMat(points2, points1, K, cv::RANSAC);
    cv::Mat R;
    std::vector<double> t;
    cv::recoverPose(E, points2, points1, K, R, t);

    EssentialMatrixDecompositionResult result(E, R, t);
    transform.essential_matrix_decomposition_result = boost::optional<EssentialMatrixDecompositionResult>(result);
  }

  return transform;
}

std::vector<KeyframeTransform> KeyframeTracker::GetGoodKeyframeTransforms() const
{
  std::vector<KeyframeTransform> transforms;
  int i;
  for (i = static_cast<int>(keyframe_transforms_.size()) - 1;
       i > keyframe_transforms_.size() - GlobalParams::NumGoodKeyframesForInitialization() &&
       keyframe_transforms_[i].FundamentalMatGood();
       --i)
  {
  }
  std::copy(keyframe_transforms_.begin() + i + 1, keyframe_transforms_.end(), std::back_inserter(transforms));
  return transforms;
}

bool KeyframeTracker::GoodForInitialization()
{
  if (keyframe_transforms_.size() < GlobalParams::NumGoodKeyframesForInitialization())
  {
    return false;
  }

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

bool KeyframeTracker::SafeToAddFrame(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                                     const std::vector<std::shared_ptr<Track>>& tracks)
{
  std::vector<std::shared_ptr<Track>> valid_tracks;
  OnlyValidTracks(frame1, frame2, tracks, valid_tracks);

  std::vector<cv::Point2f> points1;
  std::vector<cv::Point2f> points2;
  GetPointsSafe(frame1, frame2, valid_tracks, points1, points2);
  return points1.size() >= 8 && points2.size() >= 8;
}

bool KeyframeTracker::SafeToInitialize(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                                       const std::shared_ptr<Frame>& frame3,
                                       const std::vector<std::shared_ptr<Track>>& tracks)
{
  return SafeToAddFrame(frame1, frame2, tracks) && SafeToAddFrame(frame2, frame3, tracks);
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
