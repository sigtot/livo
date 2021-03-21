#include "keyframe_tracker.h"
#include "global_params.h"
#include <opencv2/calib3d.hpp>
#include <Initializer.h>

void KeyframeTracker::AddFrame(const std::shared_ptr<Frame>& frame2, const std::vector<std::shared_ptr<Track>>& tracks)
{
  auto frame1 = keyframe_transforms_.back().frame2;
  if (SafeToAddFrame(frame2, tracks))
  {
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;
    GetPoints(frame1, frame2, tracks, points1, points2);

    keyframe_transforms_.push_back(MakeKeyframeTransform(points1, points2, frame1, frame2));
  }
  else
  {
    keyframe_transforms_.emplace_back(frame1, frame2, cv::Mat(), -1, -1, -1);
  }
}

KeyframeTracker::KeyframeTracker(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                                 const std::vector<std::shared_ptr<Track>>& tracks)
{
  std::vector<cv::Point2f> points1;
  std::vector<cv::Point2f> points2;
  GetPoints(frame1, frame2, tracks, points1, points2, true);

  keyframe_transforms_.push_back(MakeKeyframeTransform(points1, points2, frame1, frame2));
}

void KeyframeTracker::GetPoints(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                                const std::vector<std::shared_ptr<Track>>& tracks, std::vector<cv::Point2f>& points1,
                                std::vector<cv::Point2f>& points2, bool init)
{
  for (auto& track : tracks)
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

KeyframeTransform KeyframeTracker::MakeKeyframeTransform(const std::vector<cv::Point2f>& points1,
                                                         const std::vector<cv::Point2f>& points2,
                                                         const std::shared_ptr<Frame>& frame1,
                                                         const std::shared_ptr<Frame>& frame2)
{
  std::vector<uchar> inlier_mask;
  auto F = findFundamentalMat(points1, points2, cv::FM_RANSAC, 3., 0.99, inlier_mask);
  auto H = findHomography(points1, points2, cv::RANSAC, 3);

  std::vector<bool> F_check_inliers;
  double S_F = ORB_SLAM::CheckFundamental(F, F_check_inliers, points1, points2);

  std::vector<bool> H_check_inliers;
  double S_H = ORB_SLAM::CheckHomography(H, H.inv(), H_check_inliers, points1, points2);

  double R_H = S_H / (S_H + S_F);

  std::cout << "R_H = " << R_H << ", S_H = " << S_H << ", S_F = " << S_F << std::endl;

  return KeyframeTransform(frame1, frame2, F, S_H, S_F, R_H);
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

bool KeyframeTracker::SafeToAddFrame(const std::shared_ptr<Frame>& frame2,
                                     const std::vector<std::shared_ptr<Track>>& tracks)
{
  auto frame1 = keyframe_transforms_.back().frame2;
  std::vector<cv::Point2f> points1;
  std::vector<cv::Point2f> points2;
  GetPointsSafe(frame1, frame2, tracks, points1, points2);
  return points1.size() >= 8 && points2.size() >= 8;
}

bool KeyframeTracker::SafeToInitialize(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                                       const std::vector<std::shared_ptr<Track>>& tracks)
{
  std::vector<cv::Point2f> points1;
  std::vector<cv::Point2f> points2;
  GetPointsSafe(frame1, frame2, tracks, points1, points2);
  return points1.size() >= 8 && points2.size() >= 8;
}
