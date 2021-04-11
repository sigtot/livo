#include "ground_truth.h"
#include "gtsam_conversions.h"

#include <map>
#include <iostream>
#include <iomanip>
#include <array>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Lie.h>

double THRESH = 0.4;

Pose3 GroundTruth::At(double timestamp)
{
  // Logarithmic in size of container. Code is a bit messy. Should clean it up.
  auto gt_it = GetInstance().gt_.upper_bound(timestamp);
  if (gt_it != GetInstance().gt_.end())
  {
    auto ts_after = gt_it->first;
    auto pose_after = gt_it->second;
    --gt_it;
    if (gt_it != GetInstance().gt_.end() && std::abs(gt_it->first - timestamp) < THRESH)
    {
      auto ts_before = gt_it->first;
      auto pose_before = gt_it->second;
      return Interpolate(pose_before, pose_after, ts_before, ts_after, timestamp);
    }
  }

  std::cout << "Did not find any ground truth pose with timestamp close to " << std::setprecision(20) << timestamp
            << std::endl;
  exit(1);
}

std::multimap<double, Pose3> GroundTruth::GetAllPoses()
{
  return GetInstance().gt_;
}

Pose3 GroundTruth::Interpolate(Pose3 from_pose, Pose3 to_pose, double from_ts, double to_ts,
                                           double target_ts)
{
  assert(from_ts < to_ts);
  assert(from_ts <= target_ts);
  assert(target_ts <= to_ts);
  gtsam::Pose3 gtsam_from_pose = ToGtsamPose(from_pose);
  gtsam::Pose3 gtsam_to_pose = ToGtsamPose(to_pose);
  double t = (target_ts - from_ts) / (to_ts - from_ts);

  // X o Exp(t * Log(between(X, Y)))
  return ToPose(gtsam::interpolate(gtsam_from_pose, gtsam_to_pose, t));
}

void GroundTruth::Load(const GroundTruthProvider& gt_provider)
{
  gt_provider.LoadFromFile(GetInstance().gt_);
}

GroundTruth& GroundTruth::GetInstance()
{
  static GroundTruth instance;
  return instance;
}
