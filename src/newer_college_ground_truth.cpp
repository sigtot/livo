#include "newer_college_ground_truth.h"
#include "gtsam_conversions.h"

#include <map>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <array>
#include <sstream>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Lie.h>

double THRESH = 0.4;

Pose3 NewerCollegeGroundTruth::At(double timestamp)
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

std::multimap<double, Pose3> NewerCollegeGroundTruth::GetAllPoses()
{
  return GetInstance().gt_;
}

NewerCollegeGroundTruth& NewerCollegeGroundTruth::GetInstance()
{
  static NewerCollegeGroundTruth instance;
  return instance;
}

void NewerCollegeGroundTruth::LoadFromFile(const std::string& filename)
{
  std::ifstream f;
  f.open(filename, std::ios::in);
  std::string line;
  bool past_first_line = false;
  while (std::getline(f, line))
  {
    if (!past_first_line)
    {
      past_first_line = true;
      continue;
    }
    std::stringstream ss(line);
    int secs, nsecs;
    double data[7];
    char c;
    ss >> secs >> c >> nsecs >> c;
    int i = 0;
    while ((ss >> data[i++] >> c) && (c == ','))
      ;
    Pose3 pose{ .point = { .x = data[0], .y = data[1], .z = data[2] },
                .rot = { .x = data[3], .y = data[4], .z = data[5], .w = data[6] } };
    double ts = double(secs) + double(nsecs) * 1e-9;
    GetInstance().gt_.insert({ ts, pose });
  }
  f.close();
}

Pose3 NewerCollegeGroundTruth::Interpolate(Pose3 from_pose, Pose3 to_pose, double from_ts, double to_ts,
                                           double target_ts)
{
  assert(from_ts < to_ts);
  assert(from_ts < target_ts);
  assert(target_ts < to_ts);
  gtsam::Pose3 gtsam_from_pose = ToGtsamPose(from_pose);
  gtsam::Pose3 gtsam_to_pose = ToGtsamPose(to_pose);
  double t = (target_ts - from_ts) / (to_ts - from_ts);

  // X o Exp(t * Log(between(X, Y)))
  return ToPose(gtsam::interpolate(gtsam_from_pose, gtsam_to_pose, t));
}
