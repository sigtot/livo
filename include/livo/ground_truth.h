#ifndef ORB_TEST_SRC_GROUND_TRUTH_H_
#define ORB_TEST_SRC_GROUND_TRUTH_H_

#include "pose3.h"
#include "ground_truth_provider.h"
#include <vector>
#include <map>
#include <string>

class GroundTruth
{
private:
  std::multimap<double, Pose3> gt_;

  GroundTruth() = default;
  static GroundTruth& GetInstance();
  static Pose3 Interpolate(Pose3 from_pose, Pose3 to_pose, double from_ts, double to_ts, double target_ts);

public:
  GroundTruth(GroundTruth const&) = delete;
  void operator=(GroundTruth const&) = delete;

  static Pose3 At(double timestamp);
  static std::multimap<double, Pose3> GetAllPoses();
  static void Load(const GroundTruthProvider& gt_provider);
};

#endif
