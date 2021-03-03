#ifndef ORB_TEST_SRC_NEWER_COLLEGE_GROUND_TRUTH_H_
#define ORB_TEST_SRC_NEWER_COLLEGE_GROUND_TRUTH_H_

#include "pose3.h"
#include <vector>
#include <map>

class NewerCollegeGroundTruth
{
public:
  static Pose3 At(double timestamp);
  static std::map<double, Pose3> GetAllPoses();
};

#endif
