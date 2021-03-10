#ifndef ORB_TEST_SRC_NEWER_COLLEGE_GROUND_TRUTH_H_
#define ORB_TEST_SRC_NEWER_COLLEGE_GROUND_TRUTH_H_

#include "pose3.h"
#include <vector>
#include <map>
#include <string>

class NewerCollegeGroundTruth
{
private:
  std::multimap<double, Pose3> gt_;

  NewerCollegeGroundTruth() = default;
  static NewerCollegeGroundTruth& GetInstance();

public:
  NewerCollegeGroundTruth(NewerCollegeGroundTruth const&) = delete;
  void operator=(NewerCollegeGroundTruth const&) = delete;

  static Pose3 At(double timestamp);
  static std::multimap<double, Pose3> GetAllPoses();
  static void LoadFromFile(const std::string& filename);
};

#endif