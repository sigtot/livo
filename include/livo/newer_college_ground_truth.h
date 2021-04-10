#ifndef ORB_TEST_SRC_NEWER_COLLEGE_GROUND_TRUTH_H_
#define ORB_TEST_SRC_NEWER_COLLEGE_GROUND_TRUTH_H_

#include "pose3.h"
#include "ground_truth.h"
#include <vector>
#include <map>
#include <string>

class NewerCollegeGroundTruth : public GroundTruthProvider
{
private:
  std::string filename_;

public:
  NewerCollegeGroundTruth(std::string filename);
  void LoadFromFile(std::multimap<double, Pose3>& gt) const override;
};

#endif
