#ifndef ORB_TEST_SRC_GROUND_TRUTH_PROVIDER_H_
#define ORB_TEST_SRC_GROUND_TRUTH_PROVIDER_H_

#include "pose3.h"
#include <utility>
#include <vector>
#include <map>
#include <string>

class GroundTruthProvider
{
public:
  virtual void LoadFromFile(std::multimap<double, Pose3>& gt) const = 0;
};

#endif
