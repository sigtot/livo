#ifndef ORB_TEST_SRC_EUROC_GROUND_TRUTH_PROVIDER_H_
#define ORB_TEST_SRC_EUROC_GROUND_TRUTH_PROVIDER_H_

#include "ground_truth_provider.h"
class EurocGroundTruthProvider : public GroundTruthProvider
{
private:
  std::string filename_;

public:
  explicit EurocGroundTruthProvider(std::string filename);
  void LoadFromFile(std::multimap<double, Pose3>& gt) const override;
};

#endif
