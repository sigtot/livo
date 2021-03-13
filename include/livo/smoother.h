#ifndef ORB_TEST_SRC_SMOOTHER_H_
#define ORB_TEST_SRC_SMOOTHER_H_

#include "landmark.h"
#include "pose3_stamped.h"
#include "point3.h"
#include "feature.h"

#include <memory>
#include <vector>
#include <map>
#include <gtsam/nonlinear/ISAM2.h>

class Smoother
{
private:
  gtsam::ISAM2 isam2;

public:
  Smoother();
  void Initialize(const std::vector<std::shared_ptr<Frame>>& frames, const std::vector<std::vector<Feature>>& tracks,
                   std::vector<Pose3Stamped>& pose_estimates, std::vector<Point3>& landmark_estimates);
};

#endif
