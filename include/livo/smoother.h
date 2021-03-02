#ifndef ORB_TEST_SRC_SMOOTHER_H_
#define ORB_TEST_SRC_SMOOTHER_H_

#include "landmark.h"
#include "pose3_stamped.h"
#include "point3.h"
#include <memory>
#include <vector>

class Smoother {
 public:
  static void SmoothBatch(
      const std::vector<std::shared_ptr<Frame>>& frames,
      const std::vector<std::shared_ptr<Landmark>>& landmarks,
      std::vector<Pose3Stamped>& pose_estimates,
      std::vector<Point3>& landmark_estimates);
};

#endif
