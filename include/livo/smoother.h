#ifndef ORB_TEST_SRC_SMOOTHER_H_
#define ORB_TEST_SRC_SMOOTHER_H_

#include "landmark.h"
#include "pose3_stamped.h"
#include "point3.h"
#include "feature.h"
#include "track.h"

#include <memory>
#include <vector>
#include <map>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> SmartFactor;

class Smoother
{
private:
  gtsam::ISAM2 isam2;
  std::map<int, SmartFactor::shared_ptr> smart_factors_;

public:
  Smoother();
  void Initialize(const std::vector<std::shared_ptr<Frame>>& frames, const std::vector<shared_ptr<Track>>& tracks,
                  std::vector<Pose3Stamped>& pose_estimates, std::vector<Point3>& landmark_estimates);

  Pose3Stamped Update(const shared_ptr<Frame>& frame, const std::vector<shared_ptr<Track>>& tracks,
                      vector<Point3>& landmark_estimates);
};

#endif
