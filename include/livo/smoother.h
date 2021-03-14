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

// GTSAM forward declarations to speed up compilation time
namespace gtsam
{
class Cal3_S2;
class ISAM2;
template <class T>
class SmartProjectionPoseFactor;
}  // namespace gtsam

typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> SmartFactor;

class Smoother
{
private:
  gtsam::ISAM2* isam2;
  std::map<int, boost::shared_ptr<SmartFactor>> smart_factors_;

public:
  Smoother();
  void Initialize(const std::vector<std::shared_ptr<Frame>>& frames, const std::vector<std::shared_ptr<Track>>& tracks,
                  std::vector<Pose3Stamped>& pose_estimates, std::vector<Point3>& landmark_estimates);

  Pose3Stamped Update(const std::shared_ptr<Frame>& frame, const std::vector<std::shared_ptr<Track>>& tracks,
                      std::vector<Point3>& landmark_estimates);
};

#endif
