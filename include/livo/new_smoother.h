#ifndef ORB_TEST_SRC_NEW_SMOOTHER_H_
#define ORB_TEST_SRC_NEW_SMOOTHER_H_

#include "frame.h"
#include "track.h"
#include "graph_manager.h"
#include "imu_integrator.h"
#include "imu_queue.h"

#include <vector>
#include <utility>
#include <map>

#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>
#include <Eigen/Core>

namespace gtsam
{
class Cal3_S2;
class Pose3;

namespace noiseModel
{
class Isotropic;
class Diagonal;
}  // namespace noiseModel
namespace imuBias
{
class ConstantBias;
}
}  // namespace gtsam

class NewSmoother
{
private:
  bool initialized_ = false;
  boost::shared_ptr<gtsam::Cal3_S2> K_;
  int last_frame_id_ = -1;
  std::map<int, std::shared_ptr<Frame>> added_frames_;
  boost::shared_ptr<gtsam::noiseModel::Isotropic> feature_noise_;
  boost::shared_ptr<gtsam::noiseModel::Isotropic> range_noise_;

  GraphManager graph_manager_;
  IMUIntegrator imu_integrator_;


public:
  NewSmoother(std::shared_ptr<IMUQueue> imu_queue);
  void Initialize(const std::vector<std::shared_ptr<Frame>>& frames,
                  const std::vector<std::shared_ptr<Track>>& tracks,
                  const std::pair<double, double>& imu_gravity_alignment_timestamps);
};

#endif  // ORB_TEST_SRC_NEW_SMOOTHER_H_
