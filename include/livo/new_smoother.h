#ifndef ORB_TEST_SRC_NEW_SMOOTHER_H_
#define ORB_TEST_SRC_NEW_SMOOTHER_H_

#include "frame.h"
#include "track.h"
#include "graph_manager.h"
#include "imu_integrator.h"
#include "imu_queue.h"
#include "pose3_stamped.h"
#include "point3.h"
#include "landmark_result.h"

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
class Base;
class Isotropic;
namespace mEstimator
{
class Base;
}
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
  boost::shared_ptr<gtsam::noiseModel::Base> range_noise_;
  boost::shared_ptr<gtsam::noiseModel::mEstimator::Base> feature_m_estimator_;
  boost::shared_ptr<gtsam::Pose3> body_p_cam_;

  GraphManager graph_manager_;
  IMUIntegrator imu_integrator_;

  gtsam::Point3 CalculatePointEstimate(const gtsam::Pose3& pose, const gtsam::Point2& pt, double depth) const;
  void InitializeLandmarkWithDepth(int lmk_id, int frame_id, double timestamp, const gtsam::Point2& pt, double depth,
                                   const gtsam::Pose3& init_pose);
  void InitializeStructurelessLandmark(int lmk_id, int frame_id, double timestamp, const gtsam::Point2& pt);

public:
  explicit NewSmoother(std::shared_ptr<IMUQueue> imu_queue);

  void Initialize(const std::shared_ptr<Frame>& frame,
                  const boost::optional<std::pair<double, double>>& imu_gravity_alignment_timestamps = boost::none);
  void AddFrame(const std::shared_ptr<Frame>& frame);
  void AddKeyframe(const std::shared_ptr<Frame>& frame);

  void GetPoses(std::map<int, Pose3Stamped>& poses) const;
  void GetLandmarks(std::map<int, LandmarkResult>& landmarks) const;

  bool IsInitialized() const;
};

#endif  // ORB_TEST_SRC_NEW_SMOOTHER_H_
