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
#include "between_transform_provider.h"
#include "keyframe_timestamps.h"

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
class Diagonal;
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
  int last_keyframe_id_ = -1;
  std::map<int, std::shared_ptr<Frame>> added_frames_;
  boost::shared_ptr<gtsam::noiseModel::Diagonal> between_noise_;
  boost::shared_ptr<gtsam::noiseModel::Diagonal> between_noise_keyframe_;
  boost::shared_ptr<gtsam::noiseModel::Isotropic> feature_noise_;
  boost::shared_ptr<gtsam::noiseModel::Base> range_noise_;
  boost::shared_ptr<gtsam::noiseModel::mEstimator::Base> feature_m_estimator_;
  boost::shared_ptr<gtsam::Pose3> body_p_cam_;

  // Dependencies
  std::shared_ptr<BetweenTransformProvider> between_transform_provider_;
  KeyframeTimestamps keyframe_timestamps_;
  GraphManager graph_manager_;
  IMUIntegrator imu_integrator_;

  gtsam::Point3 CalculatePointEstimate(const gtsam::Pose3& pose, const gtsam::Point2& pt, double depth) const;
  void InitializeProjLandmarkWithDepth(int lmk_id, int frame_id, double timestamp, const gtsam::Point2& pt,
                                       double depth, const gtsam::Pose3& init_pose);
  bool TryInitializeProjLandmarkByTriangulation(int lmk_id, int frame_id, double timestamp,
                                                const std::shared_ptr<Track>& track);
  void InitializeStructurelessLandmark(int lmk_id, int frame_id, double timestamp, const gtsam::Point2& pt);
  void TryAddBetweenConstraint(int frame_id_1, int frame_id_2, double timestamp_1, double timestamp_2,
                               const boost::shared_ptr<gtsam::noiseModel::Base>& noise);
  double CalculateParallax(const std::shared_ptr<Track>& track) const;

public:
  explicit NewSmoother(std::shared_ptr<IMUQueue> imu_queue);

  void Initialize(const std::shared_ptr<Frame>& frame,
                  const boost::optional<std::pair<double, double>>& imu_gravity_alignment_timestamps = boost::none);
  void AddKeyframe(const std::shared_ptr<Frame>& frame, bool is_keyframe);

  void UpdateTrackParallaxes(const std::shared_ptr<Frame>& frame);

  void GetPoses(std::map<int, Pose3Stamped>& poses) const;
  void GetLandmarks(std::map<int, LandmarkResult>& landmarks) const;

  bool IsInitialized() const;
};

#endif  // ORB_TEST_SRC_NEW_SMOOTHER_H_
