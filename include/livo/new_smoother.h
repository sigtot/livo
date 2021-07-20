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
#include "time_offset_provider.h"
#include "refined_camera_matrix_provider.h"
#include "lidar_depth_result.h"

#include <utility>
#include <map>

#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>
#include <Eigen/Core>
#include <backend/frontend_result.h>
#include <backend/feature.h>
#include <backend/track.h>

namespace gtsam
{
class Cal3_S2;
class Pose3;
class TriangulationResult;

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

class FeatureExtractor;

class NewSmoother
{
private:
  bool initialized_ = false;
  boost::shared_ptr<gtsam::Cal3_S2> K_;
  int last_frame_id_ = -1;
  int last_keyframe_id_ = -1;
  std::map<int, backend::FrontendResult> added_frames_;

  boost::shared_ptr<gtsam::noiseModel::Diagonal> between_noise_;
  boost::shared_ptr<gtsam::noiseModel::Diagonal> between_noise_keyframe_;
  boost::shared_ptr<gtsam::noiseModel::Isotropic> feature_noise_;
  boost::shared_ptr<gtsam::noiseModel::Base> range_noise_;
  boost::shared_ptr<gtsam::noiseModel::mEstimator::Base> feature_m_estimator_;
  boost::shared_ptr<gtsam::Pose3> body_p_cam_;

  // Dependencies
  std::shared_ptr<BetweenTransformProvider> between_transform_provider_;
  std::shared_ptr<TimeOffsetProvider> lidar_time_offset_provider_;
  KeyframeTimestamps keyframe_timestamps_;
  GraphManager graph_manager_;
  IMUIntegrator imu_integrator_;
  std::shared_ptr<FeatureExtractor> feature_extractor_ = nullptr;

  gtsam::Point3 CalculatePointEstimate(const gtsam::Pose3& pose, const gtsam::Point2& pt, double depth) const;
  void InitializeProjLandmarkWithDepth(int lmk_id, int frame_id, double timestamp, const gtsam::Point2& pt,
                                       LidarDepthResult depth_result, const gtsam::Pose3& init_pose);
  bool TryInitializeProjLandmarkByTriangulation(int lmk_id, int frame_id, double timestamp,
                                                const backend::Track& track);
  void InitializeStructurelessLandmark(int lmk_id, int frame_id, double timestamp, const gtsam::Point2& pt);
  void TryAddBetweenConstraint(int frame_id_1, int frame_id_2, double timestamp_1, double timestamp_2,
                               const boost::shared_ptr<gtsam::noiseModel::Base>& noise);
  void DoExtraUpdateSteps(int steps);
  void RemoveUntrackedFramesFromBookkeeping();
  gtsam::TriangulationResult TriangulateTrack(const backend::Track& track) const;
  void InitializeNewLandmarks(const std::vector<backend::Track>& new_tracks, int frame_id,
                              const gtsam::Pose3& pred_pose, double timestamp_for_values);
  void AddLandmarkObservations(const std::vector<backend::Track>& existing_tracks, int frame_id,
                               const gtsam::Pose3& pred_pose, double timestamp_for_values);
  void PublishHighDeltaTrackImage();

public:
  NewSmoother(std::shared_ptr<IMUQueue> imu_queue, std::shared_ptr<TimeOffsetProvider> lidar_time_offset_provider,
              const std::shared_ptr<RefinedCameraMatrixProvider>& refined_camera_matrix_provider,
              const std::shared_ptr<BetweenTransformProvider>& between_transform_provider);

  void SetFrontend(std::shared_ptr<FeatureExtractor> frontend);

  void Initialize(const backend::FrontendResult& frame,
                  const boost::optional<std::pair<double, double>>& imu_gravity_alignment_timestamps = boost::none);
  void AddKeyframe(const backend::FrontendResult& frontend_result, bool is_keyframe);

  void UpdateTrackParallaxes(const std::shared_ptr<Frame>& frame);

  double CalculateParallax(const std::shared_ptr<Track>& track) const;

  void GetPoses(std::map<int, Pose3Stamped>& poses) const;
  void GetLandmarks(std::map<int, LandmarkResult>& landmarks) const;
  boost::optional<Pose3Stamped> GetLatestLidarPose();
  bool IsLandmarkTracked(int lmk_id) const;

  bool IsInitialized() const;
};

#endif  // ORB_TEST_SRC_NEW_SMOOTHER_H_
