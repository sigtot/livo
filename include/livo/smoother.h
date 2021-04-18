#ifndef ORB_TEST_SRC_SMOOTHER_H_
#define ORB_TEST_SRC_SMOOTHER_H_

#include "landmark.h"
#include "pose3_stamped.h"
#include "point3.h"
#include "feature.h"
#include "track.h"
#include "imu_queue.h"
#include "keyframe_transform.h"

#include <memory>
#include <vector>
#include <map>

// GTSAM forward declarations to speed up compilation time
namespace gtsam
{
class Cal3_S2;
class ISAM2;
class NonlinearFactorGraph;
class Values;
class Marginals;
class Pose3;

namespace noiseModel
{
class Isotropic;
}

class TangentPreintegration;
typedef TangentPreintegration PreintegrationType;

class SmartProjectionParams;

template <class T>
class SmartProjectionPoseFactor;

template <typename A1, typename A2, typename T>
class RangeFactor;
}  // namespace gtsam

typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> SmartFactor;

enum BackendStatus
{
  kUninitialized = 0,
  kIMUInitialized,
  kLandmarksInitialized,
};

class Smoother
{
private:
  gtsam::ISAM2* isam2;
  gtsam::NonlinearFactorGraph* graph_;
  gtsam::Values* values_;
  gtsam::Marginals* marginals_ = nullptr;
  boost::shared_ptr<gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3, double>> range_factor_;
  boost::shared_ptr<gtsam::Cal3_S2> K_;
  boost::shared_ptr<gtsam::Pose3> body_p_cam_;
  boost::shared_ptr<gtsam::Pose3> body_p_imu_;
  boost::shared_ptr<gtsam::noiseModel::Isotropic> feature_noise_;

  // Bookkeeping
  std::map<int, boost::shared_ptr<SmartFactor>> smart_factors_;
  std::map<int, double> added_frame_timestamps_;  // Map for looking up timestamps of frames added to the optimization
  std::map<int, std::shared_ptr<Track>> added_tracks_;
  std::shared_ptr<gtsam::Pose3> init_pose_;

  int last_frame_id_added_ = -1;
  std::shared_ptr<IMUQueue> imu_queue_;
  std::shared_ptr<gtsam::PreintegrationType> imu_measurements_;
  BackendStatus status_ = kUninitialized;

  void WaitForAndIntegrateIMU(double timestamp1, double timestamp2);
  void PublishReprojectionErrorImages();
  void PublishNewReprojectionErrorImage(const gtsam::Values& values, const shared_ptr<Frame>& frame);
  void UpdateSmartFactorParams(const gtsam::SmartProjectionParams& params);
  std::shared_ptr<gtsam::PreintegrationType> MakeIMUIntegrator();

public:
  explicit Smoother(std::shared_ptr<IMUQueue> imu_queue);
  void InitializeLandmarks(
      std::vector<KeyframeTransform> keyframe_transforms, const std::vector<std::shared_ptr<Track>>& tracks,
      const boost::optional<std::pair<std::shared_ptr<Frame>, std::shared_ptr<Frame>>>& frames_for_imu_init,
      std::vector<Pose3Stamped>& pose_estimates, std::map<int, Point3>& landmark_estimates);

  Pose3Stamped AddKeyframe(const KeyframeTransform& keyframe_transform,
                           const std::vector<std::shared_ptr<Track>>& tracks, std::vector<Pose3Stamped>& pose_estimates,
                           std::map<int, Point3>& landmark_estimates);

  Pose3Stamped AddFrame(const std::shared_ptr<Frame>& frame, const std::vector<std::shared_ptr<Track>>& tracks,
                        std::vector<Pose3Stamped>& pose_estimates, std::map<int, Point3>& landmark_estimates);

  BackendStatus GetStatus();
  void InitializeIMU(const std::vector<KeyframeTransform>& keyframe_transforms,
                     std::vector<Pose3Stamped>& pose_estimates, std::map<int, Point3>& landmark_estimates);
  void GetPoseEstimates(std::vector<Pose3Stamped>& pose_estimates);
  void GetLandmarkEstimates(std::map<int, Point3>& landmark_estimates);
  int GetLastFrameId() const;
  void RemoveBadLandmarks();
};

#endif
