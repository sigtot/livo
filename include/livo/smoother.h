#ifndef ORB_TEST_SRC_SMOOTHER_H_
#define ORB_TEST_SRC_SMOOTHER_H_

#include "landmark.h"
#include "pose3_stamped.h"
#include "point3.h"
#include "feature.h"
#include "track.h"
#include "imu_queue.h"

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

class TangentPreintegration;
typedef TangentPreintegration PreintegrationType;

template <class T>
class SmartProjectionPoseFactor;
}  // namespace gtsam

typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> SmartFactor;

enum BackendStatus {
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
  std::map<int, boost::shared_ptr<SmartFactor>> smart_factors_;
  std::shared_ptr<IMUQueue> imu_queue_;
  std::shared_ptr<gtsam::PreintegrationType> imu_measurements_;
  BackendStatus status_ = kUninitialized;

public:
  explicit Smoother(std::shared_ptr<IMUQueue> imu_queue);
  void InitializeLandmarks(const std::vector<std::shared_ptr<Frame>>& frames, const std::vector<std::shared_ptr<Track>>& tracks,
                  std::vector<Pose3Stamped>& pose_estimates, std::vector<Point3>& landmark_estimates);

  void InitIMUOnly(const std::vector<std::shared_ptr<Frame>>& frames, const std::vector<std::shared_ptr<Track>>& tracks,
                   std::vector<Pose3Stamped>& pose_estimates);

  Pose3Stamped Update(const std::shared_ptr<Frame>& frame, const std::vector<std::shared_ptr<Track>>& tracks,
                      std::vector<Point3>& landmark_estimates);
  void InitIMU(const vector<shared_ptr<Frame>>& vector);

  BackendStatus GetStatus();
};

#endif
