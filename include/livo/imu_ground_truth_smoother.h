#ifndef ORB_TEST_SRC_IMU_GROUND_TRUTH_SMOOTHER_H_
#define ORB_TEST_SRC_IMU_GROUND_TRUTH_SMOOTHER_H_

#include "imu_queue.h"
#include "pose3_stamped.h"

#include <memory>
namespace gtsam
{
class ISAM2;
class Values;
class NonlinearFactorGraph;
class Pose3;

class TangentPreintegration;
typedef TangentPreintegration PreintegrationType;
}  // namespace gtsam

class IMUGroundTruthSmoother
{
public:
  explicit IMUGroundTruthSmoother(const std::shared_ptr<IMUQueue>& imu_queue);
  void Initialize(double stamp1, double stamp2, std::vector<Pose3Stamped>& pose_estimates);
  void IMUPredict(double stamp, std::vector<Pose3Stamped>& pose_estimates);
  void GroundTruthUpdate(double stamp, std::vector<Pose3Stamped>& pose_estimates);
  bool IsInitialized();

private:
  std::shared_ptr<gtsam::ISAM2> isam2_;
  std::shared_ptr<gtsam::NonlinearFactorGraph> graph_;
  std::shared_ptr<gtsam::Values> values_;

  boost::shared_ptr<gtsam::Pose3> body_p_imu_;
  std::shared_ptr<IMUQueue> imu_queue_;
  std::shared_ptr<gtsam::PreintegrationType> imu_measurements_;
  std::map<int, double> added_frame_timestamps_;  // Map for looking up timestamps of frames added to the optimization
  int last_frame_id_added_ = -1;
  bool initialized_ = false;

  std::shared_ptr<gtsam::PreintegrationType> MakeIMUIntegrator();
  void WaitForAndIntegrateIMU(double timestamp1, double timestamp2);
  void GetPoseEstimates(std::vector<Pose3Stamped>& pose_estimates);
  void PerformIsamUpdate();
};

#endif  // ORB_TEST_SRC_IMU_GROUND_TRUTH_SMOOTHER_H_
