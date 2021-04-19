#include "imu_ground_truth_smoother.h"
#include "global_params.h"
#include "ground_truth.h"
#include "pose3_stamped.h"
#include "point3.h"
#include "gtsam_conversions.h"

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <thread>
#include <debug_value_publisher.h>

void WaitForAndIntegrateIMU(double timestamp1, double timestamp2);
void GetPoseEstimates(vector<Pose3Stamped>& pose_estimates);
void PerformIsamUpdate();
using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

gtsam::ISAM2Params MakeIsam2ParamsForIMUGroundTruthSmoother()
{
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = GlobalParams::IsamRelinearizeThresh();
  params.relinearizeSkip = 1;
  params.evaluateNonlinearError = true;
  params.optimizationParams = gtsam::ISAM2GaussNewtonParams();
  return params;
}

std::shared_ptr<gtsam::PreintegrationType> IMUGroundTruthSmoother::MakeIMUIntegrator()
{
  auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(GlobalParams::IMUG());
  imu_params->accelerometerCovariance = gtsam::I_3x3 * std::pow(GlobalParams::IMUAccelNoiseDensity(), 2.0);
  imu_params->gyroscopeCovariance = gtsam::I_3x3 * std::pow(GlobalParams::IMUGyroNoiseDensity(), 2.0);
  imu_params->biasAccCovariance = gtsam::I_3x3 * std::pow(GlobalParams::IMUAccelRandomWalk(), 2.0);
  imu_params->biasOmegaCovariance = gtsam::I_3x3 * std::pow(GlobalParams::IMUGyroRandomWalk(), 2.0);
  imu_params->integrationCovariance = gtsam::I_3x3 * 1e-8;

  imu_params->body_P_sensor = *body_p_imu_;

  auto imu_bias = gtsam::imuBias::ConstantBias();  // Initialize at zero bias

  auto imu_measurements = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imu_params, imu_bias);
  imu_measurements->resetIntegration();
  return imu_measurements;
}

IMUGroundTruthSmoother::IMUGroundTruthSmoother(const std::shared_ptr<IMUQueue>& imu_queue)
  : isam2_(std::make_shared<gtsam::ISAM2>(MakeIsam2ParamsForIMUGroundTruthSmoother()))
  , graph_(std::make_shared<gtsam::NonlinearFactorGraph>())
  , values_(std::make_shared<gtsam::Values>())
  , imu_queue_(imu_queue)
  , imu_measurements_(MakeIMUIntegrator())
  , body_p_imu_(gtsam::make_shared<gtsam::Pose3>(
        gtsam::Rot3::Quaternion(GlobalParams::BodyPImuQuat()[3], GlobalParams::BodyPImuQuat()[0],
                                GlobalParams::BodyPImuQuat()[1], GlobalParams::BodyPImuQuat()[2]),
        gtsam::Point3(GlobalParams::BodyPImuVec()[0], GlobalParams::BodyPImuVec()[1], GlobalParams::BodyPImuVec()[2])))
{
}

void IMUGroundTruthSmoother::Initialize(double stamp1, double stamp2, std::vector<Pose3Stamped>& pose_estimates)
{
  std::cout << "Initializing between timestamps " << stamp1 << " and " << stamp2 << std::endl;

  auto prior_noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3(GlobalParams::PriorNoiseXRollPitch(), GlobalParams::PriorNoiseXRollPitch(),
                                          GlobalParams::PriorNoiseXYaw()),
       gtsam::Vector3::Constant(GlobalParams::PriorNoiseXTranslation()))
          .finished());
  auto prior_noise_v = gtsam::noiseModel::Isotropic::Sigma(3, GlobalParams::PriorNoiseVelocity());
  auto prior_noise_b = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(GlobalParams::PriorNoiseAccel()),
       gtsam::Vector3::Constant(GlobalParams::PriorNoiseGyro()))
          .finished());

  auto init_pose = ToGtsamPose(GroundTruth::At(stamp1));
  auto init_velocity = gtsam::Vector3(0.000001, 0.000002, 0.000001);
  gtsam::imuBias::ConstantBias init_bias(gtsam::Vector3(-0.025266, 0.136696, 0.075593),
                                         gtsam::Vector3(-0.003172, 0.021267, 0.078502));

  graph_->addPrior(X(1), init_pose, prior_noise_x);
  graph_->addPrior(V(1), init_velocity, prior_noise_v);
  graph_->addPrior(B(1), init_bias, prior_noise_b);

  values_->insert(X(1), init_pose);
  values_->insert(V(1), init_velocity);
  values_->insert(B(1), init_bias);

  auto second_pose = ToGtsamPose(GroundTruth::At(stamp2));
  values_->insert(X(2), second_pose);
  values_->insert(V(2), init_velocity);  // Assume v1 == v2
  values_->insert(B(2), init_bias);

  imu_measurements_->resetIntegrationAndSetBias(init_bias);

  WaitForAndIntegrateIMU(stamp1, stamp2);

  auto imu_combined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imu_measurements_);
  gtsam::CombinedImuFactor imu_factor(X(1), V(1), X(2), V(2), B(1), B(2), imu_combined);
  graph_->add(imu_factor);

  PerformIsamUpdate();

  // Make us ready for the next iteration
  auto new_bias = isam2_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(2));
  imu_measurements_->resetIntegrationAndSetBias(new_bias);
  added_frame_timestamps_[1] = stamp1;
  added_frame_timestamps_[2] = stamp2;
  last_frame_id_added_ = 2;
  graph_->resize(0);
  values_->clear();
  initialized_ = true;

  // Fill output vector
  GetPoseEstimates(pose_estimates);

  // Publish debug stuff
  std::vector<double> bias_acc = { new_bias.accelerometer().x(), new_bias.accelerometer().y(),
                                   new_bias.accelerometer().z() };
  std::vector<double> bias_gyro = { new_bias.gyroscope().x(), new_bias.gyroscope().y(), new_bias.gyroscope().z() };
  DebugValuePublisher::PublishBias(bias_acc, bias_gyro);
}

void IMUGroundTruthSmoother::IMUPredict(double stamp, std::vector<Pose3Stamped>& pose_estimates)
{
  std::cout << "Performing imu prediction for stamp " << stamp << std::endl;
  auto prev_pose = isam2_->calculateEstimate<gtsam::Pose3>(X(last_frame_id_added_));
  auto prev_velocity = isam2_->calculateEstimate<gtsam::Vector3>(V(last_frame_id_added_));
  auto prev_bias = isam2_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(last_frame_id_added_));

  WaitForAndIntegrateIMU(added_frame_timestamps_[last_frame_id_added_], stamp);

  auto predicted_navstate = imu_measurements_->predict(gtsam::NavState(prev_pose, prev_velocity), prev_bias);

  auto new_id = last_frame_id_added_ + 1;

  values_->insert(X(new_id), predicted_navstate.pose());
  values_->insert(V(new_id), predicted_navstate.velocity());
  values_->insert(B(new_id), prev_bias);

  auto imu_combined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imu_measurements_);
  gtsam::CombinedImuFactor imu_factor(X(last_frame_id_added_), V(last_frame_id_added_), X(new_id), V(new_id),
                                      B(last_frame_id_added_), B(new_id), imu_combined);
  graph_->add(imu_factor);

  PerformIsamUpdate();

  // Make us ready for the next iteration
  auto new_bias = isam2_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(new_id));
  imu_measurements_->resetIntegrationAndSetBias(isam2_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(new_id)));
  added_frame_timestamps_[new_id] = stamp;
  last_frame_id_added_ = new_id;
  graph_->resize(0);
  values_->clear();
  initialized_ = true;

  // Fill output vector
  GetPoseEstimates(pose_estimates);

  // Publish debug stuff
  std::vector<double> bias_acc = { new_bias.accelerometer().x(), new_bias.accelerometer().y(),
                                   new_bias.accelerometer().z() };
  std::vector<double> bias_gyro = { new_bias.gyroscope().x(), new_bias.gyroscope().y(), new_bias.gyroscope().z() };
  DebugValuePublisher::PublishBias(bias_acc, bias_gyro);
}
void IMUGroundTruthSmoother::GroundTruthUpdate(double stamp, std::vector<Pose3Stamped>& pose_estimates)
{
  std::cout << "Performing ground truth update for stamp " << stamp << std::endl;
  WaitForAndIntegrateIMU(added_frame_timestamps_[last_frame_id_added_], stamp);

  auto prev_pose = isam2_->calculateEstimate<gtsam::Pose3>(X(last_frame_id_added_));
  auto prev_velocity = isam2_->calculateEstimate<gtsam::Vector3>(V(last_frame_id_added_));
  auto prev_bias = isam2_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(last_frame_id_added_));

  auto predicted_navstate = imu_measurements_->predict(gtsam::NavState(prev_pose, prev_velocity), prev_bias);

  auto new_id = last_frame_id_added_ + 1;

  values_->insert(X(new_id), predicted_navstate.pose());
  values_->insert(V(new_id), predicted_navstate.velocity());
  values_->insert(B(new_id), prev_bias);

  auto imu_combined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imu_measurements_);
  gtsam::CombinedImuFactor imu_factor(X(last_frame_id_added_), V(last_frame_id_added_), X(new_id), V(new_id),
                                      B(last_frame_id_added_), B(new_id), imu_combined);
  graph_->add(imu_factor);

  auto noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(0.3), gtsam::Vector3::Constant(0.3)).finished());
  graph_->addPrior(X(new_id), ToGtsamPose(GroundTruth::At(stamp)));

  PerformIsamUpdate();

  // Make us ready for the next iteration
  auto new_bias = isam2_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(new_id));
  imu_measurements_->resetIntegrationAndSetBias(new_bias);
  added_frame_timestamps_[new_id] = stamp;
  last_frame_id_added_ = new_id;
  graph_->resize(0);
  values_->clear();
  initialized_ = true;

  // Fill output vector
  GetPoseEstimates(pose_estimates);

  // Publish debug stuff
  std::vector<double> bias_acc = { new_bias.accelerometer().x(), new_bias.accelerometer().y(),
                                   new_bias.accelerometer().z() };
  std::vector<double> bias_gyro = { new_bias.gyroscope().x(), new_bias.gyroscope().y(), new_bias.gyroscope().z() };
  DebugValuePublisher::PublishBias(bias_acc, bias_gyro);
}

void IMUGroundTruthSmoother::PerformIsamUpdate()
{
  auto isam_result = isam2_->update(*graph_, *values_);
  isam_result.print();
  if (isam_result.errorBefore && isam_result.errorAfter)
  {
    std::cout << "error before after: " << *isam_result.errorBefore << " -> " << *isam_result.errorAfter << std::endl;
  }
}

void IMUGroundTruthSmoother::WaitForAndIntegrateIMU(double timestamp1, double timestamp2)
{
  while (!imu_queue_->hasMeasurementsInRange(timestamp1, timestamp2))
  {
    std::cout << "No IMU measurements in time range " << std::setprecision(17) << timestamp1 << " -> " << timestamp2
              << std::endl;
    std::cout << "Waiting 1 ms" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  imu_queue_->integrateIMUMeasurements(imu_measurements_, timestamp1, timestamp2);
}

void IMUGroundTruthSmoother::GetPoseEstimates(std::vector<Pose3Stamped>& pose_estimates)
{
  for (auto& frame_pair : added_frame_timestamps_)
  {
    auto gtsam_pose = isam2_->calculateEstimate<gtsam::Pose3>(X(frame_pair.first));
    pose_estimates.push_back(Pose3Stamped{ .pose = ToPose(gtsam_pose), .stamp = frame_pair.second });
  }
}

bool IMUGroundTruthSmoother::IsInitialized()
{
  return initialized_;
}
