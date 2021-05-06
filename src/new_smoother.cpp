#include "new_smoother.h"
#include "global_params.h"
#include "ground_truth.h"
#include "gtsam_conversions.h"

#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/slam/SmartFactorParams.h>
#include <gtsam/base/make_shared.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2Result.h>

gtsam::ISAM2Params MakeISAM2Params()
{
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = GlobalParams::IsamRelinearizeThresh();
  params.relinearizeSkip = 1;
  params.evaluateNonlinearError = true;
  if (GlobalParams::UseDogLeg())
  {
    params.optimizationParams =
        gtsam::ISAM2DoglegParams(1.0, 1e-05, gtsam::DoglegOptimizerImpl::ONE_STEP_PER_ITERATION, false);
  }
  else
  {
    params.optimizationParams = gtsam::ISAM2GaussNewtonParams();
  }
  return params;
}

boost::shared_ptr<gtsam::PreintegrationCombinedParams> MakeIMUParams()
{
  auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(GlobalParams::IMUG());
  imu_params->accelerometerCovariance = gtsam::I_3x3 * std::pow(GlobalParams::IMUAccelNoiseDensity(), 2.0);
  imu_params->gyroscopeCovariance = gtsam::I_3x3 * std::pow(GlobalParams::IMUGyroNoiseDensity(), 2.0);
  imu_params->biasAccCovariance = gtsam::I_3x3 * std::pow(GlobalParams::IMUAccelRandomWalk(), 2.0);
  imu_params->biasOmegaCovariance = gtsam::I_3x3 * std::pow(GlobalParams::IMUGyroRandomWalk(), 2.0);
  imu_params->integrationCovariance = gtsam::I_3x3 * 1e-8;

  imu_params->body_P_sensor = gtsam::Pose3(
      gtsam::Rot3::Quaternion(GlobalParams::BodyPImuQuat()[3], GlobalParams::BodyPImuQuat()[0],
                              GlobalParams::BodyPImuQuat()[1], GlobalParams::BodyPImuQuat()[2]),
      gtsam::Point3(GlobalParams::BodyPImuVec()[0], GlobalParams::BodyPImuVec()[1], GlobalParams::BodyPImuVec()[2]));

  return imu_params;
}

NewSmoother::NewSmoother(std::shared_ptr<IMUQueue> imu_queue)
  : K_(gtsam::make_shared<gtsam::Cal3_S2>(GlobalParams::CamFx(), GlobalParams::CamFy(), 0.0, GlobalParams::CamU0(),
                                          GlobalParams::CamV0()))
  , feature_noise_(gtsam::noiseModel::Isotropic::Sigma(2, GlobalParams::NoiseFeature()))
  , range_noise_(gtsam::noiseModel::Isotropic::Sigma(1, GlobalParams::NoiseRange()))
  , graph_manager_(GraphManager(
        MakeISAM2Params(), gtsam::SmartProjectionParams(gtsam::HESSIAN, gtsam::IGNORE_DEGENERACY, false, true, 1e-5)))
  , imu_integrator_(std::move(imu_queue), MakeIMUParams(), gtsam::imuBias::ConstantBias())
{
}
void NewSmoother::Initialize(const shared_ptr<Frame>& frame,
                             const boost::optional<pair<double, double>>& imu_gravity_alignment_timestamps)
{
  auto unrefined_init_pose =
      GlobalParams::InitOnGroundTruth() ? ToGtsamPose(GroundTruth::At(frame->timestamp)) : gtsam::Pose3();
  auto refined_init_pose = gtsam::Pose3(
      imu_gravity_alignment_timestamps ? imu_integrator_.RefineInitialAttitude(imu_gravity_alignment_timestamps->first,
                                                                               imu_gravity_alignment_timestamps->second,
                                                                               unrefined_init_pose.rotation()) :
                                         unrefined_init_pose.rotation(),
      unrefined_init_pose.translation());

  auto noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3(GlobalParams::PriorNoiseXRollPitch(), GlobalParams::PriorNoiseXRollPitch(),
                                          GlobalParams::PriorNoiseXYaw()),
       gtsam::Vector3::Constant(GlobalParams::PriorNoiseXTranslation()))
          .finished());
  auto noise_v = gtsam::noiseModel::Isotropic::Sigma(3, frame->stationary ? 0.001 : GlobalParams::PriorNoiseVelocity());
  auto noise_b = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(GlobalParams::PriorNoiseAccel()),
       gtsam::Vector3::Constant(GlobalParams::PriorNoiseGyro()))
          .finished());

  auto init_velocity = gtsam::Vector3::Zero();
  auto init_bias = gtsam::imuBias::ConstantBias();

  gtsam::NavState init_nav_state(refined_init_pose, init_velocity);

  graph_manager_.SetInitNavstate(frame->id, init_nav_state, init_bias, noise_x, noise_v, noise_b);
  graph_manager_.Update();
  added_frames_[frame->id] = frame;
  last_frame_id_ = frame->id;
  initialized_ = true;
}

void NewSmoother::GetPoses(map<int, Pose3Stamped>& poses)
{
  for (const auto& frame : added_frames_)
  {
    poses[frame.first] = Pose3Stamped{ ToPose(graph_manager_.GetPose(frame.first)), frame.second->timestamp };
  }
}

void NewSmoother::GetLandmarks(map<int, Point3>& landmarks)
{
  for (const auto & landmark : graph_manager_.GetLandmarks())
  {
    if (landmark.second)
    {
      landmarks[landmark.first] = ToPoint(*(landmark.second));
    }
  }
}

bool NewSmoother::IsInitialized()
{
  return initialized_;
}
