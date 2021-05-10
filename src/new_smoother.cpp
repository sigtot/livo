#include "new_smoother.h"
#include "global_params.h"
#include "ground_truth.h"
#include "gtsam_conversions.h"
#include "depth_triangulation.h"
#include "gtsam_helpers.h"

#include <algorithm>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/ISAM2UpdateParams.h>
#include <gtsam/slam/SmartFactorParams.h>
#include <gtsam/base/make_shared.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2Result.h>

gtsam::ISAM2Params MakeISAM2Params()
{
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = GlobalParams::IsamRelinearizeThresh();
  params.relinearizeSkip = 10;
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

gtsam::SmartProjectionParams MakeSmartFactorParams()
{
  auto smart_factor_params = gtsam::SmartProjectionParams(gtsam::HESSIAN, gtsam::ZERO_ON_DEGENERACY, false, true, 1e-3);
  smart_factor_params.setDynamicOutlierRejectionThreshold(GlobalParams::DynamicOutlierRejectionThreshold());
  smart_factor_params.setLandmarkDistanceThreshold(GlobalParams::LandmarkDistanceThreshold());
  smart_factor_params.setRankTolerance(1);
  return smart_factor_params;
}

NewSmoother::NewSmoother(std::shared_ptr<IMUQueue> imu_queue)
  : K_(gtsam::make_shared<gtsam::Cal3_S2>(GlobalParams::CamFx(), GlobalParams::CamFy(), 0.0, GlobalParams::CamU0(),
                                          GlobalParams::CamV0()))
  , feature_noise_(gtsam::noiseModel::Isotropic::Sigma(2, GlobalParams::NoiseFeature()))
  , feature_m_estimator_(gtsam::noiseModel::mEstimator::Huber::Create(GlobalParams::RobustFeatureK()))
  , range_noise_(
        gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(GlobalParams::RobustRangeK()),
                                          gtsam::noiseModel::Isotropic::Sigma(1, GlobalParams::NoiseRange())))
  , graph_manager_(GraphManager(MakeISAM2Params(), MakeSmartFactorParams()))
  , imu_integrator_(std::move(imu_queue), MakeIMUParams(), gtsam::imuBias::ConstantBias())
  , body_p_cam_(gtsam::make_shared<gtsam::Pose3>(
        gtsam::Rot3::Quaternion(GlobalParams::BodyPCamQuat()[3], GlobalParams::BodyPCamQuat()[0],
                                GlobalParams::BodyPCamQuat()[1], GlobalParams::BodyPCamQuat()[2]),
        gtsam::Point3(GlobalParams::BodyPCamVec()[0], GlobalParams::BodyPCamVec()[1], GlobalParams::BodyPCamVec()[2])))
{
}

void NewSmoother::InitializeLandmarkWithDepth(int lmk_id, int frame_id, const gtsam::Point2& pt, double depth,
                                              const gtsam::Pose3& init_pose)
{
  std::cout << "Initializing lmk " << lmk_id << " with depth " << std::endl;
  gtsam::PinholeCamera<gtsam::Cal3_S2> camera(init_pose * *body_p_cam_, *K_);
  auto initial_landmark_estimate = DepthTriangulation::PixelAndDepthToPoint3(pt, depth, camera);
  graph_manager_.InitProjectionLandmark(lmk_id, frame_id, pt, initial_landmark_estimate, K_, *body_p_cam_,
                                        feature_noise_, feature_m_estimator_);
  graph_manager_.AddRangeObservation(lmk_id, frame_id, depth, range_noise_);
}

void NewSmoother::InitializeStructurelessLandmark(int lmk_id, int frame_id, const gtsam::Point2& pt)
{
  std::cout << "Initializing lmk " << lmk_id << " without depth" << std::endl;
  graph_manager_.InitStructurelessLandmark(lmk_id, frame_id, pt, K_, *body_p_cam_, feature_noise_,
                                           feature_m_estimator_);
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

  for (auto& feature_pair : SortFeatures(frame->features))
  {
    auto lmk_id = feature_pair.first;
    auto feature = feature_pair.second.lock();
    if (!feature)
    {
      continue;
    }
    gtsam::Point2 gtsam_pt(feature->pt.x, feature->pt.y);
    if (feature->depth)
    {
      InitializeLandmarkWithDepth(lmk_id, frame->id, gtsam_pt, *feature->depth, refined_init_pose);
    }
    else
    {
      InitializeStructurelessLandmark(lmk_id, frame->id, gtsam_pt);
    }
  }

  graph_manager_.Update();
  added_frames_[frame->id] = frame;
  last_frame_id_ = frame->id;
  initialized_ = true;
}

void NewSmoother::AddFrame(const std::shared_ptr<Frame>& frame)
{
  imu_integrator_.WaitAndIntegrate(added_frames_[last_frame_id_]->timestamp, frame->timestamp);
  auto prev_nav_state = graph_manager_.GetNavState(last_frame_id_);
  auto prev_bias = graph_manager_.GetBias(last_frame_id_);
  auto predicted_nav_state = imu_integrator_.PredictNavState(prev_nav_state, prev_bias);
  auto pim = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imu_integrator_.GetPim());
  graph_manager_.AddFrame(frame->id, pim, predicted_nav_state, prev_bias);
  imu_integrator_.ResetIntegration();

  for (auto& feature_pair : frame->features)
  {
    auto lmk_id = feature_pair.first;
    auto feature = feature_pair.second.lock();
    if (!feature || !graph_manager_.IsLandmarkTracked(lmk_id))
    {
      continue;
    }
    gtsam::Point2 gtsam_pt(feature->pt.x, feature->pt.y);
    if (feature->depth)
    {
      graph_manager_.AddRangeObservation(lmk_id, frame->id, *feature->depth, range_noise_);
    }
    graph_manager_.AddLandmarkObservation(lmk_id, frame->id, gtsam_pt, K_, *body_p_cam_);
  }
  graph_manager_.Update();

  added_frames_[frame->id] = frame;
  last_frame_id_ = frame->id;
}

void NewSmoother::AddKeyframe(const std::shared_ptr<Frame>& frame)
{
  imu_integrator_.WaitAndIntegrate(added_frames_[last_frame_id_]->timestamp, frame->timestamp);
  auto prev_nav_state = graph_manager_.GetNavState(last_frame_id_);
  auto prev_bias = graph_manager_.GetBias(last_frame_id_);
  auto predicted_nav_state = imu_integrator_.PredictNavState(prev_nav_state, prev_bias);
  auto pim = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imu_integrator_.GetPim());
  graph_manager_.AddFrame(frame->id, pim, predicted_nav_state, prev_bias);
  imu_integrator_.ResetIntegration();

  std::map<int, std::weak_ptr<Feature>> existing_features;
  std::map<int, std::weak_ptr<Feature>> new_track_features;
  for (const auto& feature_pair : frame->features)
  {
    if (graph_manager_.IsLandmarkTracked(feature_pair.first))
    {
      existing_features[feature_pair.first] = feature_pair.second;
    }
    else
    {
      new_track_features[feature_pair.first] = feature_pair.second;
    }
  }

  auto sorted_new_track_features = SortFeatures(new_track_features);

  std::vector<std::pair<int, std::weak_ptr<Track>>> new_tracks;
  for (const auto& feature_pair : new_track_features)
  {
    auto feature = feature_pair.second.lock();
    if (feature)
    {
      new_tracks.emplace_back(feature_pair.first, feature->track);
    }
  }

  for (const auto& track_pair : new_tracks)
  {
    auto track = track_pair.second.lock();
    if (!track)
    {
      continue;
    }
    if (track->HasDepth())
    {
      // If we have depth, find the first feature with depth, and use it to initialize the landmark
      int frame_id_used_for_init = -1;
      for (const auto& feature : track->features)
      {
        if (feature->depth && graph_manager_.CanAddObservationsForFrame(feature->frame->id))
        {
          auto pose_for_init = (feature->frame->id == frame->id) ? predicted_nav_state.pose() :
                                                                   graph_manager_.GetPose(feature->frame->id);
          InitializeLandmarkWithDepth(track->id, feature->frame->id, gtsam::Point2(feature->pt.x, feature->pt.y),
                                      *feature->depth, pose_for_init);
          frame_id_used_for_init = feature->frame->id;
          break;
        }
      }
      assert(frame_id_used_for_init != -1);

      for (auto& feature : track->features)
      {
        if (feature->frame->id == frame_id_used_for_init ||
            !graph_manager_.CanAddObservationsForFrame(feature->frame->id))
        {
          continue;
        }
        if (feature->depth)
        {
          graph_manager_.AddRangeObservation(track->id, feature->frame->id, *feature->depth, range_noise_);
        }
        graph_manager_.AddLandmarkObservation(track->id, feature->frame->id,
                                              gtsam::Point2(feature->pt.x, feature->pt.y), K_, *body_p_cam_);
      }
    }
    else
    {
      auto lmk_initialized = false;
      for (const auto& feature : track->features)
      {
        if (!graph_manager_.CanAddObservationsForFrame(feature->frame->id))
        {
          continue;
        }
        if (lmk_initialized)
        {
          graph_manager_.AddLandmarkObservation(track->id, feature->frame->id,
                                                gtsam::Point2(feature->pt.x, feature->pt.y), K_, *body_p_cam_);
        }
        else
        {
          InitializeStructurelessLandmark(track->id, feature->frame->id, gtsam::Point2(feature->pt.x, feature->pt.y));
          lmk_initialized = true;
        }
      }
    }
  }

  for (auto& feature_pair : existing_features)
  {
    auto lmk_id = feature_pair.first;
    auto feature = feature_pair.second.lock();
    if (!feature || !graph_manager_.IsLandmarkTracked(lmk_id))
    {
      continue;
    }
    gtsam::Point2 gtsam_pt(feature->pt.x, feature->pt.y);
    if (feature->depth)
    {
      graph_manager_.AddRangeObservation(lmk_id, frame->id, *feature->depth, range_noise_);
    }
    graph_manager_.AddLandmarkObservation(lmk_id, frame->id, gtsam_pt, K_, *body_p_cam_);
  }

  gtsam::ISAM2UpdateParams update_params;
  update_params.force_relinearize = true;

  graph_manager_.Update(update_params);

  added_frames_[frame->id] = frame;
  last_frame_id_ = frame->id;
}

void NewSmoother::GetPoses(map<int, Pose3Stamped>& poses) const
{
  for (const auto& frame : added_frames_)
  {
    poses[frame.first] = Pose3Stamped{ ToPose(graph_manager_.GetPose(frame.first)), frame.second->timestamp };
  }
}

void NewSmoother::GetLandmarks(map<int, Point3>& landmarks) const
{
  for (const auto& landmark : graph_manager_.GetLandmarks())
  {
    if (landmark.second)
    {
      landmarks[landmark.first] = ToPoint(*(landmark.second));
    }
  }
}

bool NewSmoother::IsInitialized() const
{
  return initialized_;
}
