#include "new_smoother.h"
#include "global_params.h"
#include "ground_truth.h"
#include "gtsam_conversions.h"
#include "depth_triangulation.h"
#include "feature_helpers.h"
#include "landmark_result.h"
#include "landmark_result_gtsam.h"
#include "debug_value_publisher.h"
#include "isam2_solver.h"
#include "incremental_fixed_lag_solver.h"
#include "tf2_between_transform_provider.h"

#include <algorithm>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/slam/SmartFactorParams.h>
#include <gtsam/base/make_shared.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2Result.h>
#include <chrono>

gtsam::ISAM2Params MakeISAM2Params()
{
  gtsam::ISAM2Params params;
  gtsam::FastMap<char, gtsam::Vector> thresholds;
  thresholds['x'] =
      (gtsam::Vector(6) << GlobalParams::IsamRelinThreshXRotation(), GlobalParams::IsamRelinThreshXRotation(),
       GlobalParams::IsamRelinThreshXRotation(), GlobalParams::IsamRelinThreshXTranslation(),
       GlobalParams::IsamRelinThreshXTranslation(), GlobalParams::IsamRelinThreshXTranslation())
          .finished();
  thresholds['v'] = (gtsam::Vector(3) << GlobalParams::IsamRelinThreshV(), GlobalParams::IsamRelinThreshV(),
                     GlobalParams::IsamRelinThreshV())
                        .finished();
  thresholds['b'] = (gtsam::Vector(6) << GlobalParams::IsamRelinThreshB(), GlobalParams::IsamRelinThreshB(),
                     GlobalParams::IsamRelinThreshB(), GlobalParams::IsamRelinThreshB(),
                     GlobalParams::IsamRelinThreshB(), GlobalParams::IsamRelinThreshB())
                        .finished();
  thresholds['l'] = (gtsam::Vector(3) << GlobalParams::IsamRelinThreshL(), GlobalParams::IsamRelinThreshL(),
                     GlobalParams::IsamRelinThreshL())
                        .finished();
  params.relinearizeThreshold = thresholds;
  params.relinearizeSkip = GlobalParams::IsamRelinearizeSkip();
  params.enableDetailedResults = true;
  params.evaluateNonlinearError = true;
  params.findUnusedFactorSlots = true;
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

std::shared_ptr<IncrementalSolver> GetIncrementalSolver()
{
  if (GlobalParams::UseFixedLag())
  {
    return std::make_shared<IncrementalFixedLagSolver>(GlobalParams::SmootherLag(), MakeISAM2Params());
  }
  return std::make_shared<ISAM2Solver>(MakeISAM2Params());
}

NewSmoother::NewSmoother(std::shared_ptr<IMUQueue> imu_queue)
  : K_(gtsam::make_shared<gtsam::Cal3_S2>(GlobalParams::CamFx(), GlobalParams::CamFy(), 0.0, GlobalParams::CamU0(),
                                          GlobalParams::CamV0()))
  , feature_noise_(gtsam::noiseModel::Isotropic::Sigma(2, GlobalParams::NoiseFeature()))
  , feature_m_estimator_(gtsam::noiseModel::mEstimator::Huber::Create(GlobalParams::RobustFeatureK()))
  , range_noise_(
        gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(GlobalParams::RobustRangeK()),
                                          gtsam::noiseModel::Isotropic::Sigma(1, GlobalParams::NoiseRange())))
  , graph_manager_(GraphManager(GetIncrementalSolver(), MakeSmartFactorParams()))
  , imu_integrator_(std::move(imu_queue), MakeIMUParams(), gtsam::imuBias::ConstantBias())
  , between_transform_provider_(std::make_shared<TF2BetweenTransformProvider>(
        gtsam::Pose3(gtsam::Rot3::Quaternion(GlobalParams::BodyPLidarQuat()[3], GlobalParams::BodyPLidarQuat()[0],
                                             GlobalParams::BodyPLidarQuat()[1], GlobalParams::BodyPLidarQuat()[2]),
                     gtsam::Point3(GlobalParams::BodyPLidarVec()[0], GlobalParams::BodyPLidarVec()[1],
                                   GlobalParams::BodyPLidarVec()[2])),
        "test", "test"))
  , body_p_cam_(gtsam::make_shared<gtsam::Pose3>(
        gtsam::Rot3::Quaternion(GlobalParams::BodyPCamQuat()[3], GlobalParams::BodyPCamQuat()[0],
                                GlobalParams::BodyPCamQuat()[1], GlobalParams::BodyPCamQuat()[2]),
        gtsam::Point3(GlobalParams::BodyPCamVec()[0], GlobalParams::BodyPCamVec()[1], GlobalParams::BodyPCamVec()[2])))
{
}

gtsam::Point3 NewSmoother::CalculatePointEstimate(const gtsam::Pose3& pose, const gtsam::Point2& pt, double depth) const
{
  gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pose * *body_p_cam_, *K_);
  return DepthTriangulation::PixelAndDepthToPoint3(pt, depth, camera);
}

void NewSmoother::InitializeProjLandmarkWithDepth(int lmk_id, int frame_id, double timestamp, const gtsam::Point2& pt,
                                                  double depth, const gtsam::Pose3& init_pose)
{
  std::cout << "Initializing lmk " << lmk_id << " with depth " << std::endl;
  graph_manager_.InitProjectionLandmark(lmk_id, frame_id, timestamp, pt, CalculatePointEstimate(init_pose, pt, depth),
                                        K_, *body_p_cam_, feature_noise_, feature_m_estimator_);
  graph_manager_.AddRangeObservation(lmk_id, frame_id, timestamp, depth, range_noise_);
}

bool NewSmoother::TryInitializeProjLandmarkByTriangulation(int lmk_id, int frame_id, double timestamp,
                                                           const std::shared_ptr<Track>& track)
{
  std::cout << "Trying to initialize lmk " << lmk_id << " by triangulation" << std::endl;
  std::vector<gtsam::PinholeCamera<gtsam::Cal3_S2>> cameras;
  std::vector<gtsam::Point2> measurements;
  gtsam::Point2 pt_for_first_factor;
  for (const auto& feature : track->features)
  {
    if (graph_manager_.IsFrameTracked(feature->frame->id))
    {
      cameras.emplace_back(graph_manager_.GetPose(feature->frame->id) * *body_p_cam_, *K_);
      measurements.emplace_back(feature->pt.x, feature->pt.y);
    }
    if (feature->frame->id == frame_id)
    {
      pt_for_first_factor = gtsam::Point2(feature->pt.x, feature->pt.y);
    }
  }

  if (measurements.size() < 2)
  {
    return false;
  }

  auto triangulation_result =
      DepthTriangulation::Triangulate(measurements, cameras, MakeSmartFactorParams().getTriangulationParameters());

  if (!triangulation_result)
  {
    return false;
  }

  auto range = graph_manager_.GetPose(last_frame_id_).range(*triangulation_result);
  if (range > GlobalParams::ProjLandmarkInitDistanceThresh())
  {
    std::cout << "Proj lmk at range " << range << " rejected" << std::endl;
    return false;
  }

  graph_manager_.InitProjectionLandmark(lmk_id, frame_id, timestamp, pt_for_first_factor, *triangulation_result, K_,
                                        *body_p_cam_, feature_noise_, feature_m_estimator_);
  return true;
}

void NewSmoother::InitializeStructurelessLandmark(int lmk_id, int frame_id, double timestamp, const gtsam::Point2& pt)
{
  std::cout << "Initializing lmk " << lmk_id << " without depth" << std::endl;
  graph_manager_.InitStructurelessLandmark(lmk_id, frame_id, timestamp, pt, K_, *body_p_cam_, feature_noise_,
                                           feature_m_estimator_);
}

void NewSmoother::Initialize(const std::shared_ptr<Frame>& frame,
                             const boost::optional<std::pair<double, double>>& imu_gravity_alignment_timestamps)
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

  graph_manager_.SetInitNavstate(frame->id, frame->timestamp, init_nav_state, init_bias, noise_x, noise_v, noise_b);

  for (auto& feature_pair : frame->features)
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
      InitializeProjLandmarkWithDepth(lmk_id, frame->id, frame->timestamp, gtsam_pt, *feature->depth,
                                      refined_init_pose);
    }
    else
    {
      if (GlobalParams::EnableSmartFactors())
      {
        InitializeStructurelessLandmark(lmk_id, frame->id, frame->timestamp, gtsam_pt);
      }
    }
  }

  graph_manager_.Update();
  added_frames_[frame->id] = frame;
  last_frame_id_ = frame->id;
  last_keyframe_id_ = frame->id;
  initialized_ = true;
}

void NewSmoother::AddFrame(const std::shared_ptr<Frame>& frame)
{
  imu_integrator_.WaitAndIntegrate(added_frames_[last_frame_id_]->timestamp, frame->timestamp);
  auto prev_nav_state = graph_manager_.GetNavState(last_frame_id_);
  auto prev_bias = graph_manager_.GetBias(last_frame_id_);
  auto predicted_nav_state = imu_integrator_.PredictNavState(prev_nav_state, prev_bias);
  auto pim = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imu_integrator_.GetPim());
  graph_manager_.AddFrame(frame->id, added_frames_[last_keyframe_id_]->timestamp, pim, predicted_nav_state, prev_bias);
  imu_integrator_.ResetIntegration();

  auto feature_obs_count = 0;
  auto range_obs_count = 0;
  for (auto& feature_pair : frame->features)
  {
    auto lmk_id = feature_pair.first;
    auto feature = feature_pair.second.lock();
    if (!feature || !graph_manager_.CanAddObservation(lmk_id, frame->id))
    {
      continue;
    }
    gtsam::Point2 gtsam_pt(feature->pt.x, feature->pt.y);
    if (feature->depth)
    {
      if (graph_manager_.IsSmartFactorLandmark(lmk_id))
      {
        auto track = feature->track.lock();
        assert(track);
        auto init_point = CalculatePointEstimate(predicted_nav_state.pose(), gtsam_pt, *feature->depth);
        graph_manager_.ConvertSmartFactorToProjectionFactor(lmk_id, added_frames_[last_keyframe_id_]->timestamp,
                                                            init_point);
      }
      graph_manager_.AddRangeObservation(lmk_id, frame->id, added_frames_[last_keyframe_id_]->timestamp,
                                         *feature->depth, range_noise_);
      range_obs_count++;
    }
    graph_manager_.AddLandmarkObservation(lmk_id, frame->id, added_frames_[last_keyframe_id_]->timestamp, gtsam_pt, K_,
                                          *body_p_cam_);
    feature_obs_count++;
  }

  std::cout << "Added " << feature_obs_count << " observations (" << range_obs_count << " range obs.)" << std::endl;
  auto time_before = std::chrono::system_clock::now();
  auto isam_result = graph_manager_.Update();
  auto time_after = std::chrono::system_clock::now();
  DebugValuePublisher::PublishRelinearizedCliques(static_cast<int>(isam_result.variablesRelinearized));
  DebugValuePublisher::PublishReeliminatedCliques(static_cast<int>(isam_result.variablesReeliminated));
  DebugValuePublisher::PublishTotalCliques(static_cast<int>(isam_result.cliques));
  auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(time_after - time_before);
  DebugValuePublisher::PublishUpdateDuration(static_cast<int>(millis.count()));
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
  graph_manager_.AddFrame(frame->id, frame->timestamp, pim, predicted_nav_state, prev_bias);
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

  std::vector<std::pair<int, std::weak_ptr<Track>>> new_tracks;
  for (const auto& feature_pair : new_track_features)
  {
    auto feature = feature_pair.second.lock();
    if (feature)
    {
      new_tracks.emplace_back(feature_pair.first, feature->track);
    }
  }

  auto added_landmarks_count = 0;
  for (const auto& track_pair : new_tracks)
  {
    auto obs_count = 0;
    auto track = track_pair.second.lock();
    if (!track)
    {
      continue;
    }
    if (track->HasDepth())
    {
      // If we have depth, find the first feature with depth, and use it to initialize the landmark
      // First, we need to get frame we first observed the features from, because this will be used
      boost::optional<std::shared_ptr<Frame>> frame_first_seen;
      boost::optional<std::shared_ptr<Feature>> first_feature;
      for (const auto& feature : track->features)
      {
        if (graph_manager_.CanAddObservationsForFrame(feature->frame->id, feature->frame->timestamp))
        {
          frame_first_seen = feature->frame;
          first_feature = feature;
          break;
        }
      }
      if (!frame_first_seen)
      {
        continue;
      }
      int frame_id_used_for_init = -1;
      for (const auto& feature : track->features)
      {
        if (feature->depth && graph_manager_.CanAddObservationsForFrame(feature->frame->id, feature->frame->timestamp))
        {
          std::cout << "Initializing lmk " << track->id << " with depth " << std::endl;
          auto pose_for_init = (feature->frame->id == frame->id) ? predicted_nav_state.pose() :
                                                                   graph_manager_.GetPose(feature->frame->id);
          gtsam::Point2 pt_for_init(feature->pt.x, feature->pt.y);
          // The initial point3 estimate can be obtained from any frame, so we used the first available with depth
          auto init_point_estimate = CalculatePointEstimate(pose_for_init, pt_for_init, *feature->depth);

          // We do however need to use the first seen feature in the call to InitProjectionLandmark,
          // as we can only add observations that come after this frame.
          auto first_feature_pt = gtsam::Point2((*first_feature)->pt.x, (*first_feature)->pt.y);
          graph_manager_.InitProjectionLandmark(track->id, (*frame_first_seen)->id, (*frame_first_seen)->timestamp,
                                                first_feature_pt, init_point_estimate, K_, *body_p_cam_, feature_noise_,
                                                feature_m_estimator_);
          graph_manager_.AddRangeObservation(track->id, feature->frame->id, frame->timestamp, *feature->depth,
                                             range_noise_);
          frame_id_used_for_init = (*frame_first_seen)->id;
          obs_count++;
          added_landmarks_count++;
          break;
        }
      }
      assert(frame_id_used_for_init != -1);

      // After initialization, add observations for all other features in the track
      for (auto& feature : track->features)
      {
        if (feature->frame->id == frame_id_used_for_init ||
            !graph_manager_.CanAddObservation(track->id, feature->frame->id))
        {
          continue;
        }
        gtsam::Point2 gtsam_pt = gtsam::Point2(feature->pt.x, feature->pt.y);
        if (feature->depth)
        {
          if (!graph_manager_.CanAddRangeObservation(track->id, feature->frame->id))
          {
            auto pose_for_init = (feature->frame->id == frame->id) ? predicted_nav_state.pose() :
                                                                     graph_manager_.GetPose(feature->frame->id);
            auto init_point = CalculatePointEstimate(pose_for_init, gtsam_pt, *feature->depth);
            graph_manager_.ConvertSmartFactorToProjectionFactor(track->id, frame->timestamp, init_point);
          }
          graph_manager_.AddRangeObservation(track->id, feature->frame->id, frame->timestamp, *feature->depth,
                                             range_noise_);
        }
        graph_manager_.AddLandmarkObservation(track->id, feature->frame->id, frame->timestamp, gtsam_pt, K_,
                                              *body_p_cam_);
        obs_count++;
      }
      std::cout << "Added proj with " << obs_count << " observations" << std::endl;
    }
    else if (track->features.size() >= GlobalParams::MinTrackLengthForSmoothing())
    {
      auto lmk_initialized = false;
      for (const auto& feature : track->features)
      {
        if (!graph_manager_.CanAddObservationsForFrame(feature->frame->id, feature->frame->timestamp))
        {
          std::cout << "Cannot add observations for frame " << feature->frame->id << std::endl;
          continue;
        }
        if (lmk_initialized)
        {
          if (graph_manager_.CanAddObservation(track->id, feature->frame->id))
          {
            graph_manager_.AddLandmarkObservation(track->id, feature->frame->id, frame->timestamp,
                                                  gtsam::Point2(feature->pt.x, feature->pt.y), K_, *body_p_cam_);
            obs_count++;
          }
        }
        else
        {
          if (GlobalParams::EnableSmartFactors())
          {
            InitializeStructurelessLandmark(track->id, feature->frame->id, feature->frame->timestamp,
                                            gtsam::Point2(feature->pt.x, feature->pt.y));
          }
          else
          {
            if (track->max_parallax < GlobalParams::MinParallaxForSmoothing())
            {
              track->rejected = true;
              goto for_tracks;
            }
            auto success = TryInitializeProjLandmarkByTriangulation(track->id, feature->frame->id,
                                                                    feature->frame->timestamp, track);
            if (!success)
            {
              track->rejected = true;
              goto for_tracks;  // Continue to the next track. We'll try to initialize the lmk again next KF.
            }
          }
          added_landmarks_count++;
          obs_count++;
          lmk_initialized = true;
        }
      }
      if (lmk_initialized)
      {
        std::cout << "Added landmark with " << obs_count << " observations" << std::endl;
      }
      else
      {
        std::cout << "Failed to initialize for some reason" << std::endl;
        track->rejected = true;
      }
    }
  for_tracks:;
  }
  std::cout << "Initialized " << added_landmarks_count << " landmarks" << std::endl;

  auto feature_obs_count = 0;
  auto range_obs_count = 0;
  for (auto& feature_pair : existing_features)
  {
    auto lmk_id = feature_pair.first;
    auto feature = feature_pair.second.lock();
    if (!feature || !graph_manager_.CanAddObservation(lmk_id, feature->frame->id))
    {
      continue;
    }
    gtsam::Point2 gtsam_pt(feature->pt.x, feature->pt.y);
    if (feature->depth)
    {
      if (graph_manager_.IsSmartFactorLandmark(lmk_id))
      {
        auto track = feature->track.lock();
        assert(track);
        auto init_point = CalculatePointEstimate(predicted_nav_state.pose(), gtsam_pt, *feature->depth);
        graph_manager_.ConvertSmartFactorToProjectionFactor(lmk_id, frame->timestamp, init_point);
      }
      graph_manager_.AddRangeObservation(lmk_id, frame->id, frame->timestamp, *feature->depth, range_noise_);
      range_obs_count++;
    }
    feature_obs_count++;
    graph_manager_.AddLandmarkObservation(lmk_id, frame->id, frame->timestamp, gtsam_pt, K_, *body_p_cam_);
  }

  std::cout << "Added " << feature_obs_count << " observations (" << range_obs_count << " range obs.)" << std::endl;

  auto time_before = std::chrono::system_clock::now();
  auto isam_result = graph_manager_.Update();
  auto time_after = std::chrono::system_clock::now();
  DebugValuePublisher::PublishRelinearizedCliques(static_cast<int>(isam_result.variablesRelinearized));
  DebugValuePublisher::PublishReeliminatedCliques(static_cast<int>(isam_result.variablesReeliminated));
  DebugValuePublisher::PublishTotalCliques(static_cast<int>(isam_result.cliques));
  auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(time_after - time_before);
  DebugValuePublisher::PublishUpdateDuration(static_cast<int>(millis.count()));

  added_frames_[frame->id] = frame;
  last_keyframe_id_ = frame->id;
  last_frame_id_ = frame->id;
}

void NewSmoother::GetPoses(std::map<int, Pose3Stamped>& poses) const
{
  for (const auto& frame : added_frames_)
  {
    if (graph_manager_.IsFrameTracked(frame.first))
    {
      poses[frame.first] = Pose3Stamped{ ToPose(graph_manager_.GetPose(frame.first)), frame.second->timestamp };
    }
  }
}

void NewSmoother::GetLandmarks(std::map<int, LandmarkResult>& landmarks) const
{
  for (const auto& landmark : graph_manager_.GetLandmarks())
  {
    if (landmark.second)
    {
      landmarks.insert({ landmark.first, (*(landmark.second)).ToLandmarkResult() });
    }
  }
}

bool NewSmoother::IsInitialized() const
{
  return initialized_;
}
