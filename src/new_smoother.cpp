#include "new_smoother.h"
#include "global_params.h"
#include "ground_truth.h"
#include "gtsam_conversions.h"
#include "gtsam_helpers.h"
#include "depth_triangulation.h"
#include "feature_helpers.h"
#include "landmark_result.h"
#include "landmark_result_gtsam.h"
#include "debug_value_publisher.h"
#include "isam2_solver.h"
#include "incremental_fixed_lag_solver.h"
#include "feature_extractor.h"

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
  params.findUnusedFactorSlots = true;                  // This likely causes trouble for RemoveLandmark
  params.factorization = gtsam::ISAM2Params::CHOLESKY;  // Default: CHOLESKY. QR is slower and have no less ILS.
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

boost::shared_ptr<gtsam::noiseModel::Robust> MakeRangeNoise(LidarDepthResult depth_result)
{
  return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(GlobalParams::RobustRangeK()),
      gtsam::noiseModel::Isotropic::Sigma(1, std::max(depth_result.std_dev, GlobalParams::NoiseRange())));
}

boost::shared_ptr<gtsam::PreintegrationCombinedParams> MakeIMUParams()
{
  auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(GlobalParams::IMUG());
  imu_params->accelerometerCovariance = gtsam::I_3x3 * std::pow(GlobalParams::IMUAccelNoiseDensity(), 2.0);
  imu_params->gyroscopeCovariance = gtsam::I_3x3 * std::pow(GlobalParams::IMUGyroNoiseDensity(), 2.0);
  imu_params->biasAccCovariance = gtsam::I_3x3 * std::pow(GlobalParams::IMUAccelRandomWalk(), 2.0);
  imu_params->biasOmegaCovariance = gtsam::I_3x3 * std::pow(GlobalParams::IMUGyroRandomWalk(), 2.0);
  imu_params->integrationCovariance = gtsam::I_3x3 * 1e-8;  // Try increasing this by factor of ~100-1000
  imu_params->biasAccOmegaInt = gtsam::I_6x6;  // This is the default

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

NewSmoother::NewSmoother(std::shared_ptr<IMUQueue> imu_queue,
                         std::shared_ptr<TimeOffsetProvider> lidar_time_offset_provider,
                         const std::shared_ptr<RefinedCameraMatrixProvider>& refined_camera_matrix_provider,
                         const std::shared_ptr<BetweenTransformProvider>& between_transform_provider)
  : between_noise_(gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector3::Constant(GlobalParams::NoiseBetweenRotation()),
         gtsam::Vector3::Constant(GlobalParams::NoiseBetweenTranslation()))
            .finished()))
  , between_noise_keyframe_(gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector3::Constant(GlobalParams::NoiseBetweenRotationKeyframe()),
         gtsam::Vector3::Constant(GlobalParams::NoiseBetweenTranslationKeyframe()))
            .finished()))
  , feature_noise_(gtsam::noiseModel::Isotropic::Sigma(2, GlobalParams::NoiseFeature()))
  , feature_m_estimator_(gtsam::noiseModel::mEstimator::Huber::Create(GlobalParams::RobustFeatureK()))
  , range_noise_(
        gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(GlobalParams::RobustRangeK()),
                                          gtsam::noiseModel::Isotropic::Sigma(1, GlobalParams::NoiseRange())))
  , graph_manager_(GetIncrementalSolver(), MakeSmartFactorParams(), GlobalParams::SmootherLag(),
                                GlobalParams::LandmarkRemovalHighDelta())
  , imu_integrator_(std::move(imu_queue), MakeIMUParams(), gtsam::imuBias::ConstantBias())
  , lidar_time_offset_provider_(std::move(lidar_time_offset_provider))
  , between_transform_provider_(between_transform_provider)
  , body_p_cam_(gtsam::make_shared<gtsam::Pose3>(
        gtsam::Rot3::Quaternion(GlobalParams::BodyPCamQuat()[3], GlobalParams::BodyPCamQuat()[0],
                                GlobalParams::BodyPCamQuat()[1], GlobalParams::BodyPCamQuat()[2]),
        gtsam::Point3(GlobalParams::BodyPCamVec()[0], GlobalParams::BodyPCamVec()[1], GlobalParams::BodyPCamVec()[2])))
{
  auto K_cv = refined_camera_matrix_provider->GetRefinedCameraMatrix();
  K_ = gtsam::make_shared<gtsam::Cal3_S2>(K_cv.at<double>(0, 0), K_cv.at<double>(1, 1), 0.0, K_cv.at<double>(0, 2),
                                          K_cv.at<double>(1, 2));
  K_->print();
}

void NewSmoother::SetFrontend(std::shared_ptr<FeatureExtractor> frontend)
{
  feature_extractor_ = std::move(frontend);
}

gtsam::Point3 NewSmoother::CalculatePointEstimate(const gtsam::Pose3& pose, const gtsam::Point2& pt, double depth) const
{
  gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pose * *body_p_cam_, *K_);
  return DepthTriangulation::PixelAndDepthToPoint3(pt, depth, camera);
}

double NewSmoother::CalculateParallax(const std::shared_ptr<Track>& track) const
{
  // Obtain first observable feature by iterating from the beginning of the track
  boost::optional<std::shared_ptr<Feature>> first_feature;
  for (auto& feature : track->features)
  {
    if (graph_manager_.IsFrameTracked(feature->frame_id))
    {
      first_feature = feature;
      break;
    }
  }
  if (!first_feature)
  {
    return -1;
  }

  // Obtain last observable feature by iterating backwards from the end of the track
  boost::optional<std::shared_ptr<Feature>> second_feature;
  for (auto feature_it = track->features.rbegin(); feature_it != track->features.rend(); ++feature_it)
  {
    if (graph_manager_.IsFrameTracked((*feature_it)->frame_id))
    {
      second_feature = *feature_it;
      break;
    }
  }
  if (!second_feature || (*first_feature)->frame_id >= (*second_feature)->frame_id)
  {
    return -1;
  }

  // Obtain the rotation between the two frames
  auto pose1 = graph_manager_.GetPose((*first_feature)->frame_id);
  auto pose2 = graph_manager_.GetPose((*second_feature)->frame_id);
  if (!pose1 || !pose2)
  {
    return -1;
  }
  auto pt1 = FromCvPoint((*first_feature)->pt);
  auto pt2 = FromCvPoint((*second_feature)->pt);
  if (GlobalParams::UseAngleParallax())
  {
    return ComputeParallaxAngle(pt1, pt2, *pose1, *pose2, K_, *body_p_cam_);
  }
  else
  {
    auto R1 = pose1->rotation();
    auto R2 = pose2->rotation();
    auto R12 = R1.between(R2);
    return ComputeParallaxWithOpenCV(pt1, pt2, R12, K_, body_p_cam_->rotation());
  }
}

void NewSmoother::InitializeProjLandmarkWithDepth(int lmk_id, int frame_id, double timestamp, const gtsam::Point2& pt,
                                                  LidarDepthResult depth_result, const gtsam::Pose3& init_pose)
{
  std::cout << "Initializing lmk " << lmk_id << " with depth " << std::endl;
  graph_manager_.InitProjectionLandmark(lmk_id, frame_id, timestamp, pt,
                                        CalculatePointEstimate(init_pose, pt, depth_result.depth), K_, *body_p_cam_,
                                        feature_noise_, feature_m_estimator_);
  graph_manager_.AddRangeObservation(lmk_id, frame_id, timestamp, depth_result.depth, MakeRangeNoise(depth_result));
}

gtsam::TriangulationResult NewSmoother::TriangulateTrack(const backend::Track& track) const
{
  std::vector<gtsam::PinholeCamera<gtsam::Cal3_S2>> cameras;
  std::vector<gtsam::Point2> measurements;
  gtsam::Point2 pt_for_first_factor;
  for (const auto& feature : track.features)
  {
    auto pose = graph_manager_.GetPose(feature.frame_id);
    if (pose)
    {
      cameras.emplace_back(*pose * *body_p_cam_, *K_);
      measurements.emplace_back(feature.pt.x, feature.pt.y);
    }
  }

  if (measurements.size() < 2)
  {
    return gtsam::TriangulationResult::Degenerate();
  }

  gtsam::TriangulationResult triangulation_result;

  try
  {
    triangulation_result =
        DepthTriangulation::Triangulate(measurements, cameras, MakeSmartFactorParams().getTriangulationParameters());
  }
  catch (std::exception& e)
  {
    std::cout << "Caught exception during Triangulate" << std::endl;
    std::cout << e.what() << std::endl;
  }

  return triangulation_result;
}

bool NewSmoother::TryInitializeProjLandmarkByTriangulation(int lmk_id, int frame_id, double timestamp,
                                                           const backend::Track& track)
{
  std::cout << "Trying to initialize lmk " << lmk_id << " by triangulation" << std::endl;
  gtsam::Point2 pt_for_first_factor;
  for (const auto& feature : track.features)
  {
    if (feature.frame_id == frame_id)
    {
      pt_for_first_factor = gtsam::Point2(feature.pt.x, feature.pt.y);
      break;
    }
  }

  auto triangulation_result = TriangulateTrack(track);

  if (!triangulation_result)
  {
    return false;
  }

  auto newest_pose = graph_manager_.GetPose(last_frame_id_);
  if (!newest_pose)
  {
    std::cout << "Warn: Could not get pose for last frame id " << last_frame_id_ << std::endl;
    return false;
  }
  auto range = newest_pose->range(*triangulation_result);
  if (range > GlobalParams::ProjLandmarkInitDistanceThresh())
  {
    std::cout << "Proj lmk at range " << range << " rejected" << std::endl;
    return false;
  }
  std::cout << "Initialized l" << lmk_id << " at range " << range << std::endl;

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

void NewSmoother::TryAddBetweenConstraint(int frame_id_1, int frame_id_2, double timestamp_1, double timestamp_2,
                                          const boost::shared_ptr<gtsam::noiseModel::Base>& noise)
{
  if (!GlobalParams::LoamBetweenFactorsEnabled())
  {
    return;
  }
  double offset_1 = lidar_time_offset_provider_->GetOffset(timestamp_1);
  double offset_2 = lidar_time_offset_provider_->GetOffset(timestamp_2);
  auto between_tf = between_transform_provider_->GetBetweenTransform(timestamp_1 - offset_1, timestamp_2 - offset_2);
  if (between_tf)
  {
    graph_manager_.AddBetweenFactor(frame_id_1, frame_id_2, *between_tf, noise);
  }
}

void NewSmoother::Initialize(const backend::FrontendResult& frame,
                             const boost::optional<std::pair<double, double>>& imu_gravity_alignment_timestamps)
{
  keyframe_timestamps_.AddKeyframeTimestamp(frame.timestamp);

  auto init_pose = GlobalParams::InitOnGroundTruth() ? ToGtsamPose(GroundTruth::At(frame.timestamp)) : gtsam::Pose3();
  auto gt_init_pose_yaw_only =
      gtsam::Pose3(gtsam::Rot3::Ypr(init_pose.rotation().yaw(), 0., 0.), init_pose.translation());

  auto refined_rotation = imu_gravity_alignment_timestamps ?
      imu_integrator_.RefineInitialAttitude(imu_gravity_alignment_timestamps->first,
                                            imu_gravity_alignment_timestamps->second, gtsam::Rot3()) : gtsam::Rot3();

  auto init_translation =
      GlobalParams::InitOnGroundTruth() ? gt_init_pose_yaw_only.translation() : gtsam::Point3::Zero();
  auto init_rotation = gtsam::Rot3::Ypr(GlobalParams::InitOnGroundTruth() ? gt_init_pose_yaw_only.rotation().yaw() : 0.,
                                        refined_rotation.pitch(), refined_rotation.roll());

  auto unrefined_init_pose = GlobalParams::InitOnGroundTruth() ? ToGtsamPose(GroundTruth::At(frame.timestamp)) : gtsam::Pose3();
  auto refined_init_pose = gtsam::Pose3(init_rotation, init_translation);

  refined_init_pose.print("Init pose: ");

  auto noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3(GlobalParams::PriorNoiseXRollPitch(), GlobalParams::PriorNoiseXRollPitch(),
                                          GlobalParams::PriorNoiseXYaw()),
       gtsam::Vector3::Constant(GlobalParams::PriorNoiseXTranslation()))
          .finished());
  auto noise_v = gtsam::noiseModel::Isotropic::Sigma(3, frame.stationary ? 0.001 : GlobalParams::PriorNoiseVelocity());
  auto noise_b = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(GlobalParams::PriorNoiseAccel()),
       gtsam::Vector3::Constant(GlobalParams::PriorNoiseGyro()))
          .finished());

  auto init_velocity = gtsam::Vector3::Zero();
  auto init_bias = gtsam::imuBias::ConstantBias(
      gtsam::Vector3(GlobalParams::IMUInitBiasAccel()[0], GlobalParams::IMUInitBiasAccel()[1],
                     GlobalParams::IMUInitBiasAccel()[2]),
      gtsam::Vector3(GlobalParams::IMUInitBiasGyro()[0], GlobalParams::IMUInitBiasGyro()[1],
                     GlobalParams::IMUInitBiasGyro()[2]));

  gtsam::NavState init_nav_state(refined_init_pose, init_velocity);

  graph_manager_.SetInitNavstate(frame.frame_id, frame.timestamp, init_nav_state, init_bias, noise_x, noise_v, noise_b);

  // We skip any landmark initialization for the first frame and instead rely on between factors

  graph_manager_.Update();
  DoExtraUpdateSteps(GlobalParams::ExtraISAM2UpdateSteps());
  added_frames_[frame.frame_id] = frame;
  last_frame_id_ = frame.frame_id;
  last_keyframe_id_ = frame.frame_id;
  initialized_ = true;
}

void NewSmoother::UpdateTrackParallaxes(const std::shared_ptr<Frame>& frame)
{
  if (added_frames_.size() > GlobalParams::MinKeyframesForNominal())
  {
    int count = 0;
    auto time_before = std::chrono::system_clock::now();
    for (auto& feature_pair : frame->features)
    {
      auto feature = feature_pair.second.lock();
      if (feature)
      {
        auto track = feature->track.lock();
        if (track && track->max_parallax < GlobalParams::MinParallaxForSmoothing())
        {
          auto parallax = CalculateParallax(track);
          track->max_parallax = std::max(parallax, track->max_parallax);
          count++;
        }
      }
    }
    auto time_after = std::chrono::system_clock::now();
    auto micros = std::chrono::duration_cast<std::chrono::microseconds>(time_after - time_before);
    double millis = static_cast<double>(micros.count()) / 1000.;
    std::cout << "Computed parallaxes for " << count << " tracks (took " << millis << "ms)" << std::endl;
  }
}

void NewSmoother::InitializeNewLandmarks(const std::vector<backend::Track>& new_tracks, int frame_id,
                                         const gtsam::Pose3& pred_pose, double timestamp_for_values)
{
  auto added_landmarks_count = 0;
  for (const auto& track : new_tracks)
  {
    auto obs_count = 0;
    if (track.depth_feature_count > 0)
    {
      if (track.max_parallax > GlobalParams::MinParallaxForSmoothingDepth() &&
          track.features.size() > GlobalParams::MinTrackLengthForSmoothingDepth() &&
          track.depth_feature_count > GlobalParams::MinDepthMeasurementsForSmoothing())
      {
        // If we have depth, find the first feature with depth, and use it to initialize the landmark
        // First, we need to get frame we first observed the features from, because this will be used
        int frame_id_first_seen = -1;
        boost::optional<backend::Feature> first_feature;
        for (const auto& feature : track.features)
        {
          if (graph_manager_.CanAddObservationsForFrame(feature.frame_id, feature.timestamp))
          {
            frame_id_first_seen = feature.frame_id;
            first_feature = feature;
            break;
          }
        }
        if (frame_id_first_seen == -1)
        {
          continue;
        }
        int frame_id_used_for_init = -1;
        for (const auto& feature : track.features)
        {
          if (feature.depth &&
              graph_manager_.CanAddObservationsForFrame(feature.frame_id, feature.timestamp))
          {
            std::cout << "Initializing lmk " << track.id << " with depth " << std::endl;
            auto pose_for_init = (feature.frame_id == frame_id) ? pred_pose : graph_manager_.GetPose(feature.frame_id);
            if (!pose_for_init)
            {
              continue;
            }
            gtsam::Point2 pt_for_init(feature.pt.x, feature.pt.y);
            // The initial point3 estimate can be obtained from any frame, so we use the first available with depth
            auto init_point_estimate = CalculatePointEstimate(*pose_for_init, pt_for_init, feature.depth->depth);

            // We do however need to use the first seen feature in the call to InitProjectionLandmark,
            // as we can only add observations that come after this frame.
            auto first_feature_pt = gtsam::Point2(first_feature->pt.x, first_feature->pt.y);
            // Careful with this. May be something weird going on with the timestamp
            auto ts_for_init =
                keyframe_timestamps_.GetMostRecentKeyframeTimestamp(track.features.back().timestamp);
            if (graph_manager_.IsFrameTracked(feature.frame_id))
            {
              auto triangulation_result = TriangulateTrack(track);
              if (triangulation_result)
              {
                auto range_from_triangulation = graph_manager_.GetPose(feature.frame_id)->range(*triangulation_result);
                std::cout << "Range from triangulation is " << range_from_triangulation << " while range from lidar is "
                          << feature.depth->depth << std::endl;
                if (std::abs(range_from_triangulation - feature.depth->depth) > 1.5) {
                  std::cout << "Rejecting because range from triangulation too off (> 1.5)!" << std::endl;
                  break;
                }
              }
              else {
                std::cout << "Triangulation of l" << track.id << " failed, but we have range, so it's fine?"
                          << std::endl;
              }
            }
            graph_manager_.InitProjectionLandmark(track.id, frame_id_first_seen, ts_for_init, first_feature_pt,
                                                  init_point_estimate, K_, *body_p_cam_, feature_noise_,
                                                  feature_m_estimator_);
            graph_manager_.AddRangeObservation(track.id, feature.frame_id, timestamp_for_values,
                                               feature.depth->depth, MakeRangeNoise(*feature.depth));
            frame_id_used_for_init = frame_id_first_seen;
            obs_count++;
            added_landmarks_count++;

            {
              std::cout << "Range values for l" << track.id << ": ";
              for (const auto& feat : track.features)
              {
                if (feat.depth)
                {
                  std::cout << feat.depth->depth << " ";
                }
              }
              std::cout << std::endl;
            }

            break;
          }
        }
        if (frame_id_used_for_init == -1)
        {
          std::cout << "WARN: " << track.id << " reports to have depth but could not find any." << std::endl;
        }

        // After initialization, add observations for all other features in the track
        for (auto& feature : track.features)
        {
          if (feature.frame_id == frame_id_used_for_init ||
              !graph_manager_.CanAddObservation(track.id, feature.frame_id))
          {
            continue;
          }
          gtsam::Point2 gtsam_pt = gtsam::Point2(feature.pt.x, feature.pt.y);
          if (feature.depth)
          {
            if (!graph_manager_.CanAddRangeObservation(track.id, feature.frame_id))
            {
              auto pose_for_init =
                  (feature.frame_id == frame_id) ? pred_pose : graph_manager_.GetPose(feature.frame_id);
              if (!pose_for_init)
              {
                continue;
              }
              auto init_point = CalculatePointEstimate(*pose_for_init, gtsam_pt, feature.depth->depth);
              graph_manager_.ConvertSmartFactorToProjectionFactor(track.id, timestamp_for_values, init_point);
            }
            graph_manager_.AddRangeObservation(track.id, feature.frame_id, timestamp_for_values,
                                               feature.depth->depth, MakeRangeNoise(*feature.depth));
          }
          graph_manager_.AddLandmarkObservation(track.id, feature.frame_id, timestamp_for_values, gtsam_pt, K_,
                                                *body_p_cam_);
          obs_count++;
        }
        std::cout << "Added proj with " << obs_count << " observations" << std::endl;
      }
    }
    else if (track.features.size() >= GlobalParams::MinTrackLengthForSmoothing())
    {
      auto lmk_initialized = false;
      for (const auto& feature : track.features)
      {
        if (!graph_manager_.CanAddObservationsForFrame(feature.frame_id, feature.timestamp))
        {
          continue;
        }
        if (lmk_initialized)
        {
          if (graph_manager_.CanAddObservation(track.id, feature.frame_id))
          {
            graph_manager_.AddLandmarkObservation(track.id, feature.frame_id, timestamp_for_values,
                                                  gtsam::Point2(feature.pt.x, feature.pt.y), K_, *body_p_cam_);
            obs_count++;
          }
        }
        else
        {
          if (GlobalParams::EnableSmartFactors())
          {
            InitializeStructurelessLandmark(track.id, feature.frame_id, feature.timestamp,
                                            gtsam::Point2(feature.pt.x, feature.pt.y));
          }
          else
          {
            if (track.max_parallax < GlobalParams::MinParallaxForSmoothing())
            {
              goto for_tracks;
            }
            auto ts_for_init =
                keyframe_timestamps_.GetMostRecentKeyframeTimestamp(track.features.back().timestamp);
            auto success = TryInitializeProjLandmarkByTriangulation(track.id, feature.frame_id, ts_for_init, track);
            if (!success)
            {
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
      }
    }
    for_tracks:;
  }
  std::cout << "Initialized " << added_landmarks_count << " landmarks" << std::endl;
}

void NewSmoother::AddLandmarkObservations(const std::vector<backend::Track>& existing_tracks, int frame_id,
                                          const gtsam::Pose3& pred_pose, double timestamp_for_values)
{
  auto feature_obs_count = 0;
  auto range_obs_count = 0;
  for (const auto& track : existing_tracks)
  {
    auto lmk_id = track.id;
    auto feature = track.features.back();
    if (!graph_manager_.CanAddObservation(lmk_id, feature.frame_id))
    {
      continue;
    }
    gtsam::Point2 gtsam_pt(feature.pt.x, feature.pt.y);
    if (feature.depth)
    {
      if (graph_manager_.IsSmartFactorLandmark(lmk_id))
      {
        auto init_point = CalculatePointEstimate(pred_pose, gtsam_pt, feature.depth->depth);
        graph_manager_.ConvertSmartFactorToProjectionFactor(lmk_id, timestamp_for_values, init_point);
      }
      graph_manager_.AddRangeObservation(lmk_id, frame_id, timestamp_for_values, feature.depth->depth,
                                         MakeRangeNoise(*feature.depth));
      {
        std::cout << "Range values for l" << lmk_id << ": ";
        for (const auto& feat : track.features)
        {
          if (feat.depth)
          {
            std::cout << feat.depth->depth << " ";
          }
        }
        std::cout << std::endl;
      }
      range_obs_count++;
    }
    feature_obs_count++;
    graph_manager_.AddLandmarkObservation(lmk_id, frame_id, timestamp_for_values, gtsam_pt, K_, *body_p_cam_);
  }

  std::cout << "Added " << feature_obs_count << " observations (" << range_obs_count << " range obs.)" << std::endl;
}

void NewSmoother::AddKeyframe(const backend::FrontendResult& frontend_result, bool is_keyframe)
{
  if (frontend_result.is_keyframe)
  {
    keyframe_timestamps_.AddKeyframeTimestamp(frontend_result.timestamp);
  }
  auto timestamp_for_values = keyframe_timestamps_.GetMostRecentKeyframeTimestamp(frontend_result.timestamp);
  if (!timestamp_for_values)
  {
    std::cout << "Got boost::none from keyframe_timestamps_. Maybe we received out of order frames?" << std::endl;
    exit(1);
  }

  if (is_keyframe)
  {
    std::cout << "Adding kf " << last_keyframe_id_ << " -> " << frontend_result.frame_id << std::endl;
  }
  else
  {
    std::cout << "Adding regular frame " << frontend_result.frame_id << std::endl;
  }
  imu_integrator_.WaitAndIntegrate(added_frames_[last_frame_id_].timestamp, frontend_result.timestamp);
  auto prev_nav_state = graph_manager_.GetNavState(last_frame_id_);
  auto prev_bias = graph_manager_.GetBias(last_frame_id_);
  if (!prev_nav_state || !prev_bias)
  {
    std::cout << "Fatal: Did not find nav state and bias for prev frame " << last_frame_id_ << std::endl;
    exit(1);
  }
  auto predicted_nav_state = imu_integrator_.PredictNavState(*prev_nav_state, *prev_bias);
  auto pim = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imu_integrator_.GetPim());
  graph_manager_.AddFrame(frontend_result.frame_id, timestamp_for_values, pim, predicted_nav_state, *prev_bias);
  imu_integrator_.ResetIntegration();

  std::vector<backend::Track> new_tracks;
  std::vector<backend::Track> existing_tracks;
  for (auto& track : frontend_result.active_tracks)
  {
    if (graph_manager_.IsLandmarkTracked(track.id))
    {
      existing_tracks.push_back(track);
    }
    else
    {
      new_tracks.push_back(track);
    }
  }

  if (!GlobalParams::LoamIMUOnly())
  {
    InitializeNewLandmarks(new_tracks, frontend_result.frame_id, predicted_nav_state.pose(), timestamp_for_values);
    AddLandmarkObservations(existing_tracks, frontend_result.frame_id, predicted_nav_state.pose(),
                            timestamp_for_values);
  }

  if (GlobalParams::FrameBetweenFactors())
  {
    TryAddBetweenConstraint(last_frame_id_, frontend_result.frame_id, added_frames_[last_frame_id_].timestamp,
                            frontend_result.timestamp,
                            between_noise_);
  }
  if (GlobalParams::KeyframeBetweenFactors() && is_keyframe)
  {
    TryAddBetweenConstraint(last_keyframe_id_, frontend_result.frame_id, added_frames_[last_keyframe_id_].timestamp,
                            frontend_result.timestamp,
                            between_noise_keyframe_);
  }

  auto time_before = std::chrono::system_clock::now();
  auto isam_result = graph_manager_.Update();
  auto time_after = std::chrono::system_clock::now();
  DebugValuePublisher::PublishRelinearizedCliques(static_cast<int>(isam_result.variablesRelinearized));
  DebugValuePublisher::PublishReeliminatedCliques(static_cast<int>(isam_result.variablesReeliminated));
  DebugValuePublisher::PublishTotalCliques(static_cast<int>(isam_result.cliques));
  auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(time_after - time_before);
  DebugValuePublisher::PublishUpdateDuration(static_cast<int>(millis.count()));

  DoExtraUpdateSteps(GlobalParams::ExtraISAM2UpdateSteps());

  auto bias = graph_manager_.GetBias(frontend_result.frame_id);
  if (bias)
  {
    std::vector<double> bias_acc = { bias->accelerometer().x(), bias->accelerometer().y(), bias->accelerometer().z() };
    std::vector<double> bias_gyro = { bias->gyroscope().x(), bias->gyroscope().y(), bias->gyroscope().z() };
    DebugValuePublisher::PublishBias(bias_acc, bias_gyro);
  }

  if (isam_result.errorAfter)
  {
    DebugValuePublisher::PublishNonlinearError(*isam_result.errorAfter);
  }

  if (isam_result.detail)
  {
    int poses_relinearized = 0;
    int landmarks_relinearized = 0;
    int velocities_relinearized = 0;
    int biases_relinearized = 0;
    for (const auto& x : isam_result.detail->variableStatus)
    {
      switch (gtsam::_defaultKeyFormatter(x.first)[0])
      {
        case 'x':
          poses_relinearized++;
          break;
        case 'l':
          landmarks_relinearized++;
          break;
        case 'v':
          velocities_relinearized++;
          break;
        case 'b':
          biases_relinearized++;
          break;
      }
      // std::cout << "========== " << gtsam::_defaultKeyFormatter(x.first) << " ==========" << std::endl;
      // PrintVariableStatus(x.second);
    }
    std::cout << poses_relinearized << " poses relinearized" << std::endl;
    std::cout << landmarks_relinearized << " landmarks relinearized" << std::endl;
    std::cout << velocities_relinearized << " velocities relinearized" << std::endl;
    std::cout << biases_relinearized << " biases relinearized" << std::endl;
  }

  if (!GlobalParams::GroundTruthFile().empty())
  {
    auto ts_offset = lidar_time_offset_provider_->GetOffset(frontend_result.timestamp);
    auto pose_estimate = graph_manager_.GetPose(frontend_result.frame_id);
    if (pose_estimate)
    {
      auto abs_gt_error = ToGtsamPose(GroundTruth::At(frontend_result.timestamp - ts_offset)).range(*pose_estimate);
      DebugValuePublisher::PublishAbsoluteGroundTruthError(abs_gt_error);
    }
  }

  added_frames_[frontend_result.frame_id] = frontend_result;
  last_frame_id_ = frontend_result.frame_id;

  if (is_keyframe)
  {
    last_keyframe_id_ = frontend_result.frame_id;
  }

  PublishHighDeltaTrackImage();

  RemoveUntrackedFramesFromBookkeeping();
}

void NewSmoother::PublishHighDeltaTrackImage()
{
  for (const auto& delta : graph_manager_.GetDelta())
  {
    int delta_danger_thresh = 5;
    if (delta.second.norm() > delta_danger_thresh)
    {
      std::cout << "=== " << gtsam::_defaultKeyFormatter(delta.first) << " delta is quite high! ===" << std::endl;
      std::cout << "delta = " << delta.second << std::endl;
      std::cout << "norm(delta) = " << delta.second.norm() << " > " << delta_danger_thresh << std::endl;
      auto track_id = static_cast<int>(gtsam::Symbol(delta.first).index());

      for (auto it = added_frames_.rbegin(); it != added_frames_.rend(); ++it)
      {
        for (const auto& track : it->second.active_tracks)
        {
          if (track.id == track_id)
          {
            feature_extractor_->PublishSingleTrackImage(track);
            return;
          }
        }
      }
    }
  }
}

void NewSmoother::DoExtraUpdateSteps(int steps)
{
  auto time_before = std::chrono::system_clock::now();
  for (int i = 0; i < steps; ++i)
  {
    graph_manager_.Update();
  }
  auto time_after = std::chrono::system_clock::now();
  auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(time_after - time_before);
  DebugValuePublisher::PublishExtraUpdatesDuration(static_cast<int>(millis.count()));
}

void NewSmoother::GetPoses(std::map<int, Pose3Stamped>& poses) const
{
  for (const auto& frame : added_frames_)
  {
    auto pose = graph_manager_.GetPose(frame.first);
    if (graph_manager_.IsFrameTracked(frame.first))
    {
      poses[frame.first] = Pose3Stamped{ ToPose(*pose), frame.second.timestamp };
    }
  }
}

void NewSmoother::RemoveUntrackedFramesFromBookkeeping()
{
  for (auto it = added_frames_.cbegin(); it != added_frames_.cend(); /* no increment */)
  {
    if (!graph_manager_.IsFrameTracked(it->second.frame_id))
    {
      added_frames_.erase(it++);
    }
    else {
      // When reaching the first tracked frame, we know that all following frames are tracked, so we just return
      return;
    }
  }
}

boost::optional<Pose3Stamped> NewSmoother::GetLatestLidarPose()
{
  auto latest_world_T_body = graph_manager_.GetPose(last_frame_id_);
  if (!latest_world_T_body)
  {
    return boost::none;
  }
  // TODO move to member variable
  gtsam::Pose3 body_p_lidar(
      gtsam::Rot3::Quaternion(GlobalParams::BodyPLidarQuat()[3], GlobalParams::BodyPLidarQuat()[0],
                              GlobalParams::BodyPLidarQuat()[1], GlobalParams::BodyPLidarQuat()[2]),
      gtsam::Point3(GlobalParams::BodyPLidarVec()[0], GlobalParams::BodyPLidarVec()[1],
                    GlobalParams::BodyPLidarVec()[2]));
  auto latest_world_T_lidar = *latest_world_T_body * body_p_lidar;
  auto ts_cam = added_frames_[last_frame_id_].timestamp;
  auto ts_offset = lidar_time_offset_provider_->GetOffset(ts_cam);
  auto ts_lidar = ts_cam - ts_offset;
  return Pose3Stamped { ToPose(latest_world_T_lidar), ts_lidar };
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
