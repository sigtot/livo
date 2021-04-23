#include "smoother.h"
#include "key_point_observation.h"
#include "gtsam_conversions.h"
#include "pose3_stamped.h"
#include "global_params.h"
#include "newer_college_ground_truth.h"
#include "gtsam_helpers.h"
#include "debug_image_publisher.h"
#include "debug_value_publisher.h"

#include <iostream>
#include <utility>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/base/types.h>
#include <thread>
#include <chrono>
#include <gtsam/sam/RangeFactor.h>
#include <numeric>

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

gtsam::ISAM2Params MakeIsam2Params()
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

std::shared_ptr<gtsam::PreintegrationType> Smoother::MakeIMUIntegrator()
{
  auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(GlobalParams::IMUG());
  // imu_params->n_gravity = gtsam::Vector3(GlobalParams::IMUNGravityX(), GlobalParams::IMUNGravityY(),
  // GlobalParams::IMUNGravityZ());
  imu_params->accelerometerCovariance = gtsam::I_3x3 * std::pow(GlobalParams::IMUAccelNoiseDensity(), 2.0);
  imu_params->gyroscopeCovariance = gtsam::I_3x3 * std::pow(GlobalParams::IMUGyroNoiseDensity(), 2.0);
  imu_params->biasAccCovariance = gtsam::I_3x3 * std::pow(GlobalParams::IMUAccelRandomWalk(), 2.0);
  imu_params->biasOmegaCovariance = gtsam::I_3x3 * std::pow(GlobalParams::IMUGyroRandomWalk(), 2.0);
  imu_params->integrationCovariance = gtsam::I_3x3 * 1e-8;
  // imu_params->biasAccOmegaInt = gtsam::I_6x6 * 1e-5;  // error in the bias used for preintegration
  /*
  gtsam::Pose3 imu_to_cam = gtsam::Pose3(
      gtsam::Rot3::Quaternion(GlobalParams::IMUCamQuat()[3], GlobalParams::IMUCamQuat()[0],
                              GlobalParams::IMUCamQuat()[1], GlobalParams::IMUCamQuat()[2]),
      gtsam::Point3(GlobalParams::IMUCamVector()[0], GlobalParams::IMUCamVector()[1], GlobalParams::IMUCamVector()[2]));
  auto body_to_cam = gtsam::Pose3(gtsam::Rot3::Ypr(-M_PI / 2, 0, M_PI / 2), gtsam::Point3::Zero());
  auto cam_to_body = body_to_cam.inverse();
  imu_params->body_P_sensor = (imu_to_cam * cam_to_body).inverse();
  imu_params->body_P_sensor->print("imu body p sensor: ");
   */

  imu_params->body_P_sensor = *body_p_imu_;

  auto imu_bias = gtsam::imuBias::ConstantBias();  // Initialize at zero bias

  auto imu_measurements = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imu_params, imu_bias);
  imu_measurements->resetIntegration();
  return imu_measurements;
}

Smoother::Smoother(std::shared_ptr<IMUQueue> imu_queue)
  : isam2(new gtsam::ISAM2(MakeIsam2Params()))
  , graph_(new gtsam::NonlinearFactorGraph())
  , values_(new gtsam::Values())
  , imu_queue_(std::move(imu_queue))
  , imu_measurements_(MakeIMUIntegrator())
  , K_(new gtsam::Cal3_S2(GlobalParams::CamFx(), GlobalParams::CamFy(), 0.0, GlobalParams::CamU0(),
                          GlobalParams::CamV0()))
  , body_p_cam_(gtsam::make_shared<gtsam::Pose3>(
        gtsam::Rot3::Quaternion(GlobalParams::BodyPCamQuat()[3], GlobalParams::BodyPCamQuat()[0],
                                GlobalParams::BodyPCamQuat()[1], GlobalParams::BodyPCamQuat()[2]),
        gtsam::Point3(GlobalParams::BodyPCamVec()[0], GlobalParams::BodyPCamVec()[1], GlobalParams::BodyPCamVec()[2])))
  , body_p_imu_(gtsam::make_shared<gtsam::Pose3>(
        gtsam::Rot3::Quaternion(GlobalParams::BodyPImuQuat()[3], GlobalParams::BodyPImuQuat()[0],
                                GlobalParams::BodyPImuQuat()[1], GlobalParams::BodyPImuQuat()[2]),
        gtsam::Point3(GlobalParams::BodyPImuVec()[0], GlobalParams::BodyPImuVec()[1], GlobalParams::BodyPImuVec()[2])))
  , feature_noise_(gtsam::noiseModel::Isotropic::Sigma(2, GlobalParams::NoiseFeature()))
  , smart_factor_params_(
        std::make_shared<gtsam::SmartProjectionParams>(gtsam::HESSIAN, gtsam::IGNORE_DEGENERACY, false, true, 1e-5))
{
  smart_factor_params_->setDynamicOutlierRejectionThreshold(GlobalParams::DynamicOutlierRejectionThreshold());
}

void Smoother::UpdateSmartFactorParams(const gtsam::SmartProjectionParams& params)
{
  for (auto& smart_factor_pair : smart_factors_)
  {
    smart_factor_pair.second.reset();

    SmartFactor::shared_ptr smart_factor(new SmartFactor(feature_noise_, K_, *body_p_cam_, params));
    for (auto& feature : added_tracks_[smart_factor_pair.first]->features)
    {
      if (added_frame_timestamps_.count(feature->frame->id))
      {
        auto pt = feature->pt;
        smart_factor->add(gtsam::Point2(pt.x, pt.y), X(feature->frame->id));
        feature->in_smoother = true;
      }
    }
    smart_factor_pair.second = smart_factor;
  }
}

void Smoother::InitializeLandmarks(
    std::vector<KeyframeTransform> keyframe_transforms, const std::vector<shared_ptr<Track>>& tracks,
    const boost::optional<std::pair<std::shared_ptr<Frame>, std::shared_ptr<Frame>>>& frames_for_imu_init,
    std::vector<Pose3Stamped>& pose_estimates, std::map<int, Point3>& landmark_estimates)
{
  std::cout << "Let's initialize those landmarks" << std::endl;

  auto unrefined_init_pose =
      GlobalParams::InitOnGroundTruth() ?
          ToGtsamPose(GroundTruth::At(keyframe_transforms[0].frame1->timestamp)) :
          gtsam::Pose3(gtsam::Rot3::ypr(M_PI, -M_PI_2, -0.0001), gtsam::Point3(0.0001, -0.0001, -0.0001));

  init_pose_ = std::make_shared<gtsam::Pose3>(
      (frames_for_imu_init && GlobalParams::DoInitialGravityAlignment()) ?
          gtsam::Pose3(ToGtsamRot(imu_queue_->RefineInitialAttitude(frames_for_imu_init->first->timestamp,
                                                                    frames_for_imu_init->second->timestamp,
                                                                    ToRot(unrefined_init_pose.rotation()))),
                       unrefined_init_pose.translation()) :
          unrefined_init_pose);

  auto noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3(GlobalParams::PriorNoiseXRollPitch(), GlobalParams::PriorNoiseXRollPitch(),
                                          GlobalParams::PriorNoiseXYaw()),
       gtsam::Vector3::Constant(GlobalParams::PriorNoiseXTranslation()))
          .finished());

  graph_->addPrior(X(keyframe_transforms[0].frame1->id), *init_pose_, noise_x);

  values_->insert(X(keyframe_transforms[0].frame1->id), *init_pose_);

  added_frame_timestamps_[keyframe_transforms[0].frame1->id] = keyframe_transforms[0].frame1->timestamp;

  std::map<int, bool> frame_ids;
  frame_ids[keyframe_transforms[0].frame1->id] = true;

  // Initialize values on R and t from essential matrix
  std::vector<gtsam::Pose3> poses = { *init_pose_ };
  for (auto& keyframe_transform : keyframe_transforms)
  {
    added_frame_timestamps_[keyframe_transform.frame2->id] = keyframe_transform.frame2->timestamp;

    std::cout << "==================== Frame " << keyframe_transform.frame1->id << " -> "
              << keyframe_transform.frame2->id << " ====================" << std::endl;
    std::cout << (keyframe_transform.stationary ? "stationary" : "moving") << std::endl;
    if (keyframe_transform.Valid() && !keyframe_transform.stationary)
    {
      auto R_mat = ToMatrix3(*keyframe_transform.GetRotation());
      auto t = *keyframe_transform.GetTranslation();
      gtsam::Unit3 t_unit_cam_frame(t[0], t[1], t[2]);
      gtsam::Rot3 R_cam_frame(R_mat);

      gtsam::Unit3 t_i(*body_p_cam_ * t_unit_cam_frame.point3());
      gtsam::Rot3 R_i(body_p_cam_->rotation() * R_cam_frame * body_p_cam_->rotation().inverse());
      gtsam::Pose3 X_i = gtsam::Pose3(R_i, t_i.point3());

      gtsam::Pose3 X_world = poses.back().compose(X_i);
      poses.push_back(X_world);

      std::cout << "R_c = " << R_cam_frame.quaternion() << std::endl;
      t_unit_cam_frame.print("t_c = ");
      std::cout << "R_i = " << R_i.quaternion() << std::endl;
      t_i.print("t_i = ");
      std::cout << "R_world = " << X_world.rotation().quaternion() << std::endl;
      std::cout << "t_world = " << X_world.translation().transpose() << std::endl;
    }
    else
    {
      // Assume invalid transform means low motion, so use previous pose
      poses.push_back(poses.back());
    }

    std::cout << "pose R=" << poses.back().rotation().ypr().transpose() << std::endl;
    std::cout << "pose t=" << poses.back().translation().transpose() << std::endl;

    std::cout << "=========================================================" << std::endl;

    values_->insert(X(keyframe_transform.frame2->id), poses.back());

    frame_ids[keyframe_transform.frame2->id] = true;
  }

  // The SfM-only problem has scale ambiguity, so we add a second prior
  auto noise_x_second_prior = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.1)).finished());
  auto second_prior = gtsam::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(keyframe_transforms.back().frame2->id),
                                                                           poses.back(), noise_x_second_prior);
  graph_->add(second_prior);

  for (auto& track : tracks)
  {
    SmartFactor::shared_ptr smart_factor(
        new SmartFactor(feature_noise_, K_, *body_p_cam_, *smart_factor_params_));
    std::vector<std::shared_ptr<Feature>> added_features;
    for (auto& feature : track->features)
    {
      if (frame_ids.count(feature->frame->id))
      {
        auto pt = feature->pt;
        smart_factor->add(gtsam::Point2(pt.x, pt.y), X(feature->frame->id));
        added_features.push_back(feature);
      }
    }
    if (smart_factor->size() >= GlobalParams::MinTrackLengthForSmoothing())
    {
      std::cout << "adding landmark " << track->id << " with " << smart_factor->size() << " observations and parallax "
                << track->max_parallax << " and inlier ratio " << track->InlierRatio() << std::endl;
      added_tracks_[track->id] = track;
      smart_factors_[track->id] = smart_factor;
      graph_->add(smart_factor);
      for (auto& feature : added_features)
      {
        feature->in_smoother = true;
      }
    }
  }
  std::cout << "Added " << smart_factors_.size() << " smart factors" << std::endl;

  try
  {
    gtsam::LevenbergMarquardtParams params;
    gtsam::LevenbergMarquardtOptimizer optimizer(*graph_, *values_);
    std::cout << "Error before LM step: " << optimizer.error() << std::endl;
    *values_ = optimizer.optimize();
    std::cout << "Error after LM step: " << optimizer.error() << std::endl;
  }
  catch (exception& e)
  {
    PublishReprojectionErrorImages();
    std::cout << e.what() << std::endl;
    throw e;
  }

  // TODO: This param is no longer used like this: We essentially use it as a bool now.
  // TODO: If it's > 0, we scale the solution by the scale given by the ground truth, while the given value is ignored
  // If an initial range factor length is provided, we scale the solution to account for it and redo the optimization
  if (GlobalParams::InitRangeFactorLength() > 0)
  {
    auto pose_delta_range = poses[0].inverse().compose(poses.back()).translation().norm();
    auto range = GlobalParams::InitRangeFactorLength();
    double scale_ratio = range / pose_delta_range;
    std::cout << "Gt range: " << range << " old range: " << pose_delta_range << " scale_ratio: " << scale_ratio
              << std::endl;
    auto frame_it = added_frame_timestamps_.begin();
    std::vector<std::pair<std::pair<int, int>, gtsam::Pose3>> scaled_between_poses;
    std::vector<std::pair<int, gtsam::Pose3>> scaled_poses{ { frame_it->first,
                                                              values_->at<gtsam::Pose3>(X(frame_it->first)) } };

    while (true)
    {
      auto pose1_id = frame_it->first;
      auto pose1 = values_->at<gtsam::Pose3>(X(frame_it->first));
      ++frame_it;
      if (frame_it == added_frame_timestamps_.end())
      {
        break;
      }
      auto pose2 = values_->at<gtsam::Pose3>(X(frame_it->first));
      auto between_pose = pose1.between(pose2);
      auto scaled_between_pose = gtsam::Pose3(between_pose.rotation(), scale_ratio * between_pose.translation());
      auto scaled_pose2 = scaled_poses.back().second.compose(scaled_between_pose);
      scaled_poses.emplace_back(frame_it->first, scaled_pose2);
      scaled_between_poses.emplace_back(std::pair<int, int>{ pose1_id, frame_it->first }, scaled_between_pose);
    }

    values_->clear();
    for (const auto& scaled_pose : scaled_poses)
    {
      values_->insert(X(scaled_pose.first), scaled_pose.second);
    }

    // The second prior is already added as a shared pointer in the graph, we update the pointer value here.
    *second_prior = gtsam::PriorFactor<gtsam::Pose3>(X(scaled_poses.back().first), scaled_poses.back().second,
                                                     noise_x_second_prior);

    try
    {
      gtsam::LevenbergMarquardtOptimizer optimizer(*graph_, *values_);
      std::cout << "Error before LM step: " << optimizer.error() << std::endl;
      *values_ = optimizer.optimize();
      std::cout << "Error after LM step: " << optimizer.error() << std::endl;
    }
    catch (exception& e)
    {
      PublishReprojectionErrorImages();
      std::cout << e.what() << std::endl;
      throw e;
    }
  }

  std::cout << "Done with scaled optimization" << std::endl;

  GetPoseEstimates(pose_estimates);
  GetLandmarkEstimates(landmark_estimates);

  // PublishReprojectionErrorImages();

  last_frame_id_added_ = keyframe_transforms.back().frame2->id;

  if (GlobalParams::SaveFactorGraphsToFile())
  {
    SaveGraphToFile("/tmp/landmarks-graph.dot", *graph_, *values_);
  }

  initialized_ = true;
}

void Smoother::PublishReprojectionErrorImages()
{
  std::map<int, std::vector<cv::Point2f>> measured_points_by_frame;
  std::map<int, std::vector<cv::Point2f>> reprojected_points_by_frame;
  std::map<int, std::vector<bool>> inlier_masks_by_frame;
  std::map<int, std::shared_ptr<Frame>> frames_by_frame;
  for (const auto& added_frame_pair : added_frame_timestamps_)
  {
    measured_points_by_frame[added_frame_pair.first] = std::vector<cv::Point2f>{};
    reprojected_points_by_frame[added_frame_pair.first] = std::vector<cv::Point2f>{};
    inlier_masks_by_frame[added_frame_pair.first] = std::vector<bool>{};
  }
  for (const auto& smart_factor_pair : smart_factors_)
  {
    auto reproj_error = smart_factor_pair.second->reprojectionErrorAfterTriangulation(
        isam2->empty() ? *values_ : isam2->calculateEstimate());
    auto features = added_tracks_[smart_factor_pair.first]->features;

    int i = 0;
    for (const auto& feature : features)
    {
      if (feature->in_smoother)
      {
        if (!frames_by_frame.count(feature->frame->id))
        {
          frames_by_frame[feature->frame->id] = feature->frame;
        }
        auto reproj_error_x = static_cast<float>(reproj_error(2 * i, 0));
        auto reproj_error_y = static_cast<float>(reproj_error(2 * i + 1, 0));

        const float zero_thresh = 0.00001;
        inlier_masks_by_frame[feature->frame->id].push_back(std::abs(reproj_error_x) > zero_thresh ||
                                                            std::abs(reproj_error_y) > zero_thresh);

        measured_points_by_frame[feature->frame->id].push_back(feature->pt);
        auto point_reproj_error = cv::Point2f(reproj_error_x, reproj_error_y);
        cv::Point2f reproj_point = feature->pt + point_reproj_error;
        reprojected_points_by_frame[feature->frame->id].push_back(reproj_point);
        ++i;
      }
    }
  }

  for (const auto& added_frame_pair : added_frame_timestamps_)
  {
    auto frame_id = added_frame_pair.first;
    DebugImagePublisher::PublishReprojectionErrorImage(
        frames_by_frame[frame_id]->image, measured_points_by_frame[frame_id], reprojected_points_by_frame[frame_id],
        inlier_masks_by_frame[frame_id], added_frame_pair.second);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void Smoother::PublishNewReprojectionErrorImage(const gtsam::Values& values, const std::shared_ptr<Frame>& frame)
{
  std::vector<cv::Point2f> measured_points;
  std::vector<cv::Point2f> reprojected_points;
  std::vector<bool> inlier_mask;
  for (const auto& smart_factor_pair : smart_factors_)
  {
    auto reproj_error = smart_factor_pair.second->reprojectionErrorAfterTriangulation(values);
    auto features = added_tracks_[smart_factor_pair.first]->features;

    auto feature = features.back();
    if (feature->frame->id == frame->id && feature->in_smoother && reproj_error.rows() >= 2)
    {
      auto reproj_error_x = static_cast<float>(reproj_error(reproj_error.rows() - 2, 0));
      auto reproj_error_y = static_cast<float>(reproj_error(reproj_error.rows() - 1, 0));

      const float zero_thresh = 0.00001;
      inlier_mask.push_back(std::abs(reproj_error_x) > zero_thresh || std::abs(reproj_error_y) > zero_thresh);

      measured_points.push_back(feature->pt);
      auto point_reproj_error = cv::Point2f(reproj_error_x, reproj_error_y);
      cv::Point2f reproj_point = feature->pt + point_reproj_error;
      reprojected_points.push_back(reproj_point);
    }
  }

  DebugImagePublisher::PublishReprojectionErrorImage(frame->image, measured_points, reprojected_points, inlier_mask,
                                                     frame->timestamp);
}

void Smoother::GetPoseEstimates(std::vector<Pose3Stamped>& pose_estimates)
{
  for (auto& frame_pair : added_frame_timestamps_)
  {
    auto gtsam_pose = isam2->empty() ? values_->at<gtsam::Pose3>(X(frame_pair.first)) :
                                       isam2->calculateEstimate<gtsam::Pose3>(X(frame_pair.first));
    pose_estimates.push_back(Pose3Stamped{ .pose = ToPose(gtsam_pose), .stamp = frame_pair.second });
  }
}

void Smoother::GetDegenerateLandmarks(
    std::vector<std::pair<int, boost::shared_ptr<SmartFactor>>>& degenerate_landmarks) const
{
  for (const auto& it : smart_factors_)
  {
    auto smart_factor = it.second;
    if (!smart_factor->isValid())
    {
      degenerate_landmarks.emplace_back(it.first, it.second);
    }
  }
}

void Smoother::GetLandmarkEstimates(std::map<int, Point3>& landmark_estimates)
{
  auto values = isam2->empty() ? *values_ : isam2->calculateEstimate();
  for (const auto& smart_factor_pair : smart_factors_)
  {
    auto smart_factor = smart_factor_pair.second;
    if (smart_factor->isValid())
    {
      gtsam::Matrix E;
      bool worked = smart_factor->triangulateAndComputeE(E, values);
      if (worked)
      {
        auto P = smart_factor->PointCov(E);
        // std::cout << "point covariance: " << std::endl << P << std::endl;
        if (P.norm() < 1)
        {
          boost::optional<gtsam::Point3> point = smart_factor->point();
          if (point)  // I think this check is redundant because we already check if the factor is valid, but doesn't
                      // hurt
          {           // ignore if boost::optional returns nullptr
            landmark_estimates[smart_factor_pair.first] = ToPoint(*point);
            continue;
          }
        }
      }
      else
      {
        std::cout << "Triangulation failed for landmark " << smart_factor_pair.first << std::endl;
      }
    }
    landmark_estimates[smart_factor_pair.first] = Point3{ .x = 0, .y = 0, .z = 0 };

    /* debug reprojection
    gtsam::Point3 point = smart_factor->cameras(*values_).back().backproject(smart_factor->measured().back(), 10);
    landmark_estimates[smart_factor_pair.first] = ToPoint(point);
     */
  }
}

void Smoother::InitializeIMU(const std::vector<KeyframeTransform>& keyframe_transforms,
                             std::vector<Pose3Stamped>& pose_estimates, std::map<int, Point3>& landmark_estimates)
{
  graph_->resize(0);
  // Init on "random values" as this apparently helps convergence
  gtsam::imuBias::ConstantBias init_bias(gtsam::Vector3(-0.025266, 0.136696, 0.075593),
                                         gtsam::Vector3(-0.003172, 0.021267, 0.078502));

  auto init_translation_delta =
      values_->at<gtsam::Pose3>(X(keyframe_transforms.front().frame1->id))
          .translation()
          .between(values_->at<gtsam::Pose3>(X(keyframe_transforms.front().frame2->id)).translation());
  auto init_time_delta = keyframe_transforms.front().frame2->timestamp - keyframe_transforms.front().frame1->timestamp;

  auto init_velocity = keyframe_transforms.front().frame1->stationary ?
                           gtsam::Vector3(0.0000001, 0.0000001, 0.0000001) :
                           init_translation_delta / init_time_delta;  // assume constant velocity: v1 == v2
  std::vector<gtsam::Vector3> velocity_estimates = { init_velocity };
  for (auto& keyframe_transform : keyframe_transforms)
  {
    WaitForAndIntegrateIMU(keyframe_transform.frame1->timestamp, keyframe_transform.frame2->timestamp);

    auto imu_combined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imu_measurements_);
    gtsam::CombinedImuFactor imu_factor(X(keyframe_transform.frame1->id), V(keyframe_transform.frame1->id),
                                        X(keyframe_transform.frame2->id), V(keyframe_transform.frame2->id),
                                        B(keyframe_transform.frame1->id), B(keyframe_transform.frame2->id),
                                        imu_combined);
    graph_->add(imu_factor);

    /*
    auto velocity_prediction =
        imu_measurements_
            ->predict(
                gtsam::NavState(values_->at<gtsam::Pose3>(X(keyframe_transform.frame1->id)), velocity_estimates.back()),
                init_bias)
            .velocity();
            */

    auto translation_delta = values_->at<gtsam::Pose3>(X(keyframe_transform.frame1->id))
                                 .translation()
                                 .between(values_->at<gtsam::Pose3>(X(keyframe_transform.frame2->id)).translation());
    auto time_delta = keyframe_transform.frame2->timestamp - keyframe_transform.frame1->timestamp;

    std::cout << "delta vi: " << imu_measurements_->deltaVij() << std::endl;

    // v_k = (p_k+1 - p_k)/t - 1/2 a_k t (a bit shady: is deltaVij really a_k t?)
    // gtsam::Vector3 velocity_estimate = (translation_delta / time_delta) - 0.5 * imu_measurements_->deltaVij();
    gtsam::Vector3 velocity_estimate = gtsam::Vector3(0.0001, 0.0001, 0.0001);

    imu_measurements_->resetIntegration();

    velocity_estimates.push_back(velocity_estimate);
    std::cout << "v" << keyframe_transform.frame2->id << " = " << velocity_estimate << std::endl;

    values_->insert(V(keyframe_transform.frame2->id), velocity_estimate);
    values_->insert(B(keyframe_transform.frame2->id), init_bias);
  }

  auto noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3(GlobalParams::PriorNoiseXYaw(), GlobalParams::PriorNoiseXRollPitch(),
                                          GlobalParams::PriorNoiseXRollPitch()),
       gtsam::Vector3::Constant(GlobalParams::PriorNoiseXTranslation()))
          .finished());
  auto noise_v = gtsam::noiseModel::Isotropic::Sigma(
      3, keyframe_transforms[0].frame1->stationary ? 0.01 : GlobalParams::PriorNoiseVelocity());
  auto noise_b = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(GlobalParams::PriorNoiseAccel()),
       gtsam::Vector3::Constant(GlobalParams::PriorNoiseGyro()))
          .finished());

  graph_->addPrior(X(keyframe_transforms[0].frame1->id), *init_pose_, noise_x);
  graph_->addPrior(V(keyframe_transforms[0].frame1->id), init_velocity, noise_v);
  graph_->addPrior(B(keyframe_transforms[0].frame1->id), init_bias, noise_b);

  values_->insert(V(keyframe_transforms[0].frame1->id), init_velocity);
  values_->insert(B(keyframe_transforms[0].frame1->id), init_bias);

  std::map<gtsam::FactorIndex, int> new_factor_to_track_id;

  for (const auto& smart_factor_pair : smart_factors_)
  {
    // Use graph->size() as index so will correspond to ISAM2Result::newFactorIndices
    new_factor_to_track_id[graph_->size()] = smart_factor_pair.first;
    graph_->add(smart_factor_pair.second);
  }

  try
  {
    gtsam::LevenbergMarquardtOptimizer optimizer(*graph_, *values_);
    std::cout << "Error before LM step: " << optimizer.error() << std::endl;
    *values_ = optimizer.optimize();
    std::cout << "Error after LM step: " << optimizer.error() << std::endl;
  }
  catch (exception& e)
  {
    std::cout << "Got exception during batch IMU initialization" << std::endl;
    PublishReprojectionErrorImages();
    std::cout << e.what() << std::endl;
    throw e;
  }

  GetPoseEstimates(pose_estimates);
  GetLandmarkEstimates(landmark_estimates);

  try
  {
    auto isam_result = isam2->update(*graph_, *values_);
    isam_result.print("isam result: ");
    if (isam_result.errorBefore && isam_result.errorAfter)
    {
      std::cout << "error before after: " << *isam_result.errorBefore << " -> " << *isam_result.errorAfter << std::endl;
      DebugValuePublisher::PublishNonlinearError(*isam_result.errorAfter);
    }
    DebugValuePublisher::PublishRelinearizedCliques(static_cast<int>(isam_result.variablesRelinearized));
    DebugValuePublisher::PublishReeliminatedCliques(static_cast<int>(isam_result.variablesReeliminated));
    DebugValuePublisher::PublishTotalCliques(static_cast<int>(isam_result.cliques));

    for (const auto& factor_track_pair : new_factor_to_track_id)
    {
      track_id_to_factor_index_[factor_track_pair.second] = isam_result.newFactorsIndices.at(factor_track_pair.first);
    }
  }
  catch (exception& e)
  {
    std::cout << "Got exception during isam2 initialization" << std::endl;
    PublishReprojectionErrorImages();
    std::cout << e.what() << std::endl;
    throw e;
  }

  // GetPoseEstimates(pose_estimates);
  // GetLandmarkEstimates(landmark_estimates);

  if (GlobalParams::SaveFactorGraphsToFile())
  {
    SaveGraphToFile("/tmp/imu-graph.dot", *graph_, *values_);
  }

  graph_->resize(0);
  values_->clear();
}

void Smoother::RefineInitialNavstate(int new_frame_id, gtsam::NavState& navstate,
                                     const gtsam::CombinedImuFactor& imu_factor)
{
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;

  auto prior_noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(0.001), gtsam::Vector3::Constant(0.001)).finished());
  auto prior_noise_v = gtsam::noiseModel::Isotropic::Sigma(3, 0.001);
  auto prior_noise_b = gtsam::noiseModel::Isotropic::Sigma(6, 0.001);

  std::map<int, bool> existing_frame_ids_to_add;
  int i = 0;
  for (auto it = added_frame_timestamps_.rbegin(); it != added_frame_timestamps_.rend() && i < 4; ++it, ++i)
  {
    existing_frame_ids_to_add[it->first] = true;
    auto pose = isam2->calculateEstimate<gtsam::Pose3>(X(it->first));
    auto velocity = isam2->calculateEstimate<gtsam::Vector3>(V(it->first));
    auto bias = isam2->calculateEstimate<gtsam::imuBias::ConstantBias>(B(it->first));

    values.insert(X(it->first), pose);
    values.insert(V(it->first), velocity);
    values.insert(B(it->first), bias);

    graph.addPrior(X(it->first), pose, prior_noise_x);
    graph.addPrior(V(it->first), velocity, prior_noise_v);
    graph.addPrior(B(it->first), bias, prior_noise_b);
  }

  values.insert(X(new_frame_id), navstate.pose());
  values.insert(V(new_frame_id), navstate.velocity());
  values.insert(B(new_frame_id), values.at<gtsam::imuBias::ConstantBias>(B(existing_frame_ids_to_add.rbegin()->first)));

  graph.add(imu_factor);

  // We don't set the dynamic outlier rejection threshold here, as this might wrongly flag some factors as outliers
  gtsam::SmartProjectionParams smart_projection_params(gtsam::HESSIAN, gtsam::IGNORE_DEGENERACY, false, true, 1e-5);

  for (const auto& track : added_tracks_)
  {
    SmartFactor::shared_ptr smart_factor(new SmartFactor(feature_noise_, K_, *body_p_cam_, smart_projection_params));
    bool observed_in_new_frame = false;
    for (auto it = track.second->features.rbegin();
         it != track.second->features.rend() && (*it)->frame->id >= existing_frame_ids_to_add.cbegin()->first; ++it)
    {
      auto frame_id = (*it)->frame->id;
      if (existing_frame_ids_to_add.count(frame_id) || frame_id == new_frame_id)
      {
        smart_factor->add(gtsam::Point2((*it)->pt.x, (*it)->pt.y), X(frame_id));
      }
      if (frame_id == new_frame_id)
      {
        observed_in_new_frame = true;
      }
    }
    if (smart_factor->size() > 3 && observed_in_new_frame)
    {
      graph.add(smart_factor);
    }
  }

  gtsam::LevenbergMarquardtParams params;
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values);
  std::cout << "Navstate refinement: Error before LM step: " << optimizer.error() << std::endl;
  auto estimate = optimizer.optimize();
  std::cout << "Navstate refinement: Error after LM step: " << optimizer.error() << std::endl;
  navstate = gtsam::NavState(estimate.at<gtsam::Pose3>(X(new_frame_id)), estimate.at<gtsam::Vector3>(V(new_frame_id)));
}

void Smoother::HandleDegenerateLandmarks(int new_frame_id, gtsam::Values& values, gtsam::ISAM2Result& isam_result)
{
  std::vector<std::pair<int, boost::shared_ptr<SmartFactor>>> degenerate_smart_factors;
  std::set<int> factors_marked_for_removal;
  GetDegenerateLandmarks(degenerate_smart_factors);
  std::cout << "Have " << degenerate_smart_factors.size() << " degenerate smart factors" << std::endl;
  // First, try the optimization again with full solve
  if (!degenerate_smart_factors.empty())
  {
    gtsam::ISAM2UpdateParams params;
    params.forceFullSolve = true;
    auto result = isam2->update(gtsam::NonlinearFactorGraph(), gtsam::Values(), params);
    if (result.errorBefore && result.errorAfter)
    {
      std::cout << "error before after: " << *result.errorBefore << " -> " << *result.errorAfter << std::endl;
    }
    degenerate_smart_factors.clear();
    GetDegenerateLandmarks(degenerate_smart_factors);
    std::cout << "Have " << degenerate_smart_factors.size() << " degenerate smart factors after update" << std::endl;
  }

  if (!degenerate_smart_factors.empty())
  {
    for (const auto& degenerate_factor : degenerate_smart_factors)
    {
      if (degenerate_factor.second->size() <= GlobalParams::MinTrackLengthForSmoothing())
      {
        factors_marked_for_removal.insert(degenerate_factor.first);
        continue;
      }
      SmartFactor new_smart_factor(feature_noise_, K_, *body_p_cam_, *smart_factor_params_);
      for (const auto& feature : added_tracks_[degenerate_factor.first]->features)
      {
        // We recreate the factor with all features except the latest, in hopes that the it triggered the degeneracy
        if (feature->frame->id == new_frame_id)
        {
          feature->in_smoother = false;
          continue;
        }
        if (added_frame_timestamps_.count(feature->frame->id))
        {
          new_smart_factor.add(gtsam::Point2(feature->pt.x, feature->pt.y), X(feature->frame->id));
        }
      }
      new_smart_factor.triangulateForLinearize(new_smart_factor.cameras(values));
      smart_factors_[degenerate_factor.first].reset();
      smart_factors_[degenerate_factor.first] = gtsam::make_shared<SmartFactor>(new_smart_factor);
    }

    isam_result = isam2->update();
    values = isam2->calculateEstimate();

    std::vector<std::pair<int, boost::shared_ptr<SmartFactor>>> degenerate_smart_factors_post_update;
    GetDegenerateLandmarks(degenerate_smart_factors_post_update);
    std::cout << "Have " << degenerate_smart_factors_post_update.size()
              << " degenerate smart factors after removing newest observations" << std::endl;

    // factors_marked_for_removal already contain the too-short tracks at this point: also add the still-degenerate
    for (const auto& degenerate_factor : degenerate_smart_factors_post_update)
    {
      factors_marked_for_removal.insert(degenerate_factor.first);
    }

    for (const auto& factor : degenerate_smart_factors)
    {
      BlacklistTrack(factor.first);
    }

    for (const auto track_id : factors_marked_for_removal)
    {
      BlacklistTrack(track_id);
      RemoveTrack(track_id);
    }
  }
}

Pose3Stamped Smoother::AddKeyframe(const KeyframeTransform& keyframe_transform,
                                   const std::vector<shared_ptr<Track>>& tracks,
                                   std::vector<Pose3Stamped>& pose_estimates, std::map<int, Point3>& landmark_estimates)
{
  std::cout << "Performing update for frame " << keyframe_transform.frame1->id << " -> "
            << keyframe_transform.frame2->id << std::endl;

  auto prev_estimate = isam2->calculateEstimate();

  gtsam::FastMap<gtsam::FactorIndex, gtsam::KeySet> factor_new_affected_keys;
  std::map<gtsam::FactorIndex, int> new_factor_to_track_id;
  std::vector<std::vector<cv::Point2f>> initialized_landmarks;
  for (auto& track : tracks)
  {
    auto feat_it = track->features.rbegin();
    auto new_feature =
        *std::find_if(feat_it, track->features.rend(), [keyframe_transform](const std::shared_ptr<Feature>& f) -> bool {
          return f->frame->id == keyframe_transform.frame2->id;
        });

    if (smart_factors_.count(track->id) && !blacklisted_tracks_.count(track->id))
    {
      assert(new_feature->frame->id == keyframe_transform.frame2->id);
      if (GlobalParams::UseDogLeg())
      {
        /*
         * Workaround for using the dogleg solver:
         *
         * Completely remove the smart factor and re-add an identical one with the one added measurement.
         *
         * See issue https://github.com/borglab/gtsam/issues/301
         * and the one that recommended this fix:
         *   https://bitbucket.org/gtborg/gtsam/issues/367/isam2-with-smart-factors-null-ptr-segfault
         */
        auto smart_factor = *smart_factors_[track->id];
        smart_factor.add(gtsam::Point2(new_feature->pt.x, new_feature->pt.y), X(new_feature->frame->id));
        smart_factors_[track->id].reset();
        smart_factors_[track->id] = gtsam::make_shared<SmartFactor>(smart_factor);
        new_feature->in_smoother = true;
      }
      else
      {
        smart_factors_[track->id]->add(gtsam::Point2(new_feature->pt.x, new_feature->pt.y), X(new_feature->frame->id));
        new_feature->in_smoother = true;
      }
      auto factor_idx = track_id_to_factor_index_.at(track->id);
      factor_new_affected_keys[factor_idx].insert(X(new_feature->frame->id));
    }
    else
    {
      //  TODO: is the new here causing a memory leak? investigate. maybe make_shared instead?
      SmartFactor::shared_ptr smart_factor(new SmartFactor(feature_noise_, K_, *body_p_cam_, *smart_factor_params_));
      std::vector<cv::Point2f> added_feature_points = { new_feature->pt };
      std::vector<std::shared_ptr<Feature>> added_features = { new_feature };
      for (int i = static_cast<int>(track->features.size()) - 1; i >= 0; --i)
      {
        auto feature = track->features[i];
        if (added_frame_timestamps_.count(feature->frame->id))
        {
          smart_factor->add(gtsam::Point2(feature->pt.x, feature->pt.y), X(feature->frame->id));
          added_feature_points.push_back(feature->pt);
          added_features.push_back(feature);
        }
      }
      if (smart_factor->size() >= GlobalParams::MinTrackLengthForSmoothing() && smart_factor->size() <= 25)
      {
        auto triangulationResult = smart_factor->triangulateSafe(smart_factor->cameras(prev_estimate));
        if (triangulationResult.valid())
        {
          gtsam::Matrix E;
          auto reproj_error = smart_factor->totalReprojectionError(smart_factor->cameras(prev_estimate));
          auto avg_error = reproj_error / static_cast<double>(smart_factor->size());
          std::cout << "smart factor " << track->id << " with error " << reproj_error << " and size "
                    << smart_factor->size() << " (average " << avg_error << ")" << std::endl;
          smart_factor->triangulateAndComputeE(E, prev_estimate);
          auto P = smart_factor->PointCov(E);
          smart_factor->add(gtsam::Point2(new_feature->pt.x, new_feature->pt.y), X(new_feature->frame->id));
          std::cout << "initializing landmark " << track->id << " with " << smart_factor->size() << " observations"
                    << std::endl;
          smart_factors_[track->id] = smart_factor;
          added_tracks_[track->id] = track;
          graph_->add(smart_factor);
          initialized_landmarks.push_back(added_feature_points);
          for (auto& feature : added_features)
          {
            feature->in_smoother = true;
          }
          new_factor_to_track_id[graph_->size()] = track->id;
        }
      }
    }
  }

  DebugImagePublisher::PublishNewLandmarksImage(keyframe_transform.frame2->image, initialized_landmarks,
                                                keyframe_transform.frame2->timestamp);

  std::cout << "Have " << smart_factors_.size() << " smart factors" << std::endl;

  WaitForAndIntegrateIMU(added_frame_timestamps_[last_frame_id_added_], keyframe_transform.frame2->timestamp);

  auto imu_combined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imu_measurements_);
  gtsam::CombinedImuFactor imu_factor(X(last_frame_id_added_), V(last_frame_id_added_),
                                      X(keyframe_transform.frame2->id), V(keyframe_transform.frame2->id),
                                      B(last_frame_id_added_), B(keyframe_transform.frame2->id), imu_combined);
  graph_->add(imu_factor);

  // TODO all these can also be extracted
  auto prev_pose = prev_estimate.at<gtsam::Pose3>(X(last_frame_id_added_));
  auto prev_velocity = prev_estimate.at<gtsam::Vector3>(V(last_frame_id_added_));
  auto prev_bias = prev_estimate.at<gtsam::imuBias::ConstantBias>(B(last_frame_id_added_));
  // TODO end

  auto predicted_navstate = imu_measurements_->predict(gtsam::NavState(prev_pose, prev_velocity), prev_bias);

  predicted_navstate.print("navstate before: ");
  RefineInitialNavstate(keyframe_transform.frame2->id, predicted_navstate, imu_factor);
  predicted_navstate.print("navstate after: ");

  values_->insert(X(keyframe_transform.frame2->id), predicted_navstate.pose());
  values_->insert(V(keyframe_transform.frame2->id), predicted_navstate.velocity());
  values_->insert(B(keyframe_transform.frame2->id), prev_bias);

  std::cout << "Performing optimization" << std::endl;
  gtsam::Values new_estimate;
  // Set newAffectedKeys to notify isam about which new keys existing smart factors are now affecting
  gtsam::ISAM2UpdateParams update_params;
  update_params.newAffectedKeys = std::move(factor_new_affected_keys);

  auto isam_result = isam2->update(*graph_, *values_, update_params);
  isam_result.print("isam result: ");
  new_estimate = isam2->calculateEstimate();
  if (isam_result.errorBefore && isam_result.errorAfter)
  {
    std::cout << "error before after: " << *isam_result.errorBefore << " -> " << *isam_result.errorAfter << std::endl;
    /*
    if (isam_result.errorAfter > isam_result.errorBefore)
    {
      std::cout << "WARN: Error increased during ISAM update. Will re-do update with forceFullSolve on " << std::endl;
      gtsam::ISAM2UpdateParams new_update_params;
      new_update_params.forceFullSolve = true;
      isam2->update(gtsam::NonlinearFactorGraph(), gtsam::Values(), new_update_params);
    }
     */
    DebugValuePublisher::PublishNonlinearError(*isam_result.errorAfter);
  }
  DebugValuePublisher::PublishRelinearizedCliques(static_cast<int>(isam_result.variablesRelinearized));
  DebugValuePublisher::PublishReeliminatedCliques(static_cast<int>(isam_result.variablesReeliminated));
  DebugValuePublisher::PublishTotalCliques(static_cast<int>(isam_result.cliques));

  gtsam::ISAM2Result new_isam_result;
  HandleDegenerateLandmarks(keyframe_transform.frame2->id, new_estimate, new_isam_result);
  if (new_isam_result.errorBefore && new_isam_result.errorAfter)
  {
    std::cout << "error before after: " << *new_isam_result.errorBefore << " -> " << *new_isam_result.errorAfter
              << std::endl;
  }

  for (const auto& factor_track_pair : new_factor_to_track_id)
  {
    track_id_to_factor_index_[factor_track_pair.second] = isam_result.newFactorsIndices.at(factor_track_pair.first);
  }
  std::cout << "Optimization done" << std::endl;

  // PublishReprojectionErrorImages();
  PublishNewReprojectionErrorImage(new_estimate, keyframe_transform.frame2);

  GetPoseEstimates(pose_estimates);
  GetLandmarkEstimates(landmark_estimates);

  auto new_bias = new_estimate.at<gtsam::imuBias::ConstantBias>(B(keyframe_transform.frame2->id));

  std::vector<double> bias_acc = { new_bias.accelerometer().x(), new_bias.accelerometer().y(),
                                   new_bias.accelerometer().z() };
  std::vector<double> bias_gyro = { new_bias.gyroscope().x(), new_bias.gyroscope().y(), new_bias.gyroscope().z() };
  DebugValuePublisher::PublishBias(bias_acc, bias_gyro);

  std::vector<double> v_norms;
  for (auto frame : added_frame_timestamps_)
  {
    auto velocity = new_estimate.at<gtsam::Vector3>(V(frame.first));
    v_norms.push_back(velocity.norm());
  }
  auto v_average = std::accumulate(v_norms.begin(), v_norms.end(), 0.0) / static_cast<double>(v_norms.size());
  DebugValuePublisher::PublishVelocityNormAverage(v_average);
  DebugValuePublisher::PublishFrameId(keyframe_transform.frame2->id);

  imu_measurements_->resetIntegrationAndSetBias(new_bias);

  last_frame_id_added_ = keyframe_transform.frame2->id;
  added_frame_timestamps_[keyframe_transform.frame2->id] = keyframe_transform.frame2->timestamp;

  auto new_pose = new_estimate.at<gtsam::Pose3>(X(keyframe_transform.frame2->id));

  if (GlobalParams::SaveFactorGraphsToFile())
  {
    SaveGraphToFile("/tmp/update-graph.dot", *graph_, *values_);
  }

  /*
  for (const auto& factor : isam2->getFactorsUnsafe())
  {
    factor->printKeys();
    std::cout << factor->error(new_estimate) << std::endl;
  }
   */

  graph_->resize(0);
  values_->clear();

  return Pose3Stamped{ .pose = ToPose(new_pose), .stamp = keyframe_transform.frame2->timestamp };
}

Pose3Stamped Smoother::AddFrame(const std::shared_ptr<Frame>& frame, const std::vector<std::shared_ptr<Track>>& tracks,
                                std::vector<Pose3Stamped>& pose_estimates, std::map<int, Point3>& landmark_estimates)
{
  std::cout << "Performing non-kf update for frame " << frame->id << std::endl;

  auto prev_estimate = isam2->calculateEstimate();

  gtsam::FastMap<gtsam::FactorIndex, gtsam::KeySet> factor_new_affected_keys;
  std::vector<std::vector<cv::Point2f>> initialized_landmarks;
  for (auto& track : tracks)
  {
    auto new_feature = track->features.back();

    if (smart_factors_.count(track->id) && !blacklisted_tracks_.count(track->id))
    {
      assert(new_feature->frame->id == frame->id);
      if (GlobalParams::UseDogLeg())
      {
        /*
         * Workaround for using the dogleg solver:
         *
         * Completely remove the smart factor and re-add an identical one with the one added measurement.
         *
         * See issue https://github.com/borglab/gtsam/issues/301
         * and the one that recommended this fix:
         *   https://bitbucket.org/gtborg/gtsam/issues/367/isam2-with-smart-factors-null-ptr-segfault
         */
        auto smart_factor = *smart_factors_[track->id];
        smart_factor.add(gtsam::Point2(new_feature->pt.x, new_feature->pt.y), X(new_feature->frame->id));
        smart_factors_[track->id].reset();
        smart_factors_[track->id] = gtsam::make_shared<SmartFactor>(smart_factor);
        new_feature->in_smoother = true;
      }
      else
      {
        smart_factors_[track->id]->add(gtsam::Point2(new_feature->pt.x, new_feature->pt.y), X(new_feature->frame->id));
        new_feature->in_smoother = true;
      }
      auto factor_idx = track_id_to_factor_index_.at(track->id);
      factor_new_affected_keys[factor_idx].insert(X(new_feature->frame->id));
    }
  }

  WaitForAndIntegrateIMU(added_frame_timestamps_[last_frame_id_added_], frame->timestamp);

  auto imu_combined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imu_measurements_);
  gtsam::CombinedImuFactor imu_factor(X(last_frame_id_added_), V(last_frame_id_added_), X(frame->id), V(frame->id),
                                      B(last_frame_id_added_), B(frame->id), imu_combined);
  graph_->add(imu_factor);

  // TODO all these can also be extracted
  auto prev_pose = prev_estimate.at<gtsam::Pose3>(X(last_frame_id_added_));
  auto prev_velocity = prev_estimate.at<gtsam::Vector3>(V(last_frame_id_added_));
  auto prev_bias = prev_estimate.at<gtsam::imuBias::ConstantBias>(B(last_frame_id_added_));
  // TODO end

  auto predicted_navstate = imu_measurements_->predict(gtsam::NavState(prev_pose, prev_velocity), prev_bias);
  RefineInitialNavstate(frame->id, predicted_navstate, imu_factor);

  values_->insert(X(frame->id), predicted_navstate.pose());
  values_->insert(V(frame->id), predicted_navstate.velocity());
  values_->insert(B(frame->id), prev_bias);

  std::cout << "Performing optimization" << std::endl;
  // Set newAffectedKeys to notify isam about which new keys existing smart factors are now affecting
  gtsam::ISAM2UpdateParams update_params;
  update_params.newAffectedKeys = std::move(factor_new_affected_keys);

  auto isam_result = isam2->update(*graph_, *values_, update_params);
  auto new_estimate = isam2->calculateEstimate();
  isam_result.print("isam result: ");
  if (isam_result.errorBefore && isam_result.errorAfter)
  {
    std::cout << "error before after: " << *isam_result.errorBefore << " -> " << *isam_result.errorAfter << std::endl;
    DebugValuePublisher::PublishNonlinearError(*isam_result.errorAfter);
    if (isam_result.errorAfter > isam_result.errorBefore)
    {
      std::cout << "WARN: Error increased during ISAM update. Will re-do update with forceFullSolve on " << std::endl;
      gtsam::ISAM2UpdateParams new_update_params;
      new_update_params.forceFullSolve = true;
      isam2->update(gtsam::NonlinearFactorGraph(), gtsam::Values(), new_update_params);
    }
  }
  DebugValuePublisher::PublishRelinearizedCliques(static_cast<int>(isam_result.variablesRelinearized));
  DebugValuePublisher::PublishReeliminatedCliques(static_cast<int>(isam_result.variablesReeliminated));
  DebugValuePublisher::PublishTotalCliques(static_cast<int>(isam_result.cliques));

  gtsam::ISAM2Result new_isam_result;
  HandleDegenerateLandmarks(frame->id, new_estimate, new_isam_result);
  if (new_isam_result.errorBefore && new_isam_result.errorAfter)
  {
    std::cout << "error before after: " << *new_isam_result.errorBefore << " -> " << *new_isam_result.errorAfter
              << std::endl;
  }
  std::cout << "Optimization done" << std::endl;

  last_frame_id_added_ = frame->id;
  added_frame_timestamps_[frame->id] = frame->timestamp;

  GetPoseEstimates(pose_estimates);
  GetLandmarkEstimates(landmark_estimates);

  PublishNewReprojectionErrorImage(new_estimate, frame);

  auto new_bias = new_estimate.at<gtsam::imuBias::ConstantBias>(B(frame->id));
  auto new_pose = new_estimate.at<gtsam::Pose3>(X(frame->id));

  imu_measurements_->resetIntegrationAndSetBias(new_bias);

  graph_->resize(0);
  values_->clear();

  return Pose3Stamped{ .pose = ToPose(new_pose), .stamp = frame->timestamp };
}

void Smoother::RemoveTrack(int track_id)
{
  for (auto& feature : added_tracks_[track_id]->features)
  {
    feature->in_smoother = false;
  }
  added_tracks_.erase(track_id);
  smart_factors_[track_id].reset();
  smart_factors_.erase(track_id);
  track_id_to_factor_index_.erase(track_id);
}

void Smoother::BlacklistTrack(int track_id)
{
  if (!blacklisted_tracks_.count(track_id))
  {
    blacklisted_tracks_[track_id] = added_tracks_[track_id];
  }
}

void Smoother::WaitForAndIntegrateIMU(double timestamp1, double timestamp2)
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

int Smoother::GetLastFrameId() const
{
  return last_frame_id_added_;
}

BackendStatus Smoother::GetStatus()
{
  if (initialized_)
  {
    if (added_frame_timestamps_.size() > GlobalParams::MinKeyframesForNominal())
    {
      return kNominal;
    }
    else
    {
      return kLandmarksInitialized;
    }
  }
  else
  {
    return kUninitialized;
  }
}
