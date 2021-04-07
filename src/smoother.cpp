#include "smoother.h"
#include "key_point_observation.h"
#include "gtsam_conversions.h"
#include "pose3_stamped.h"
#include "global_params.h"
#include "newer_college_ground_truth.h"
#include "gtsam_helpers.h"
#include "debug_image_publisher.h"

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
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <thread>
#include <chrono>
#include <gtsam/sam/RangeFactor.h>

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

gtsam::ISAM2Params MakeIsam2Params()
{
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = GlobalParams::IsamRelinearizeThresh();
  params.relinearizeSkip = 1;
  params.evaluateNonlinearError = true;
  params.optimizationParams = gtsam::ISAM2GaussNewtonParams();
  return params;
}

shared_ptr<gtsam::PreintegrationType> MakeIMUIntegrator()
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

  gtsam::Pose3 body_p_imu = gtsam::Pose3(
      gtsam::Rot3::Quaternion(GlobalParams::BodyPImuQuat()[3], GlobalParams::BodyPImuQuat()[0],
                              GlobalParams::BodyPImuQuat()[1], GlobalParams::BodyPImuQuat()[2]),
      gtsam::Point3(GlobalParams::BodyPImuVec()[0], GlobalParams::BodyPImuVec()[1], GlobalParams::BodyPImuVec()[2]));
  imu_params->body_P_sensor = body_p_imu;

  auto imu_bias = gtsam::imuBias::ConstantBias();  // Initialize at zero bias

  auto imu_measurements = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imu_params, imu_bias);
  imu_measurements->resetIntegration();
  return imu_measurements;
}

gtsam::SmartProjectionParams GetSmartProjectionParams()
{
  gtsam::SmartProjectionParams smart_projection_params(gtsam::HESSIAN, gtsam::IGNORE_DEGENERACY, false, true, 1e-5);
  // smart_projection_params.setDynamicOutlierRejectionThreshold(10.);
  // smart_projection_params.setLandmarkDistanceThreshold(30.);
  return smart_projection_params;
}

Smoother::Smoother(std::shared_ptr<IMUQueue> imu_queue)
  : isam2(new gtsam::ISAM2(MakeIsam2Params()))
  , graph_(new gtsam::NonlinearFactorGraph())
  , values_(new gtsam::Values())
  , imu_queue_(std::move(imu_queue))
  , imu_measurements_(MakeIMUIntegrator())
{
}

void Smoother::InitializeLandmarks(
    std::vector<KeyframeTransform> keyframe_transforms, const std::vector<shared_ptr<Track>>& tracks,
    const boost::optional<std::pair<std::shared_ptr<Frame>, std::shared_ptr<Frame>>>& frames_for_imu_init,
    std::vector<Pose3Stamped>& pose_estimates, std::map<int, Point3>& landmark_estimates)
{
  std::cout << "Let's initialize those landmarks" << std::endl;

  auto gt_pose = ToGtsamPose(NewerCollegeGroundTruth::At(keyframe_transforms[0].frame1->timestamp));
  // gtsam::Pose3 init_pose(gtsam::Rot3(), gtsam::Point3::Zero());
  auto init_pose = frames_for_imu_init ?
                       gtsam::Pose3(ToGtsamRot(imu_queue_->RefineInitialAttitude(frames_for_imu_init->first->timestamp,
                                                                                 frames_for_imu_init->second->timestamp,
                                                                                 ToRot(gt_pose.rotation()))),
                                    gt_pose.translation()) :
                       gt_pose;

  auto noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(0.0001), gtsam::Vector3::Constant(0.0001)).finished());

  graph_->addPrior(X(keyframe_transforms[0].frame1->id), init_pose, noise_x);

  values_->insert(X(keyframe_transforms[0].frame1->id), init_pose);

  added_frame_timestamps_[keyframe_transforms[0].frame1->id] = keyframe_transforms[0].frame1->timestamp;

  std::map<int, bool> frame_ids;
  frame_ids[keyframe_transforms[0].frame1->id] = true;

  gtsam::Pose3 body_p_cam = gtsam::Pose3(
      gtsam::Rot3::Quaternion(GlobalParams::BodyPCamQuat()[3], GlobalParams::BodyPCamQuat()[0],
                              GlobalParams::BodyPCamQuat()[1], GlobalParams::BodyPCamQuat()[2]),
      gtsam::Point3(GlobalParams::BodyPCamVec()[0], GlobalParams::BodyPCamVec()[1], GlobalParams::BodyPCamVec()[2]));

  // Initialize values on R and t from essential matrix
  std::vector<gtsam::Pose3> poses = { init_pose };
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

      gtsam::Unit3 t_i(body_p_cam * t_unit_cam_frame.point3());
      gtsam::Rot3 R_i(body_p_cam.rotation() * R_cam_frame * body_p_cam.rotation().inverse());
      gtsam::Pose3 X_i = gtsam::Pose3(R_i, t_i.point3());

      gtsam::Pose3 X_world = poses.back().compose(X_i);
      poses.push_back(X_world);

      std::cout << "R_c = " << R_cam_frame.ypr() << std::endl;
      t_unit_cam_frame.print("t_c = ");
      std::cout << "R_i = " << R_i.ypr() << std::endl;
      t_i.print("t_i = ");
      std::cout << "R_world = " << X_world.rotation().ypr() << std::endl;
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

  // The SfM-only problem has scale ambiguity, so we add a range factor to define scale
  auto pose_delta_range = poses[0].inverse().compose(poses.back()).translation().norm();
  auto range_noise = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
  range_factor_ = gtsam::make_shared<gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3>>(
      X(keyframe_transforms[0].frame1->id), X(keyframe_transforms.back().frame2->id), pose_delta_range, range_noise);
  graph_->add(range_factor_);

  gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(GlobalParams::CamFx(), GlobalParams::CamFy(), 0.0,
                                                  GlobalParams::CamU0(), GlobalParams::CamV0()));

  auto feature_noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  for (auto& track : tracks)
  {
    SmartFactor::shared_ptr smart_factor(new SmartFactor(feature_noise, K, body_p_cam, GetSmartProjectionParams()));
    for (auto& feature : track->features)
    {
      if (frame_ids.count(feature->frame->id))
      {
        auto pt = feature->pt;
        smart_factor->add(gtsam::Point2(pt.x, pt.y), X(feature->frame->id));
      }
    }
    if (smart_factor->size() >= GlobalParams::MinTrackLengthForSmoothing())
    {
      std::cout << "adding landmark " << track->id << " with " << smart_factor->size() << " observations and parallax "
                << track->max_parallax << " and inlier ratio " << track->InlierRatio() << std::endl;
      smart_factors_[track->id] = smart_factor;
      graph_->add(smart_factor);
    }
  }
  std::cout << "Added " << smart_factors_.size() << " smart factors" << std::endl;

  if (GlobalParams::UseIsam())
  {
    auto isam_result = isam2->update(*graph_, *values_);
    isam_result.print("isam result: ");
    if (isam_result.errorBefore && isam_result.errorAfter)
    {
      std::cout << "error before after: " << *isam_result.errorBefore << " -> " << *isam_result.errorAfter << std::endl;
    }
  }
  else
  {
    gtsam::GaussNewtonOptimizer optimizer(*graph_, *values_);
    *values_ = optimizer.optimize();
  }

  // If an initial range factor length is provided, we scale the solution to account for it and redo the optimization
  if (GlobalParams::InitRangeFactorLength() > 0)
  {
    double scale_ratio = GlobalParams::InitRangeFactorLength() / pose_delta_range;
    auto frame_it = added_frame_timestamps_.begin();
    std::vector<std::pair<std::pair<int, int>, gtsam::Pose3>> scaled_between_poses;
    std::vector<std::pair<int, gtsam::Pose3>> scaled_poses{
      { frame_it->first, GlobalParams::UseIsam() ? isam2->calculateEstimate<gtsam::Pose3>(X(frame_it->first)) :
                                                   values_->at<gtsam::Pose3>(X(frame_it->first)) }
    };
    while (true)
    {
      auto pose1_id = frame_it->first;
      auto pose1 = GlobalParams::UseIsam() ? isam2->calculateEstimate<gtsam::Pose3>(X(frame_it->first)) :
                                             values_->at<gtsam::Pose3>(X(frame_it->first));
      ++frame_it;
      if (frame_it == added_frame_timestamps_.end())
      {
        break;
      }
      auto pose2 = GlobalParams::UseIsam() ? isam2->calculateEstimate<gtsam::Pose3>(X(frame_it->first)) :
                                             values_->at<gtsam::Pose3>(X(frame_it->first));
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

    // Range factor is already added as a shared pointer in the graph, we update the pointer value here.
    *range_factor_ = gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3>(X(keyframe_transforms[0].frame1->id),
                                                                    X(keyframe_transforms.back().frame2->id),
                                                                    GlobalParams::InitRangeFactorLength(), range_noise);

    if (GlobalParams::UseIsam())
    {
      *isam2 = gtsam::ISAM2(MakeIsam2Params());  // Reinitialize isam
      auto isam_result = isam2->update(*graph_, *values_);
      isam_result.print("isam result: ");
      if (isam_result.errorBefore && isam_result.errorAfter)
      {
        std::cout << "error before after: " << *isam_result.errorBefore << " -> " << *isam_result.errorAfter
                  << std::endl;
      }
    }
    else
    {
      gtsam::GaussNewtonOptimizer optimizer(*graph_, *values_);
      *values_ = optimizer.optimize();
    }
  }

  GetPoseEstimates(pose_estimates);
  GetLandmarkEstimates(landmark_estimates);

  last_frame_id_added_ = keyframe_transforms.back().frame2->id;

  if (GlobalParams::SaveFactorGraphsToFile())
  {
    SaveGraphToFile("/tmp/landmarks-graph.dot", *graph_, *values_);
  }

  if (GlobalParams::UseIsam())
  {
    graph_->resize(0);
    values_->clear();
  }
  status_ = kLandmarksInitialized;
}

void Smoother::GetPoseEstimates(std::vector<Pose3Stamped>& pose_estimates)
{
  for (auto& frame_pair : added_frame_timestamps_)
  {
    auto gtsam_pose = GlobalParams::UseIsam() ? isam2->calculateEstimate<gtsam::Pose3>(X(frame_pair.first)) :
                                                values_->at<gtsam::Pose3>(X(frame_pair.first));
    pose_estimates.push_back(Pose3Stamped{ .pose = ToPose(gtsam_pose), .stamp = frame_pair.second });
  }
}

void Smoother::GetLandmarkEstimates(std::map<int, Point3>& landmark_estimates)
{
  auto values = GlobalParams::UseIsam() ? isam2->calculateEstimate() : *values_;
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
        else
        {
          std::cout << "Depth error large for landmark " << smart_factor_pair.first << ": " << P(0, 0) << " (norm "
                    << P.norm() << ")" << std::endl;
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
  // Init on "random values" as this apparently helps convergence
  gtsam::Vector3 zero_velocity(0.000001, 0.000002, -0.000001);
  gtsam::imuBias::ConstantBias init_bias(gtsam::Vector3(0.0001, 0.0002, -0.0001),
                                         gtsam::Vector3(-0.0001, 0.0001, 0.0001));
  auto noise_v = gtsam::noiseModel::Isotropic::Sigma(3, 1);
  auto noise_b = gtsam::noiseModel::Isotropic::Sigma(6, 0.02);

  graph_->addPrior(V(keyframe_transforms[0].frame1->id), zero_velocity, noise_v);
  graph_->addPrior(B(keyframe_transforms[0].frame1->id), init_bias, noise_b);

  values_->insert(V(keyframe_transforms[0].frame1->id), zero_velocity);
  values_->insert(B(keyframe_transforms[0].frame1->id), init_bias);

  for (auto& keyframe_transform : keyframe_transforms)
  {
    WaitForAndIntegrateIMU(keyframe_transform.frame1->timestamp, keyframe_transform.frame2->timestamp);

    auto imu_combined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imu_measurements_);
    gtsam::CombinedImuFactor imu_factor(X(keyframe_transform.frame1->id), V(keyframe_transform.frame1->id),
                                        X(keyframe_transform.frame2->id), V(keyframe_transform.frame2->id),
                                        B(keyframe_transform.frame1->id), B(keyframe_transform.frame2->id),
                                        imu_combined);
    graph_->add(imu_factor);
    imu_measurements_->resetIntegration();

    values_->insert(V(keyframe_transform.frame2->id), zero_velocity);
    values_->insert(B(keyframe_transform.frame2->id), init_bias);
  }

  // The IMU factors will now be defining the scale, so the range factor should be removed.
  // range_factor_ = nullptr;

  if (GlobalParams::UseIsam())
  {
    auto isam_result = isam2->update(*graph_, *values_);
    isam_result.print("isam result: ");
    if (isam_result.errorBefore && isam_result.errorAfter)
    {
      std::cout << "error before after: " << *isam_result.errorBefore << " -> " << *isam_result.errorAfter << std::endl;
    }
  }
  else
  {
    gtsam::GaussNewtonOptimizer optimizer(*graph_, *values_);
    *values_ = optimizer.optimize();
  }

  GetPoseEstimates(pose_estimates);
  GetLandmarkEstimates(landmark_estimates);

  if (GlobalParams::SaveFactorGraphsToFile())
  {
    SaveGraphToFile("/tmp/imu-graph.dot", *graph_, *values_);
  }

  if (GlobalParams::UseIsam())
  {
    graph_->resize(0);
    values_->clear();
  }
}

void Smoother::Reoptimize(std::vector<Pose3Stamped>& pose_estimates, std::map<int, Point3>& landmark_estimates)
{
  if (GlobalParams::UseIsam())
  {
    gtsam::GaussNewtonOptimizer optimizer(*graph_, *values_);
    *values_ = optimizer.optimize();
  }
  else
  {
    auto isam_result = isam2->update();
    isam_result.print("isam result: ");
    if (isam_result.errorBefore && isam_result.errorAfter)
    {
      std::cout << "error before after: " << *isam_result.errorBefore << " -> " << *isam_result.errorAfter << std::endl;
    }
  }
  GetPoseEstimates(pose_estimates);
  GetLandmarkEstimates(landmark_estimates);
}

Pose3Stamped Smoother::Update(const KeyframeTransform& keyframe_transform, const std::vector<shared_ptr<Track>>& tracks,
                              std::vector<Pose3Stamped>& pose_estimates, std::map<int, Point3>& landmark_estimates)
{
  std::cout << "Performing update for frame " << keyframe_transform.frame1->id << " -> "
            << keyframe_transform.frame2->id << std::endl;

  gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(GlobalParams::CamFx(), GlobalParams::CamFy(), 0.0,
                                                  GlobalParams::CamU0(), GlobalParams::CamV0()));

  auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  auto prev_estimate = GlobalParams::UseIsam() ? isam2->calculateEstimate() : *values_;

  gtsam::Pose3 body_p_cam = gtsam::Pose3(
      gtsam::Rot3::Quaternion(GlobalParams::BodyPCamQuat()[3], GlobalParams::BodyPCamQuat()[0],
                              GlobalParams::BodyPCamQuat()[1], GlobalParams::BodyPCamQuat()[2]),
      gtsam::Point3(GlobalParams::BodyPCamVec()[0], GlobalParams::BodyPCamVec()[1], GlobalParams::BodyPCamVec()[2]));

  std::vector<std::vector<cv::Point2f>> initialized_landmarks;
  for (auto& track : tracks)
  {
    auto feat_it = track->features.rbegin();
    const auto new_feature =
        std::find_if(feat_it, track->features.rend(), [keyframe_transform](const std::shared_ptr<Feature>& f) -> bool {
          return f->frame->id == keyframe_transform.frame2->id;
        })->get();

    if (smart_factors_.count(track->id))
    {
      assert(new_feature->frame->id == keyframe_transform.frame2->id);
      smart_factors_[track->id]->add(gtsam::Point2(new_feature->pt.x, new_feature->pt.y), X(new_feature->frame->id));
    }
    else
    {
      SmartFactor::shared_ptr smart_factor(
          new SmartFactor(measurementNoise, K, body_p_cam, GetSmartProjectionParams()));
      std::vector<cv::Point2f> added_features;
      for (int i = static_cast<int>(track->features.size()) - 1; i > 0 && added_features.size() < 15; --i)
      {
        auto feature = track->features[i];
        if (added_frame_timestamps_.count(feature->frame->id))
        {
          smart_factor->add(gtsam::Point2(feature->pt.x, feature->pt.y), X(feature->frame->id));
          added_features.push_back(feature->pt);
        }
      }
      if (smart_factor->size() >= GlobalParams::MinTrackLengthForSmoothing())
      {
        auto triangulationResult = smart_factor->triangulateSafe(smart_factor->cameras(prev_estimate));
        if (triangulationResult.valid())
        {
          gtsam::Matrix E;
          smart_factor->triangulateAndComputeE(E, prev_estimate);
          auto P = smart_factor->PointCov(E);
          if (P.norm() < 3)
          {
            smart_factor->add(gtsam::Point2(new_feature->pt.x, new_feature->pt.y), X(new_feature->frame->id));
            std::cout << "initializing landmark " << track->id << " with " << smart_factor->size() << " observations"
                      << std::endl;
            smart_factors_[track->id] = smart_factor;
            graph_->add(smart_factor);
            initialized_landmarks.push_back(added_features);
          }
          else
          {
            std::cout << "Skipping landmark " << track->id << " with P norm " << P.norm() << std::endl;
          }
        }
      }
    }
  }

  DebugImagePublisher::PublishNewLandmarksImage(keyframe_transform.frame2->image, initialized_landmarks,
                                                keyframe_transform.frame2->timestamp);

  std::cout << "Have " << smart_factors_.size() << " smart factors" << std::endl;

  WaitForAndIntegrateIMU(keyframe_transform.frame1->timestamp, keyframe_transform.frame2->timestamp);

  // TODO all these can also be extracted
  auto prev_pose = prev_estimate.at<gtsam::Pose3>(X(last_frame_id_added_));
  auto prev_velocity = prev_estimate.at<gtsam::Vector3>(V(last_frame_id_added_));
  auto prev_bias = prev_estimate.at<gtsam::imuBias::ConstantBias>(B(last_frame_id_added_));
  // TODO end

  auto predicted_navstate = imu_measurements_->predict(gtsam::NavState(prev_pose, prev_velocity), prev_bias);

  values_->insert(X(keyframe_transform.frame2->id), predicted_navstate.pose());
  values_->insert(V(keyframe_transform.frame2->id), predicted_navstate.velocity());
  values_->insert(B(keyframe_transform.frame2->id), prev_bias);

  auto imu_delta = imu_measurements_->deltaXij();

  auto R_mat = ToMatrix3(*keyframe_transform.GetRotation());
  auto t = *keyframe_transform.GetTranslation();
  gtsam::Unit3 t_unit_cam_frame(t[0], t[1], t[2]);
  gtsam::Rot3 R_cam_frame(R_mat);

  gtsam::Unit3 t_i(body_p_cam * t_unit_cam_frame.point3());
  gtsam::Rot3 R_i(body_p_cam.rotation() * R_cam_frame * body_p_cam.rotation().inverse());
  gtsam::Pose3 X_i = gtsam::Pose3(R_i, t_i.point3());

  auto imu_delta_t_length = imu_delta.pose().translation().norm();

  std::cout << "----- Rotation: ----" << std::endl;
  std::cout << "Prev: " << prev_pose.rotation().ypr().transpose() << std::endl;
  std::cout << "IMU pred: " << predicted_navstate.pose().rotation().ypr().transpose() << std::endl;
  std::cout << "IMU delta: " << imu_delta.pose().rotation().ypr().transpose() << std::endl;
  std::cout << "SfM delta: " << X_i.rotation().ypr().transpose() << std::endl;
  std::cout << "--------------------" << std::endl;

  std::cout << "--- Translation: ---" << std::endl;
  std::cout << "Prev: " << prev_pose.translation().transpose() << std::endl;
  std::cout << "IMU pred: " << predicted_navstate.pose().translation().transpose() << std::endl;
  std::cout << "IMU delta: " << imu_delta.pose().translation().transpose() << std::endl;
  std::cout << "SfM delta: " << (imu_delta_t_length * X_i.translation()).transpose() << std::endl;
  std::cout << "--------------------" << std::endl;

  std::cout << "----- Velocity: ----" << std::endl;
  std::cout << "prev -> IMU delta v: " << prev_velocity.transpose() << " -> " << imu_delta.velocity().transpose()
            << std::endl;
  std::cout << "--------------------" << std::endl;

  auto imu_combined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imu_measurements_);
  gtsam::CombinedImuFactor imu_factor(X(keyframe_transform.frame1->id), V(keyframe_transform.frame1->id),
                                      X(keyframe_transform.frame2->id), V(keyframe_transform.frame2->id),
                                      B(keyframe_transform.frame1->id), B(keyframe_transform.frame2->id), imu_combined);
  graph_->add(imu_factor);

  std::cout << "Performing optimization" << std::endl;
  try
  {
    if (GlobalParams::UseIsam())
    {
      auto isam_result = isam2->update(*graph_, *values_);
      isam_result.print("isam result: ");
      if (isam_result.errorBefore && isam_result.errorAfter)
      {
        std::cout << "error before after: " << *isam_result.errorBefore << " -> " << *isam_result.errorAfter
                  << std::endl;
      }
    }
    else
    {
      gtsam::GaussNewtonOptimizer optimizer(*graph_, *values_);
      *values_ = optimizer.optimize();
    }
  }
  catch (exception& e)
  {
    std::cout << "oopsie woopsie" << std::endl;
    throw;
  }
  std::cout << "Optimization done" << std::endl;

  GetPoseEstimates(pose_estimates);
  GetLandmarkEstimates(landmark_estimates);

  auto new_bias = GlobalParams::UseIsam() ?
                      isam2->calculateEstimate<gtsam::imuBias::ConstantBias>(B(keyframe_transform.frame2->id)) :
                      values_->at<gtsam::imuBias::ConstantBias>(B(keyframe_transform.frame2->id));

  imu_measurements_->resetIntegrationAndSetBias(new_bias);

  last_frame_id_added_ = keyframe_transform.frame2->id;
  added_frame_timestamps_[keyframe_transform.frame2->id] = keyframe_transform.frame2->timestamp;

  auto new_pose = GlobalParams::UseIsam() ? isam2->calculateEstimate<gtsam::Pose3>(X(keyframe_transform.frame2->id)) :
                                            values_->at<gtsam::Pose3>(X(keyframe_transform.frame2->id));

  if (GlobalParams::SaveFactorGraphsToFile())
  {
    SaveGraphToFile("/tmp/update-graph.dot", *graph_, *values_);
  }

  if (GlobalParams::UseIsam())
  {
    graph_->resize(0);
    values_->clear();
  }

  return Pose3Stamped{ .pose = ToPose(new_pose), .stamp = keyframe_transform.frame2->timestamp };
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
  return status_;
}
