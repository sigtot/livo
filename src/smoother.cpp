#include "smoother.h"
#include "key_point_observation.h"
#include "frame.h"
#include "gtsam_conversions.h"
#include "pose3_stamped.h"
#include "global_params.h"
#include "newer_college_ground_truth.h"

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
#include <gtsam/geometry/triangulation.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/EssentialMatrixConstraint.h>
#include <gtsam/nonlinear/Marginals.h>
#include <thread>
#include <chrono>

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

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

  auto imu_bias = gtsam::imuBias::ConstantBias();  // Initialize at zero bias

  auto imu_measurements = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imu_params, imu_bias);
  imu_measurements->resetIntegration();
  return imu_measurements;
}

gtsam::SmartProjectionParams GetSmartProjectionParams()
{
  gtsam::SmartProjectionParams smart_projection_params(gtsam::HESSIAN, gtsam::IGNORE_DEGENERACY, false, true, 1e-5);
  // smart_projection_params.setDynamicOutlierRejectionThreshold(10.);
  smart_projection_params.setLandmarkDistanceThreshold(30.);
  return smart_projection_params;
}

void Smoother::InitializeLandmarks(std::vector<KeyframeTransform> keyframe_transforms,
                                   const std::vector<shared_ptr<Track>>& tracks,
                                   std::vector<Pose3Stamped>& pose_estimates, std::map<int, Point3>& landmark_estimates)
{
  std::cout << "Let's initialize those landmarks!" << std::endl;

  gtsam::Pose3 init_pose(gtsam::Rot3(), gtsam::Point3::Zero());
  gtsam::imuBias::ConstantBias init_bias = imu_measurements_->biasHat();

  while (!imu_queue_->hasMeasurementsInRange(keyframe_transforms[0].frame1->timestamp,
                                             keyframe_transforms[0].frame2->timestamp))
  {
    std::cout << "No IMU measurements in time range " << std::setprecision(17)
              << keyframe_transforms[0].frame1->timestamp << " -> " << keyframe_transforms[0].frame2->timestamp
              << std::endl;
    std::cout << "Waiting 1 ms" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  imu_queue_->integrateIMUMeasurements(imu_measurements_, keyframe_transforms[0].frame1->timestamp,
                                       keyframe_transforms[0].frame2->timestamp);
  gtsam::Vector3 init_velocity = imu_measurements_->predict(gtsam::NavState(), init_bias).velocity();
  imu_measurements_->resetIntegrationAndSetBias(init_bias);

  std::cout << "init velocity " << init_velocity << std::endl;

  auto noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(0.0001), gtsam::Vector3::Constant(0.0001)).finished());
  auto noise_v = gtsam::noiseModel::Isotropic::Sigma(3, 0.5);
  auto noise_b = marginals_->marginalCovariance(B(last_frame_id_added_));
  auto noise_E = gtsam::noiseModel::Isotropic::Sigma(5, 0.1);

  graph_->addPrior(X(keyframe_transforms[0].frame1->id), init_pose, noise_x);
  graph_->addPrior(V(keyframe_transforms[0].frame1->id), init_velocity, noise_v);
  graph_->addPrior(B(keyframe_transforms[0].frame1->id), init_bias, noise_b);

  values_->insert(X(keyframe_transforms[0].frame1->id), init_pose);
  values_->insert(V(keyframe_transforms[0].frame1->id), init_velocity);
  values_->insert(B(keyframe_transforms[0].frame1->id), init_bias);

  added_frame_timestamps_[keyframe_transforms[0].frame1->id] = keyframe_transforms[0].frame1->timestamp;

  std::map<int, bool> frame_ids;
  frame_ids[keyframe_transforms[0].frame1->id] = true;

  gtsam::Pose3 imu_to_cam = gtsam::Pose3(
      gtsam::Rot3::Quaternion(GlobalParams::IMUCamQuat()[3], GlobalParams::IMUCamQuat()[0],
                              GlobalParams::IMUCamQuat()[1], GlobalParams::IMUCamQuat()[2]),
      gtsam::Point3(GlobalParams::IMUCamVector()[0], GlobalParams::IMUCamVector()[1], GlobalParams::IMUCamVector()[2]));

  gtsam::NavState navstate(init_pose, init_velocity);
  for (auto& keyframe_transform : keyframe_transforms)
  {
    added_frame_timestamps_[keyframe_transform.frame2->id] = keyframe_transform.frame2->timestamp;
    while (
        !imu_queue_->hasMeasurementsInRange(keyframe_transform.frame1->timestamp, keyframe_transform.frame2->timestamp))
    {
      std::cout << "No IMU measurements in time range " << std::setprecision(17) << keyframe_transform.frame1->timestamp
                << " -> " << keyframe_transform.frame2->timestamp << std::endl;
      std::cout << "Waiting 1 ms" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    imu_queue_->integrateIMUMeasurements(imu_measurements_, keyframe_transform.frame1->timestamp,
                                         keyframe_transform.frame2->timestamp);
    navstate = imu_measurements_->predict(navstate, imu_measurements_->biasHat());

    auto E_mat = ToMatrix3(*keyframe_transform.GetEssentialMat());
    auto R_mat = ToMatrix3(*keyframe_transform.GetRotation());
    auto t = *keyframe_transform.GetTranslation();
    gtsam::Unit3 t_unit_cam_frame(t[0], t[1], t[2]);
    gtsam::Rot3 R_cam_frame(R_mat);

    gtsam::Unit3 t_i(imu_to_cam.inverse() * t_unit_cam_frame.point3());
    gtsam::Rot3 R_i(imu_to_cam.rotation() * R_cam_frame * imu_to_cam.rotation().inverse());  // TODO swap inversions?
    gtsam::Point3 t_i_scaled(navstate.pose().translation().norm() * t_i.point3());

    std::cout << "==================== Frame " << keyframe_transform.frame1->id
              << " -> " << keyframe_transform.frame2->id
              << " ====================" << std::endl;

    std::cout << "R_c = " << R_cam_frame.ypr() << std::endl;
    t_unit_cam_frame.print("t_c = ");
    std::cout << "R_i = " << R_i.ypr() << std::endl;
    t_i.print("t_i = ");
    t_i_scaled.print("t_i_scaled = ");
    std::cout << "navstate R_i = " << navstate.pose().rotation().ypr() << std::endl;
    std::cout << "navstate t_i = " << navstate.pose().translation() << std::endl;
    std::cout << "navstate v_i = " << navstate.velocity() << std::endl;

    std::cout << "=========================================================" << std::endl;

    if (GlobalParams::AddEssentialMatrixConstraints())
    {
      gtsam::EssentialMatrix E(R_i, t_i);
      gtsam::EssentialMatrixConstraint E_factor(X(keyframe_transform.frame1->id), X(keyframe_transform.frame2->id), E,
                                                noise_E);
      graph_->add(E_factor);  // Doing this invalidates assumption of independent measurements
    }

    values_->insert(X(keyframe_transform.frame2->id), gtsam::Pose3(R_i, t_i_scaled));
    values_->insert(V(keyframe_transform.frame2->id), navstate.velocity());
    values_->insert(B(keyframe_transform.frame2->id), imu_measurements_->biasHat());

    auto imu_combined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imu_measurements_);
    gtsam::CombinedImuFactor imu_factor(X(keyframe_transform.frame1->id), V(keyframe_transform.frame1->id),
                                        X(keyframe_transform.frame2->id), V(keyframe_transform.frame2->id),
                                        B(keyframe_transform.frame1->id), B(keyframe_transform.frame2->id),
                                        imu_combined);
    graph_->add(imu_factor);
    imu_measurements_->resetIntegrationAndSetBias(imu_measurements_->biasHat());
    frame_ids[keyframe_transform.frame2->id] = true;
  }

  gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(GlobalParams::CamFx(), GlobalParams::CamFy(), 0.0,
                                                  GlobalParams::CamU0(), GlobalParams::CamV0()));

  auto feature_noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  // auto body_P_sensor = gtsam::Pose3(gtsam::Rot3::Ypr(-M_PI / 2, 0, -M_PI / 2), gtsam::Point3::Zero());
  for (auto& track : tracks)
  {
    SmartFactor::shared_ptr smart_factor(
        new SmartFactor(feature_noise, K, imu_to_cam.inverse(), GetSmartProjectionParams()));
    for (auto& feature : track->key_features)
    {
      if (frame_ids.count(feature->frame->id))
      {
        auto pt = feature->pt;
        smart_factor->add(gtsam::Point2(pt.x, pt.y), X(feature->frame->id));
      }
    }
    if (smart_factor->size() >= GlobalParams::MinTrackLengthForSmoothing())
    {
      std::cout << "adding landmark with " << track->key_features.size() << " observations" << std::endl;
      smart_factors_[track->id] = smart_factor;
      graph_->add(smart_factor);
    }
  }
  std::cout << "Added " << smart_factors_.size() << " smart factors" << std::endl;

  if (GlobalParams::UseIsam())
  {
    isam2->update(*graph_, *values_);
    *values_ = isam2->calculateEstimate();
  }
  else
  {
    gtsam::GaussNewtonOptimizer optimizer(*graph_, *values_);
    *values_ = optimizer.optimize();
  }

  // TODO use an accessor method for this instead
  pose_estimates.push_back(
      Pose3Stamped{ .pose = ToPose(values_->at<gtsam::Pose3>(X(keyframe_transforms[0].frame1->id))),
                    .stamp = keyframe_transforms[0].frame1->timestamp });
  for (auto& keyframe_transform : keyframe_transforms)
  {
    pose_estimates.push_back(Pose3Stamped{ .pose = ToPose(values_->at<gtsam::Pose3>(X(keyframe_transform.frame2->id))),
                                           .stamp = keyframe_transform.frame2->timestamp });
  }
  // TODO end

  for (const auto& smart_factor_pair : smart_factors_)
  {
    auto smart_factor = smart_factor_pair.second;
    /*
    std::cout << "reproj error: " << smart_factor->reprojectionErrorAfterTriangulation(gn_result)
              << std::endl;
    std::cout << "error: " << smart_factor->error(gn_result) << std::endl;
    std::cout << "outlier: " << smart_factor->isOutlier() << std::endl;
    std::cout << "degenerate: " << smart_factor->isDegenerate() << std::endl;
    std::cout << "far point: " << smart_factor->isFarPoint() << std::endl;
    std::cout << "last feature: " << smart_factor->measured().back() << std::endl;
     */
    if (smart_factor->isValid())
    {
      gtsam::Matrix E;
      smart_factor->triangulateAndComputeE(E, *values_);
      auto P = smart_factor->PointCov(E);
      // std::cout << "point covariance: " << std::endl << P << std::endl;
      if (P.norm() < 1)
      {
        boost::optional<gtsam::Point3> point = smart_factor->point();
        if (point)  // I think this check is redundant because we already check if the factor is valid, but doesn't hurt
        {           // ignore if boost::optional returns nullptr
          landmark_estimates[smart_factor_pair.first] = ToPoint(*point);
        }
      }
      else
      {
        std::cout << "Depth error large for landmark " << smart_factor_pair.first << ": " << P(0, 0) << " (norm "
                  << P.norm() << ")" << std::endl;
      }
    }

    /* Debug backprojection
    gtsam::Point3 point = smart_factor->cameras(gn_result).back().backproject(smart_factor->measured().back(), 10);
    landmark_estimates.push_back(ToPoint(point));
     */
  }
  auto imu_bias = values_->at<gtsam::imuBias::ConstantBias>(B(keyframe_transforms.back().frame2->id));
  imu_bias.print("imu bias: ");
  imu_measurements_->resetIntegrationAndSetBias(imu_bias);
  last_frame_id_added_ = keyframe_transforms.back().frame2->id;
  if (GlobalParams::UseIsam())
  {
    graph_->resize(0);
    values_->clear();
  }
  status_ = kLandmarksInitialized;
}

Smoother::Smoother(std::shared_ptr<IMUQueue> imu_queue)
  : isam2(new gtsam::ISAM2())
  , graph_(new gtsam::NonlinearFactorGraph())
  , values_(new gtsam::Values())
  , imu_queue_(std::move(imu_queue))
  , imu_measurements_(MakeIMUIntegrator())
{
}

Pose3Stamped Smoother::Update(const shared_ptr<Frame>& frame, const std::vector<shared_ptr<Track>>& tracks,
                              std::vector<Pose3Stamped>& pose_estimates, std::map<int, Point3>& landmark_estimates)
{
  std::cout << "Performing isam update for frame " << frame->id << std::endl;

  gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(GlobalParams::CamFx(), GlobalParams::CamFy(), 0.0,
                                                  GlobalParams::CamU0(), GlobalParams::CamV0()));

  auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  auto prev_estimate = GlobalParams::UseIsam() ? isam2->calculateEstimate() : *values_;

  gtsam::Pose3 imu_to_cam = gtsam::Pose3(
      gtsam::Rot3::Quaternion(GlobalParams::IMUCamQuat()[3], GlobalParams::IMUCamQuat()[0],
                              GlobalParams::IMUCamQuat()[1], GlobalParams::IMUCamQuat()[2]),
      gtsam::Point3(GlobalParams::IMUCamVector()[0], GlobalParams::IMUCamVector()[1], GlobalParams::IMUCamVector()[2]));

  for (auto& track : tracks)
  {
    assert(track->features.back()->frame->id == frame->id);
    if (smart_factors_.count(track->id))
    {
      auto pt = track->features.back()->pt;
      smart_factors_[track->id]->add(gtsam::Point2(pt.x, pt.y), X(track->features.back()->frame->id));
    }
    else
    {
      SmartFactor::shared_ptr smart_factor(
          new SmartFactor(measurementNoise, K, imu_to_cam.inverse(), GetSmartProjectionParams()));
      for (size_t i = 0; i < track->features.size() - 1; ++i)
      {
        auto feature = track->features[i];
        if (added_frame_timestamps_.count(feature->frame->id) || feature->frame->id == frame->id)
        {
          smart_factor->add(gtsam::Point2(feature->pt.x, feature->pt.y), X(feature->frame->id));
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
          if (P.norm() < 1)
          {
            auto new_feature = track->features.back();
            smart_factor->add(gtsam::Point2(new_feature->pt.x, new_feature->pt.y), X(new_feature->frame->id));
            std::cout << "initializing landmark " << track->id << " with " << track->features.size() << " observations"
                      << std::endl;
            smart_factors_[track->id] = smart_factor;
            graph_->add(smart_factor);
          }
          else
          {
            std::cout << "Skipping landmark " << track->id << " with P norm " << P.norm() << std::endl;
          }
        }
      }
    }
  }
  std::cout << "Have " << smart_factors_.size() << " smart factors" << std::endl;

  // std::cout << "prev prev " << prev_prev_pose.rotation().ypr() << std::endl;
  // std::cout << "prev " << prev_pose.rotation().ypr() << std::endl;
  // std::cout << "motion delta " << motion_delta.rotation().ypr() << std::endl;
  // std::cout << "motion predicted pose " << motion_predicted_pose.rotation().ypr() << std::endl;

  // gtsam::Pose3 gt_pose = ToGtsamPose(NewerCollegeGroundTruth::At(frame->timestamp));

  // TODO extract a method WaitForIMUAndIntegrate(timestamp1, timestamp2, imu_measurements)
  while (!imu_queue_->hasMeasurementsInRange(added_frame_timestamps_[last_frame_id_added_], frame->timestamp))
  {
    std::cout << "No IMU measurements in time range " << std::setprecision(17)
              << added_frame_timestamps_[last_frame_id_added_] << " -> " << frame->timestamp << std::endl;
    std::cout << "Waiting 1 ms" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  imu_queue_->integrateIMUMeasurements(imu_measurements_, added_frame_timestamps_[last_frame_id_added_],
                                       frame->timestamp);
  // TODO end extract

  // TODO all these can also be extracted
  auto prev_pose = prev_estimate.at<gtsam::Pose3>(X(last_frame_id_added_));
  auto prev_velocity = prev_estimate.at<gtsam::Vector3>(V(last_frame_id_added_));
  auto prev_bias = prev_estimate.at<gtsam::imuBias::ConstantBias>(B(last_frame_id_added_));
  // TODO end

  auto predicted_navstate = imu_measurements_->predict(gtsam::NavState(prev_pose, prev_velocity), prev_bias);

  values_->insert(X(frame->id), predicted_navstate.pose());
  values_->insert(V(frame->id), predicted_navstate.velocity());
  values_->insert(B(frame->id), prev_bias);

  auto imu_combined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imu_measurements_);
  gtsam::CombinedImuFactor imu_factor(X(last_frame_id_added_), V(last_frame_id_added_), X(frame->id), V(frame->id),
                                      B(last_frame_id_added_), B(frame->id), imu_combined);
  graph_->add(imu_factor);

  try
  {
    if (GlobalParams::UseIsam())
    {
      auto isam_result = isam2->update(*graph_, *values_);
      isam_result.print("isam result ");
      *values_ = isam2->calculateEstimate();
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

  for (const auto& smart_factor_pair : smart_factors_)
  {
    boost::optional<gtsam::Point3> point = smart_factor_pair.second->point();
    if (point)
    {  // ignore if boost::optional returns nullptr
      landmark_estimates[smart_factor_pair.first] = ToPoint(*point);
    }
    else
    {
      landmark_estimates[smart_factor_pair.first] = Point3{ .x = 0, .y = 0, .z = 0 };
    }
  }

  imu_measurements_->resetIntegrationAndSetBias(values_->at<gtsam::imuBias::ConstantBias>(B(frame->id)));
  std::cout << "got bias" << std::endl;

  auto new_pose = values_->at<gtsam::Pose3>(X(frame->id));
  std::cout << "got new pose" << std::endl;

  last_frame_id_added_ = frame->id;
  added_frame_timestamps_[frame->id] = frame->timestamp;

  // TODO use an accessor method for this
  for (auto& f : added_frame_timestamps_)
  {
    auto id = f.first;
    auto ts = f.second;

    pose_estimates.push_back(Pose3Stamped{ .pose = ToPose(values_->at<gtsam::Pose3>(X(id))), .stamp = ts });
  }
  // TODO end

  for (auto& f : added_frame_timestamps_)
  {
    auto imu_bias = values_->at<gtsam::imuBias::ConstantBias>(B(f.first));
    imu_bias.print("imu bias: ");
  }

  if (GlobalParams::UseIsam())
  {
    graph_->resize(0);
    values_->clear();
  }

  return Pose3Stamped{ .pose = ToPose(new_pose), .stamp = frame->timestamp };
}
void Smoother::InitIMU(const vector<shared_ptr<Frame>>& frames, std::vector<Pose3Stamped>& pose_estimates)
{
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;

  auto zero_pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3::Zero());
  auto zero_velocity = gtsam::Vector3(0, 0, 0);
  auto init_bias = imu_measurements_->biasHat();

  auto noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(0.0001), gtsam::Vector3::Constant(0.0001)).finished());
  auto noise_v = gtsam::noiseModel::Isotropic::Sigma(3, 0.0001);
  auto noise_b = gtsam::noiseModel::Isotropic::Sigma(6, 10);  // High noise on bias

  values.insert(X(frames[0]->id), zero_pose);
  values.insert(V(frames[0]->id), zero_velocity);
  values.insert(B(frames[0]->id), init_bias);

  graph.addPrior(X(frames[0]->id), zero_pose, noise_x);
  graph.addPrior(V(frames[0]->id), zero_velocity, noise_v);
  graph.addPrior(B(frames[0]->id), init_bias, noise_b);

  bool moved = false;

  gtsam::NavState navstate = gtsam::NavState();
  for (size_t i = 1; i < frames.size(); ++i)
  {
    while (!imu_queue_->hasMeasurementsInRange(frames[i - 1]->timestamp, frames[i]->timestamp))
    {
      std::cout << "No IMU measurements in time range " << std::setprecision(17) << frames[i - 1]->timestamp << " -> "
                << frames[i]->timestamp << std::endl;
      std::cout << "Waiting 1 ms" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    imu_queue_->integrateIMUMeasurements(imu_measurements_, frames[i - 1]->timestamp, frames[i]->timestamp);

    if (!frames[i]->stationary)
    {
      moved = true;
    }

    if (!moved)
    {
      graph.addPrior(X(frames[i]->id), zero_pose, noise_x);
    }
    if (frames[i]->stationary)
    {
      graph.addPrior(V(frames[i]->id), zero_velocity, noise_v);
      values.insert(V(frames[i]->id), zero_velocity);
      navstate = gtsam::NavState(imu_measurements_->predict(navstate, init_bias).pose(), zero_velocity);
    }
    else
    {
      navstate = imu_measurements_->predict(navstate, init_bias);
      values.insert(V(frames[i]->id), navstate.velocity());
    }

    values.insert(X(frames[i]->id), navstate.pose());
    values.insert(B(frames[i]->id), init_bias);

    auto imu_combined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imu_measurements_);
    gtsam::CombinedImuFactor imu_factor(X(frames[i - 1]->id), V(frames[i - 1]->id), X(frames[i]->id), V(frames[i]->id),
                                        B(frames[i - 1]->id), B(frames[i]->id), imu_combined);

    graph.add(imu_factor);
    imu_measurements_->resetIntegrationAndSetBias(imu_measurements_->biasHat());
  }

  gtsam::GaussNewtonOptimizer optimizer(graph, values);
  auto gn_result = optimizer.optimize();
  auto imu_bias = gn_result.at<gtsam::imuBias::ConstantBias>(B(frames.back()->id));
  imu_bias.print("imu bias: ");
  marginals_ = new gtsam::Marginals(graph, gn_result);
  std::cout << cout.precision(2) << marginals_->marginalCovariance(B(frames.back()->id)) << std::endl;
  last_frame_id_added_ = frames.back()->id;

  imu_measurements_->resetIntegrationAndSetBias(imu_bias);
  status_ = kIMUInitialized;

  /*
  for (auto& frame : frames)
  {
    pose_estimates.push_back(
        Pose3Stamped{ .pose = ToPose(gn_result.at<gtsam::Pose3>(X(frame->id))), .stamp = frame->timestamp });
  }
   */
}

BackendStatus Smoother::GetStatus()
{
  return status_;
}
