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
                                   std::vector<Pose3Stamped>& pose_estimates, std::vector<Point3>& landmark_estimates)
{
  std::cout << "Let's initialize those landmarks!" << std::endl;
  gtsam::NonlinearFactorGraph inertial_graph;
  gtsam::NonlinearFactorGraph batch_graph;

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
  auto noise_b = gtsam::noiseModel::Isotropic::Sigma(6, 1);
  auto noise_E = gtsam::noiseModel::Isotropic::Sigma(5, 0.1);

  inertial_graph.addPrior(X(keyframe_transforms[0].frame1->id), init_pose, noise_x);
  inertial_graph.addPrior(V(keyframe_transforms[0].frame1->id), init_velocity, noise_v);
  inertial_graph.addPrior(B(keyframe_transforms[0].frame1->id), init_bias, noise_b);

  values_->insert(X(keyframe_transforms[0].frame1->id), init_pose);
  values_->insert(V(keyframe_transforms[0].frame1->id), init_velocity);
  values_->insert(B(keyframe_transforms[0].frame1->id), init_bias);

  std::map<int, bool> frame_ids;
  frame_ids[keyframe_transforms[0].frame1->id] = true;

  gtsam::Pose3 imu_to_cam = gtsam::Pose3(
      gtsam::Rot3::Quaternion(GlobalParams::IMUCamQuat()[3], GlobalParams::IMUCamQuat()[0],
                              GlobalParams::IMUCamQuat()[1], GlobalParams::IMUCamQuat()[2]),
      gtsam::Point3(GlobalParams::IMUCamVector()[0], GlobalParams::IMUCamVector()[1], GlobalParams::IMUCamVector()[2]));

  gtsam::NavState navstate(init_pose, init_velocity);
  for (auto& keyframe_transform : keyframe_transforms)
  {
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

    auto E_mat = ToMatrix3(keyframe_transform.GetEssentialMat());
    auto R_mat = ToMatrix3(keyframe_transform.GetRotation());
    auto t = keyframe_transform.GetTranslation();
    gtsam::Unit3 t_unit_cam_frame(t[0], t[1], t[2]);
    gtsam::Rot3 R_cam_frame(R_mat);

    gtsam::Unit3 t_i(imu_to_cam.inverse() * t_unit_cam_frame.point3());
    gtsam::Rot3 R_i(imu_to_cam.rotation() * R_cam_frame * imu_to_cam.rotation().inverse());  // TODO swap inversions?
    gtsam::Point3 t_i_scaled(navstate.pose().translation().norm() * t_i.point3());

    std::cout << "========================= Begin =========================" << std::endl;

    std::cout << "R_c = " << R_cam_frame.ypr() << std::endl;
    t_unit_cam_frame.print("t_c = ");
    std::cout << "R_i = " << R_i.ypr() << std::endl;
    t_i.print("t_i = ");
    t_i_scaled.print("t_i_scaled = ");
    std::cout << "navstate R_i = " << navstate.pose().rotation().ypr() << std::endl;
    std::cout << "navstate t_i = " << navstate.pose().translation() << std::endl;
    std::cout << "navstate v_i = " << navstate.velocity() << std::endl;

    std::cout << "=========================================================" << std::endl;

    if (GlobalParams::AddEssentialMatrixConstraints()) {
      gtsam::EssentialMatrix E(R_i, t_i);
      gtsam::EssentialMatrixConstraint E_factor(X(keyframe_transform.frame1->id), X(keyframe_transform.frame2->id), E,
                                                noise_E);
      batch_graph.add(E_factor);
    }

    values_->insert(X(keyframe_transform.frame2->id), gtsam::Pose3(R_i, t_i_scaled));
    values_->insert(V(keyframe_transform.frame2->id), navstate.velocity());
    values_->insert(B(keyframe_transform.frame2->id), imu_measurements_->biasHat());

    auto imu_combined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imu_measurements_);
    gtsam::CombinedImuFactor imu_factor(X(keyframe_transform.frame1->id), V(keyframe_transform.frame1->id),
                                        X(keyframe_transform.frame2->id), V(keyframe_transform.frame2->id),
                                        B(keyframe_transform.frame1->id), B(keyframe_transform.frame2->id),
                                        imu_combined);
    inertial_graph.add(imu_factor);
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
      batch_graph.add(smart_factor);
    }
  }
  std::cout << "Added " << smart_factors_.size() << " smart factors" << std::endl;

  // auto result = isam2->update(graph, init_values);
  // result.print("result");

  batch_graph.add(inertial_graph);

  gtsam::GaussNewtonOptimizer optimizer(batch_graph, *values_);
  auto gn_result = optimizer.optimize();

  std::cout << "gn worked!" << std::endl;

  pose_estimates.push_back(
      Pose3Stamped{ .pose = ToPose(gn_result.at<gtsam::Pose3>(X(keyframe_transforms[0].frame1->id))),
                    .stamp = keyframe_transforms[0].frame1->timestamp });
  for (auto& keyframe_transform : keyframe_transforms)
  {
    pose_estimates.push_back(Pose3Stamped{ .pose = ToPose(gn_result.at<gtsam::Pose3>(X(keyframe_transform.frame2->id))),
                                           .stamp = keyframe_transform.frame2->timestamp });
  }

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
      smart_factor->triangulateAndComputeE(E, gn_result);
      auto P = smart_factor->PointCov(E);
      // std::cout << "point covariance: " << std::endl << P << std::endl;
      if (P.norm() < 1)
      {
        boost::optional<gtsam::Point3> point = smart_factor->point();
        if (point)  // I think this check is redundant because we already check if the factor is valid, but doesn't hurt
        {           // ignore if boost::optional returns nullptr
          landmark_estimates.push_back(ToPoint(*point));
          graph_->add(smart_factor);
        }
      }
      else
      {
        // std::cout << "Depth error is large! " << P(0, 0) << " (norm " << P.norm() << ")" << std::endl;
      }
    }

    /* Debug backprojection
    gtsam::Point3 point = smart_factor->cameras(gn_result).back().backproject(smart_factor->measured().back(), 10);
    landmark_estimates.push_back(ToPoint(point));
     */
  }
  graph_->add(inertial_graph);
  // isam2->update(*graph_, *values_);

  graph_->resize(0);
  values_->clear();
  auto imu_bias = gn_result.at<gtsam::imuBias::ConstantBias>(B(keyframe_transforms.back().frame2->id));
  // auto imu_bias = isam2->calculateEstimate<gtsam::imuBias::ConstantBias>(B(frames.back()->id));
  imu_bias.print("imu bias: ");
  imu_measurements_->resetIntegrationAndSetBias(imu_bias);
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
                              vector<Point3>& landmark_estimates)
{
  std::cout << "Performing isam update for frame " << frame->id << std::endl;

  gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(GlobalParams::CamFx(), GlobalParams::CamFy(), 0.0,
                                                  GlobalParams::CamU0(), GlobalParams::CamV0()));

  auto body_P_sensor = gtsam::Pose3(gtsam::Rot3::Ypr(-M_PI / 2, 0, -M_PI / 2), gtsam::Point3::Zero());
  auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  auto prev_estimate = isam2->calculateEstimate();
  prev_estimate.insert(*values_);
  for (auto& track : tracks)
  {
    if (smart_factors_.count(track->id))
    {
      auto pt = track->features.back()->pt;
      smart_factors_[track->id]->add(gtsam::Point2(pt.x, pt.y), X(track->features.back()->frame->id));
    }
    else
    {
      SmartFactor::shared_ptr smart_factor(
          new SmartFactor(measurementNoise, K, body_P_sensor, GetSmartProjectionParams()));
      for (size_t i = 0; i < track->features.size() - 1; ++i)
      {
        auto feature = track->features[i];
        smart_factor->add(gtsam::Point2(feature->pt.x, feature->pt.y), X(feature->frame->id));
      }
      auto triangulationResult = smart_factor->triangulateSafe(smart_factor->cameras(prev_estimate));
      if (triangulationResult.valid())
      {
        gtsam::Matrix E;
        smart_factor->triangulateAndComputeE(E, prev_estimate);
        auto P = smart_factor->PointCov(E);
        if (track->features.size() > GlobalParams::MinTrackLengthForSmoothing() && P.norm() < 1)
        {
          auto new_feature = track->features.back();
          smart_factor->add(gtsam::Point2(new_feature->pt.x, new_feature->pt.y), X(new_feature->frame->id));
          std::cout << "initializing landmark " << track->id << " with " << track->features.size() << " observations"
                    << std::endl;
          smart_factors_[track->id] = smart_factor;
          graph_->add(smart_factor);
        }
      }
    }
  }
  std::cout << "Have " << smart_factors_.size() << " smart factors" << std::endl;

  auto prev_pose = ((frame->id % 5) == 1) ? isam2->calculateEstimate<gtsam::Pose3>(X(frame->id - 1)) :
                                            values_->at<gtsam::Pose3>(X(frame->id - 1));
  auto prev_prev_pose = (frame->id % 5 == 1 || frame->id % 5 == 2) ?
                            isam2->calculateEstimate<gtsam::Pose3>(X(frame->id - 2)) :
                            values_->at<gtsam::Pose3>(X(frame->id - 2));
  auto motion_delta = prev_prev_pose.inverse().compose(prev_pose);
  auto motion_predicted_pose = prev_pose.compose(motion_delta);

  // std::cout << "prev prev " << prev_prev_pose.rotation().ypr() << std::endl;
  // std::cout << "prev " << prev_pose.rotation().ypr() << std::endl;
  // std::cout << "motion delta " << motion_delta.rotation().ypr() << std::endl;
  // std::cout << "motion predicted pose " << motion_predicted_pose.rotation().ypr() << std::endl;

  // gtsam::Pose3 gt_pose = ToGtsamPose(NewerCollegeGroundTruth::At(frame->timestamp));

  values_->insert(X(frame->id), motion_predicted_pose);

  // auto between_noise = gtsam::noiseModel::Diagonal::Sigmas(
  //    (gtsam::Vector(6) << gtsam::Vector3::Constant(0.3), gtsam::Vector3::Constant(0.2)).finished());
  // new_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(frame->id - 1, frame->id, motion_delta, between_noise));

  // auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
  //    (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.1)).finished());
  // new_factors.addPrior(frame->id, motion_predicted_pose, prior_noise);

  if (frame->id % 5 == 0)
  {
    gtsam::ISAM2Result result;
    try
    {
      result = isam2->update(*graph_, *values_);
    }
    catch (exception& e)
    {
      for (auto& track : tracks)
      {
        if (track->features.size() == GlobalParams::MinTrackLengthForSmoothing())  // the tracks that were added
        {
          std::cout << "track " << track->id << ":";
          for (auto& feature : track->features)
          {
            std::cout << feature->pt << ", ";
          }
          std::cout << std::endl;
        }
      }
      throw;
    }
    result.print("isam2 result");

    for (const auto& smart_factor_pair : smart_factors_)
    {
      boost::optional<gtsam::Point3> point = smart_factor_pair.second->point();
      if (point)
      {  // ignore if boost::optional returns nullptr
        landmark_estimates.push_back(ToPoint(*point));
      }
    }
    graph_->resize(0);
    values_->clear();
  }

  return Pose3Stamped{ .pose = frame->id % 5 == 0 ? ToPose(isam2->calculateEstimate<gtsam::Pose3>(X(frame->id))) :
                                                    ToPose(motion_predicted_pose),
                       .stamp = frame->timestamp };
}
void Smoother::InitIMUOnly(const vector<std::shared_ptr<Frame>>& frames, const vector<std::shared_ptr<Track>>& tracks,
                           vector<Pose3Stamped>& pose_estimates)
{
  std::cout << "Let's process those " << tracks.size() << " tracks!" << std::endl;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values init_values;

  // auto init_bias = gtsam::imuBias::ConstantBias(gtsam::Vector3(0.25, 0, 0.87), gtsam::Vector3(0, 0, 0));
  // auto init_pose = gtsam::Pose3(gtsam::Rot3::Ypr(-M_PI / 2, 0, -M_PI / 2), gtsam::Point3::Zero());

  // auto init_bias = gtsam::imuBias::ConstantBias(gtsam::Vector3(-0.87, -0.25, -1), gtsam::Vector3(0, 0, 0));
  auto init_bias = gtsam::imuBias::ConstantBias();
  auto init_pose = gtsam::Pose3(gtsam::Rot3::Ypr(0, 0, 0), gtsam::Point3::Zero());

  init_values.insert(X(frames[0]->id), init_pose);
  init_values.insert(V(frames[0]->id), gtsam::Vector3(0, 0, 0));
  init_values.insert(B(frames[0]->id), init_bias);

  auto imu_measurements = MakeIMUIntegrator();

  // After init, we don't move from the spot, but we rotate
  auto prior_noise_x_later = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(5), gtsam::Vector3::Constant(0.3)).finished());

  for (int i = 1; i < frames.size(); ++i)
  {
    init_values.insert(X(frames[i]->id), init_pose);
    init_values.insert(V(frames[i]->id), gtsam::Vector3(0, 0, 0));
    init_values.insert(B(frames[i]->id), init_bias);

    graph.addPrior(X(frames[i]->id), init_pose, prior_noise_x_later);

    while (!imu_queue_->hasMeasurementsInRange(frames[i - 1]->timestamp, frames[i]->timestamp))
    {
      std::cout << "No IMU measurements in time range " << std::setprecision(17) << frames[i - 1]->timestamp << " -> "
                << frames[i]->timestamp << std::endl;
      std::cout << "Waiting 1 ms" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    int num_intg =
        imu_queue_->integrateIMUMeasurements(imu_measurements, frames[i - 1]->timestamp, frames[i]->timestamp);
    std::cout << "integrated " << num_intg << " imu messages" << std::endl;
    auto imu_combined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imu_measurements);
    gtsam::CombinedImuFactor imu_factor(X(frames[i - 1]->id), V(frames[i - 1]->id), X(frames[i]->id), V(frames[i]->id),
                                        B(frames[i - 1]->id), B(frames[i]->id), imu_combined);
    std::cout << frames[i]->id << std::endl;
    graph.add(imu_factor);
    imu_measurements->resetIntegration();
  }

  auto prior_noise_x_init = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(0.0001), gtsam::Vector3::Constant(0.0001)).finished());
  auto prior_noise_v = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  auto prior_noise_b = gtsam::noiseModel::Isotropic::Sigma(6, 0.5);

  graph.addPrior(X(0), init_pose, prior_noise_x_init);
  graph.addPrior(V(0), gtsam::Vector3(0, 0, 0), prior_noise_v);
  graph.addPrior(B(0), init_bias, prior_noise_b);

  gtsam::GaussNewtonOptimizer optimizer(graph, init_values);
  auto gn_result = optimizer.optimize();

  for (auto& frame : frames)
  {
    Pose3Stamped poseStamped{ .pose = ToPose(gn_result.at<gtsam::Pose3>(X(frame->id))), .stamp = frame->timestamp };
    pose_estimates.push_back(poseStamped);
  }

  /*
  *values_ = gn_result;
  isam2->update(*graph_, *values_);
  graph_->resize(0);
  values_->clear();
   */
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
