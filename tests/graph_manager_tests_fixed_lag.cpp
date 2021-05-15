#include <gtest/gtest.h>

#include <gtsam/nonlinear/ISAM2Result.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/geometry/Cal3_S2.h>

#include "graph_manager.h"
#include "helpers.h"
#include "incremental_fixed_lag_solver.h"

// INCLUDES FOR gtsam example
#include "incremental_fixed_lag_smoother_patched.h" // Keep this I guess. It's nice to have it tested
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <gtsam/base/debug.h>

// This one too only needed for gtsam example: remove
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

TEST(GraphManagerFixedLagTests, IMUOnly)
{
  gtsam::ISAM2Params isam2_params;
  isam2_params.findUnusedFactorSlots = true;
  GraphManager graph_manager(std::make_shared<IncrementalFixedLagSolver>(3.0, isam2_params),
                             (gtsam::SmartProjectionParams()));
  auto noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
  auto noise_v = gtsam::noiseModel::Isotropic::Sigma(3, 1.);
  auto noise_b = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
  gtsam::NavState init_nav_state;
  gtsam::imuBias::ConstantBias bias;

  gtsam::PreintegratedCombinedMeasurements pim(PimParams());

  gtsam::Vector3 measured_acc(1.0, 0.0, 9.81);
  gtsam::Vector3 measured_omega(0.0, 0.0, 0.0);
  double delta_t = 1.0;
  pim.integrateMeasurement(measured_acc, measured_omega, delta_t);

  graph_manager.SetInitNavstate(1, 0., init_nav_state, bias, noise_x, noise_v, noise_b);
  graph_manager.Update();

  std::vector<gtsam::NavState> nav_states = { init_nav_state };
  for (int i = 2; i < 7; ++i)
  {
    auto pred_nav_state = pim.predict(nav_states.back(), bias);
    graph_manager.AddFrame(i, (i - 1) * delta_t, pim, pred_nav_state, bias);
    nav_states.push_back(pred_nav_state);
    graph_manager.Update();
  }

  EXPECT_FALSE(graph_manager.CanAddObservationsForFrame(1, 0 * delta_t));
  EXPECT_FALSE(graph_manager.CanAddObservationsForFrame(2, 1 * delta_t));
  EXPECT_TRUE(graph_manager.CanAddObservationsForFrame(3, 2 * delta_t));
  EXPECT_TRUE(graph_manager.CanAddObservationsForFrame(4, 3 * delta_t));
}

TEST(GraphManagerFixedLagTests, SmartFactor)
{
  double lag = 2.5;
  gtsam::ISAM2Params isam2_params;
  isam2_params.findUnusedFactorSlots = true;
  GraphManager graph_manager(std::make_shared<IncrementalFixedLagSolver>(lag, isam2_params),
                             (gtsam::SmartProjectionParams()), lag);
  auto noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
  auto noise_v = gtsam::noiseModel::Isotropic::Sigma(3, 1.);
  auto noise_b = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
  auto feature_noise = gtsam::noiseModel::Isotropic::Sigma(2, 0.2);
  gtsam::NavState init_nav_state(gtsam::Pose3(), gtsam::Vector3(1.0, 0., 0.));
  gtsam::imuBias::ConstantBias bias;

  gtsam::PreintegratedCombinedMeasurements pim(PimParams());

  gtsam::Vector3 measured_acc(0.0, 0.0, 9.81);
  gtsam::Vector3 measured_omega(0.0, 0.0, 0.0);
  double delta_t = 1.0;
  pim.integrateMeasurement(measured_acc, measured_omega, delta_t);

  gtsam::Point3 landmark(10, 3, -1);
  auto body_p_cam = gtsam::Pose3(gtsam::Rot3::Ypr(-M_PI_2, 0., -M_PI_2), gtsam::Point3::Zero());
  auto K = gtsam::make_shared<gtsam::Cal3_S2>(650., 650., 0., 200., 200.);

  graph_manager.SetInitNavstate(1, 0., init_nav_state, bias, noise_x, noise_v, noise_b);

  auto init_feature = gtsam::PinholeCamera<gtsam::Cal3_S2>(init_nav_state.pose() * body_p_cam, *K).project(landmark);
  graph_manager.InitStructurelessLandmark(1, 1, 0., init_feature, K, body_p_cam, feature_noise);

  graph_manager.Update();

  std::vector<gtsam::NavState> nav_states = { init_nav_state };
  for (int i = 2; i < 5; ++i)
  {
    auto t = (i - 1) * delta_t;
    std::cout << "inserting frame " << i << " (t=" << t << ")" << std::endl;
    auto pred_nav_state = pim.predict(nav_states.back(), bias);
    graph_manager.AddFrame(i, t, pim, pred_nav_state, bias);
    nav_states.push_back(pred_nav_state);

    if (i == 3)
    {
      graph_manager.ConvertSmartFactorToProjectionFactor(1, 0., landmark);
    }

    if (t < lag)
    {
      EXPECT_TRUE(graph_manager.CanAddObservation(1, i));
    }
    else
    {
      EXPECT_FALSE(graph_manager.CanAddObservation(1, i));
    }
    std::cout << "CanAddObservation" << graph_manager.CanAddObservation(1, i) << std::endl;

    if (graph_manager.CanAddObservation(1, i))
    {
      auto feature = gtsam::PinholeCamera<gtsam::Cal3_S2>(pred_nav_state.pose() * body_p_cam, *K).project(landmark);
      graph_manager.AddLandmarkObservation(1, i, feature, K, body_p_cam);
    }

    std::cout << "Update frame " << i << " (t=" << t << ")" << std::endl;
    graph_manager.Update();
    std::cout << "Success" << std::endl;

    // We will observe the following only after the call to graph_manager.Update()
    if (t < lag)
    {
      EXPECT_TRUE(graph_manager.IsLandmarkTracked(1));
    }
    else
    {
      EXPECT_FALSE(graph_manager.IsLandmarkTracked(1));
    }
  }

  EXPECT_FALSE(graph_manager.CanAddObservationsForFrame(1, 0 * delta_t));
  EXPECT_FALSE(graph_manager.CanAddObservationsForFrame(2, 1 * delta_t));
  EXPECT_FALSE(graph_manager.CanAddObservationsForFrame(3, 2 * delta_t));
  EXPECT_TRUE(graph_manager.CanAddObservationsForFrame(4, 3 * delta_t));
  EXPECT_TRUE(graph_manager.CanAddObservationsForFrame(5, 4 * delta_t));
}

TEST(GraphManagerFixedLagTests, ProjectionFactor)
{
  double lag = 2.5;
  gtsam::ISAM2Params isam2_params;
  isam2_params.findUnusedFactorSlots = true;
  GraphManager graph_manager(std::make_shared<IncrementalFixedLagSolver>(lag, isam2_params),
                             (gtsam::SmartProjectionParams()), lag);
  auto noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
  auto noise_v = gtsam::noiseModel::Isotropic::Sigma(3, 1.);
  auto noise_b = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
  auto feature_noise = gtsam::noiseModel::Isotropic::Sigma(2, 0.2);
  auto range_noise = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
  gtsam::NavState init_nav_state;
  gtsam::imuBias::ConstantBias bias;

  gtsam::PreintegratedCombinedMeasurements pim(PimParams());

  gtsam::Vector3 measured_acc(1.0, 0.0, 9.81);
  gtsam::Vector3 measured_omega(0.0, 0.0, 0.0);
  double delta_t = 1.0;
  pim.integrateMeasurement(measured_acc, measured_omega, delta_t);

  gtsam::Point3 landmark(10, 3, -1);
  auto body_p_cam = gtsam::Pose3(gtsam::Rot3::Ypr(-M_PI_2, 0., -M_PI_2), gtsam::Point3::Zero());
  auto K = gtsam::make_shared<gtsam::Cal3_S2>(650., 650., 0., 200., 200.);

  graph_manager.SetInitNavstate(1, 0., init_nav_state, bias, noise_x, noise_v, noise_b);

  auto init_feature = gtsam::PinholeCamera<gtsam::Cal3_S2>(init_nav_state.pose() * body_p_cam, *K).project(landmark);
  graph_manager.InitProjectionLandmark(1, 1, 0., init_feature, landmark, K, body_p_cam, feature_noise);
  graph_manager.AddRangeObservation(1, 1, init_nav_state.pose().range(landmark), range_noise);

  graph_manager.Update();

  std::vector<gtsam::NavState> nav_states = { init_nav_state };
  for (int i = 2; i < 7; ++i)
  {
    auto t = (i - 1) * delta_t;
    auto pred_nav_state = pim.predict(nav_states.back(), bias);
    graph_manager.AddFrame(i, t, pim, pred_nav_state, bias);
    nav_states.push_back(pred_nav_state);

    if (t < lag)
    {
      EXPECT_TRUE(graph_manager.CanAddObservation(1, i));
    }
    else
    {
      EXPECT_FALSE(graph_manager.CanAddObservation(1, i));
    }

    if (graph_manager.CanAddObservation(1, i))
    {
      auto feature = gtsam::PinholeCamera<gtsam::Cal3_S2>(pred_nav_state.pose() * body_p_cam, *K).project(landmark);
      graph_manager.AddLandmarkObservation(1, i, feature, K, body_p_cam);
    }

    graph_manager.Update();

    // We will observe the following only after the call to graph_manager.Update()
    if (t < lag)
    {
      EXPECT_TRUE(graph_manager.IsLandmarkTracked(1));
    }
    else
    {
      EXPECT_FALSE(graph_manager.IsLandmarkTracked(1));
    }
  }

  EXPECT_FALSE(graph_manager.CanAddObservationsForFrame(1, 0 * delta_t));
  EXPECT_FALSE(graph_manager.CanAddObservationsForFrame(2, 1 * delta_t));
  EXPECT_FALSE(graph_manager.CanAddObservationsForFrame(3, 2 * delta_t));
  EXPECT_TRUE(graph_manager.CanAddObservationsForFrame(4, 3 * delta_t));
  EXPECT_TRUE(graph_manager.CanAddObservationsForFrame(5, 4 * delta_t));
}

TEST(ISAM2, CanMarginalizeSmartFactor)
{
  double lag = 4.5;
  gtsam::ISAM2Params isam2_params;
  //auto smoother = gtsam::IncrementalFixedLagSmoother(lag, isam2_params);
  auto isam = gtsam::ISAM2(isam2_params);


  auto noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
  auto noise_v = gtsam::noiseModel::Isotropic::Sigma(3, 1.);
  auto noise_b = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
  auto feature_noise = gtsam::noiseModel::Isotropic::Sigma(2, 0.2);

  gtsam::NavState init_nav_state(gtsam::Pose3(), gtsam::Vector3(1.0, 0., 0.));
  gtsam::imuBias::ConstantBias bias;

  auto pim_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
  pim_params->gyroscopeCovariance = 0.0001 * 0.0001 * gtsam::I_3x3;
  pim_params->accelerometerCovariance = 0.0001 * 0.0001 * gtsam::I_3x3;
  pim_params->integrationCovariance = 0.0001 * gtsam::I_3x3;
  gtsam::PreintegratedCombinedMeasurements pim(PimParams());

  gtsam::Vector3 measured_acc(1.0, 0.0, 9.81);
  gtsam::Vector3 measured_omega(0.0, 0.0, 0.0);
  double delta_t = 1.0;
  pim.integrateMeasurement(measured_acc, measured_omega, delta_t);

  gtsam::Point3 landmark(10, 3, 10);
  auto K = gtsam::make_shared<gtsam::Cal3_S2>(650., 650., 0., 200., 200.);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  gtsam::IncrementalFixedLagSmoother::KeyTimestampMap timestamps;

  // Pose prior - at identity
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3::identity(), noise_x);
  values.insert(X(0), init_nav_state.pose());
  timestamps[X(0)] = 0.0;

  // Bias prior
  graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), bias, noise_b));
  values.insert(B(0), bias);
  timestamps[B(0)] = 0.0;

  // Velocity prior - assume stationary
  graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), gtsam::Vector3(0, 0, 0), noise_v));
  values.insert(V(0), init_nav_state.velocity());
  timestamps[V(0)] = 0.0;

  gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>::shared_ptr smart_factor(
      new gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>(feature_noise, K));
  smart_factor->add(gtsam::PinholePose<gtsam::Cal3_S2>(init_nav_state.pose(), K).project(landmark), X(0));
  gtsam::FactorIndex smart_factor_idx_graph = graph.size();
  graph.push_back(smart_factor);

  //smoother.update(graph, values, timestamps);
  auto isam_result = isam.update(graph, values);
  graph.resize(0);
  values.clear();
  timestamps.clear();

  // Get the factor index within isam from the 1-1 graph<->isam factor index map
  gtsam::FactorIndex smart_factor_idx_isam = isam_result.newFactorsIndices.at(smart_factor_idx_graph);

  std::vector<gtsam::NavState> nav_states = { init_nav_state };
  for (int i = 1; i < 7; ++i)
  {
    gtsam::FastMap<gtsam::FactorIndex, gtsam::KeySet> smart_factor_new_affected_keys;
    auto t = i * delta_t;
    auto pred_nav_state = pim.predict(nav_states.back(), bias);
    nav_states.push_back(pred_nav_state);

    values.insert(X(i), pred_nav_state.pose());
    values.insert(V(i), pred_nav_state.velocity());
    values.insert(B(i), bias);

    timestamps[X(i)] = t;
    timestamps[V(i)] = t;
    timestamps[B(i)] = t;

    gtsam::CombinedImuFactor imu_factor(X(i - 1), V(i - 1), X(i), V(i), B(i - 1), B(i), pim);


    graph.add(imu_factor);
    if (t < lag)
    {
      smart_factor->add(gtsam::PinholePose<gtsam::Cal3_S2>(pred_nav_state.pose(), K).project(landmark), X(i));
      smart_factor_new_affected_keys[smart_factor_idx_isam].insert(X(i));
    }
    else if (i == 5) {
      std::cout << "Marginalizing x0" << std::endl;
      gtsam::KeyList key_list;
      key_list.push_back(X(0));
      isam.marginalizeLeaves(key_list);
    }
    std::cout << "frame " << i << " update" << std::endl;
    //smoother.update(graph, values, timestamps);

    gtsam::ISAM2UpdateParams params;
    params.newAffectedKeys = smart_factor_new_affected_keys;
    isam.update(graph, values);
    std::cout << "success " << std::endl;
    graph.resize(0);
    values.clear();
    timestamps.clear();
  }
}

TEST(IFLPatchedTest, CanMarginalizeSmartFactor)
{
  double lag = 4.5;
  gtsam::ISAM2Params isam2_params;
  isam2_params.findUnusedFactorSlots = true;
  auto smoother = IncrementalFixedLagSmootherPatched(lag, isam2_params);
  //auto isam = gtsam::ISAM2(isam2_params);


  auto noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
  auto noise_v = gtsam::noiseModel::Isotropic::Sigma(3, 1.);
  auto noise_b = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
  auto feature_noise = gtsam::noiseModel::Isotropic::Sigma(2, 0.2);

  gtsam::NavState init_nav_state(gtsam::Pose3(), gtsam::Vector3(1.0, 0., 0.));
  gtsam::imuBias::ConstantBias bias;

  auto pim_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
  pim_params->gyroscopeCovariance = 0.0001 * 0.0001 * gtsam::I_3x3;
  pim_params->accelerometerCovariance = 0.0001 * 0.0001 * gtsam::I_3x3;
  pim_params->integrationCovariance = 0.0001 * gtsam::I_3x3;
  gtsam::PreintegratedCombinedMeasurements pim(PimParams());

  gtsam::Vector3 measured_acc(1.0, 0.0, 9.81);
  gtsam::Vector3 measured_omega(0.0, 0.0, 0.0);
  double delta_t = 1.0;
  pim.integrateMeasurement(measured_acc, measured_omega, delta_t);

  gtsam::Point3 landmark(10, 3, 10);
  auto K = gtsam::make_shared<gtsam::Cal3_S2>(650., 650., 0., 200., 200.);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  gtsam::IncrementalFixedLagSmoother::KeyTimestampMap timestamps;

  // Pose prior - at identity
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3::identity(), noise_x);
  values.insert(X(0), init_nav_state.pose());
  timestamps[X(0)] = 0.0;

  // Bias prior
  graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), bias, noise_b));
  values.insert(B(0), bias);
  timestamps[B(0)] = 0.0;

  // Velocity prior - assume stationary
  graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), gtsam::Vector3(0, 0, 0), noise_v));
  values.insert(V(0), init_nav_state.velocity());
  timestamps[V(0)] = 0.0;

  gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>::shared_ptr smart_factor(
      new gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>(feature_noise, K));
  smart_factor->add(gtsam::PinholePose<gtsam::Cal3_S2>(init_nav_state.pose(), K).project(landmark), X(0));
  gtsam::FactorIndex smart_factor_idx_graph = graph.size();
  graph.push_back(smart_factor);

  smoother.update(graph, values, timestamps);
  auto isam_result = smoother.getISAM2Result();
  graph.resize(0);
  values.clear();
  timestamps.clear();

  // Get the factor index within isam from the 1-1 graph<->isam factor index map
  gtsam::FactorIndex smart_factor_idx_isam = isam_result.newFactorsIndices.at(smart_factor_idx_graph);

  std::vector<gtsam::NavState> nav_states = { init_nav_state };
  for (int i = 1; i < 7; ++i)
  {
    gtsam::FastMap<gtsam::FactorIndex, gtsam::KeySet> smart_factor_new_affected_keys;
    auto t = i * delta_t;
    auto pred_nav_state = pim.predict(nav_states.back(), bias);
    nav_states.push_back(pred_nav_state);

    values.insert(X(i), pred_nav_state.pose());
    values.insert(V(i), pred_nav_state.velocity());
    values.insert(B(i), bias);

    timestamps[X(i)] = t;
    timestamps[V(i)] = t;
    timestamps[B(i)] = t;

    gtsam::CombinedImuFactor imu_factor(X(i - 1), V(i - 1), X(i), V(i), B(i - 1), B(i), pim);

    graph.add(imu_factor);
    if (t < lag)
    {
      smart_factor->add(gtsam::PinholePose<gtsam::Cal3_S2>(pred_nav_state.pose(), K).project(landmark), X(i));
      smart_factor_new_affected_keys[smart_factor_idx_isam].insert(X(i));
    }
    std::cout << "frame " << i << " update" << std::endl;
    smoother.update(graph, values, timestamps, smart_factor_new_affected_keys);
    std::cout << "success " << std::endl;
    graph.resize(0);
    values.clear();
    timestamps.clear();
  }
}

/*

TEST(GTSAMStuff, IFLTest)
{
  auto K = gtsam::make_shared<gtsam::Cal3_S2>(650., 650., 0., 200., 200.);

  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.1;

  static constexpr double horizon = 1000.0;
  auto smoother = gtsam::IncrementalFixedLagSmoother(horizon, parameters);

  // Create a factor graph
  std::map<size_t, gtsam::SmartStereoProjectionPoseFactor::shared_ptr> smartFactors;
  gtsam::NonlinearFactorGraph graph;

  typedef gtsam::IncrementalFixedLagSmoother::KeyTimestampMap Timestamps;
  Timestamps newTimestamps;
  gtsam::Values initialEstimate;

  gtsam::PreintegratedCombinedMeasurements pim(PimParams());
  gtsam::imuBias::ConstantBias bias;
  auto noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
  auto noise_v = gtsam::noiseModel::Isotropic::Sigma(3, 1.);
  auto noise_b = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());

  gtsam::Point3 landmark(10, 0.5, 1);

  // Pose prior - at identity
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3::identity(), noise_x);
  initialEstimate.insert(X(0), gtsam::Pose3::identity());
  newTimestamps[X(0)] = 0.0;

  // Bias prior
  graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), bias, noise_b));
  initialEstimate.insert(B(0), bias);
  newTimestamps[B(0)] = 0.0;

  // Velocity prior - assume stationary
  graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), gtsam::Vector3(0, 0, 0), noise_v));
  initialEstimate.insert(V(0), gtsam::Vector3(0, 0, 0));
  newTimestamps[V(0)] = 0.0;

  int lastFrame = 1;
  int frame;
  std::map<int, double> landmark_to_first_timestamp;

  while (true)
  {
    if (frame != lastFrame)
    {
      cout << "Running iSAM for frame: " << lastFrame << "\n";

      initialEstimate.insert(X(lastFrame), gtsam::Pose3::identity());
      initialEstimate.insert(V(lastFrame), gtsam::Vector3(0, 0, 0));
      initialEstimate.insert(B(lastFrame), bias);

      newTimestamps[X(lastFrame)] = lastFrame;
      newTimestamps[V(lastFrame)] = lastFrame;
      newTimestamps[B(lastFrame)] = lastFrame;

      gtsam::CombinedImuFactor imuFactor(X(lastFrame - 1), V(lastFrame - 1), X(lastFrame), V(lastFrame),
                                         B(lastFrame - 1), B(lastFrame), pim);

      graph.add(imuFactor);

      smoother.update(graph, initialEstimate, newTimestamps);

      gtsam::Values currentEstimate = smoother.calculateEstimate();

      static int i = 0u;
      std::cerr << "Compute state covariance: keyframe # " << i++ << '\n';

      graph.print("New Graph Prior to Calculating Marginals!\n");

      //////////////////////////////////////////////////////////////////////////
      // TEST MARGINALS: this fails if we don't add measurements to the smart
      // factors
      // gtsam::Marginals marginals(smoother.getFactors(), currentEstimate, gtsam::Marginals::Factorization::CHOLESKY);
      //////////////////////////////////////////////////////////////////////////

      imu.propState = imu.preintegrated->predict(imu.prevState, imu.prevBias);
      imu.prevState = gtsam::NavState(currentEstimate.at<gtsam::Pose3>(X(lastFrame)),
                                      currentEstimate.at<gtsam::Vector3>(V(lastFrame)));

      graph.resize(0);
      initialEstimate.clear();
      newTimestamps.clear();
    }

    if (type == 'i')
    {  // Process IMU measurement
      double ax, ay, az;
      double gx, gy, gz;
      double dt = 1 / 800.0;  // IMU at ~800Hz

      ss >> ax;
      ss >> ay;
      ss >> az;

      ss >> gx;
      ss >> gy;
      ss >> gz;

      gtsam::Vector3 acc(ax, ay, az);
      gtsam::Vector3 gyr(gx, gy, gz);

      imu.preintegrated->integrateMeasurement(acc, gyr, dt);
    }
    else if (type == 's')
    {  // Process stereo measurement
      int landmark;
      double xl, xr, y;

      ss >> landmark;
      ss >> xl;
      ss >> xr;
      ss >> y;

      if (smartFactors.count(landmark) == 0)
      {
        auto gaussian = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);

        gtsam::SmartProjectionParams params(gtsam::HESSIAN, gtsam::ZERO_ON_DEGENERACY);

        smartFactors[landmark] = gtsam::SmartProjectionPoseFactor::shared_ptr(
            new gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>(gaussian, K, params));
        graph.push_back(smartFactors[landmark]);

        landmark_to_first_timestamp[landmark] = frame;
      }

      // if (frame - landmark_to_first_timestamp[landmark] < horizon) {
      //  smartFactors[landmark]->add(StereoPoint2(xl, xr, y), X(frame), K);
      // }
    }
    else
    {
      throw runtime_error("unexpected data type: " + string(1, type));
    }

    lastFrame = frame;
  }
}

 */