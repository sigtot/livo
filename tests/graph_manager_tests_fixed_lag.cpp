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
      graph_manager.AddLandmarkObservation(1, i, t, feature, K, body_p_cam);
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
  gtsam::NavState init_nav_state(gtsam::Rot3(), gtsam::Point3::Zero(), gtsam::Vector3(1.0, 0.0, 0.0));
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
  graph_manager.InitProjectionLandmark(1, 1, 0., init_feature, landmark, K, body_p_cam, feature_noise);
  graph_manager.AddRangeObservation(1, 1, 0., init_nav_state.pose().range(landmark), range_noise);

  graph_manager.Update();

  std::vector<gtsam::NavState> nav_states = { init_nav_state };
  double last_observation_t = 0.0;
  int n_obs = 3;
  int added_obs = 0;
  for (int i = 2; i < 7; ++i)
  {
    auto t = (i - 1) * delta_t;
    auto pred_nav_state = pim.predict(nav_states.back(), bias);
    graph_manager.AddFrame(i, t, pim, pred_nav_state, bias);
    nav_states.push_back(pred_nav_state);

    EXPECT_TRUE(graph_manager.CanAddObservation(1, i));

    if (added_obs < n_obs)
    {
      auto feature = gtsam::PinholeCamera<gtsam::Cal3_S2>(pred_nav_state.pose() * body_p_cam, *K).project(landmark);
      graph_manager.AddLandmarkObservation(1, i, t, feature, K, body_p_cam);
      last_observation_t = t;
    }

    graph_manager.Update();

    // We will observe the following only after the call to graph_manager.Update()
    if (last_observation_t > (t - lag))
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
