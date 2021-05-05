#include <gtest/gtest.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <vector>
#include <memory>

#include "graph_manager.h"
#include "helpers.h"

using gtsam::symbol_shorthand::X;

class GraphManagerTest : public ::testing::Test
{
protected:
  GraphManagerTest() = default;
  ~GraphManagerTest() override = default;
  void SetUp(const gtsam::ISAM2Params& isam2_params)
  {
    graph_manager = std::make_shared<GraphManager>(isam2_params, (gtsam::SmartProjectionParams()));
    noise_x = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector3::Constant(0.001), gtsam::Vector3::Constant(0.001)).finished());
    noise_v = gtsam::noiseModel::Isotropic::Sigma(3, 1.);
    noise_b = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
    feature_noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

    init_nav_state = gtsam::NavState(gtsam::Pose3(), gtsam::Vector3(1., 0., 0.));

    pim = PimParams();

    // We assume constant velocity throughout for simplicity
    measured_acc = gtsam::Vector3(0., 0., 9.81);
    measured_omega = gtsam::Vector3(0., 0., 0.);
    delta_t = 1.0;

    body_p_cam = gtsam::Pose3(gtsam::Rot3::Ypr(-M_PI_2, 0., -M_PI_2), gtsam::Point3::Zero());
    landmark = gtsam::Point3(8., 1., 1.);
    K = gtsam::make_shared<gtsam::Cal3_S2>(650., 650., 0., 200., 200.);
  }
  std::shared_ptr<GraphManager> graph_manager;
  boost::shared_ptr<gtsam::noiseModel::Diagonal> noise_x;
  boost::shared_ptr<gtsam::noiseModel::Isotropic> noise_v;
  boost::shared_ptr<gtsam::noiseModel::Diagonal> noise_b;
  boost::shared_ptr<gtsam::noiseModel::Isotropic> feature_noise;
  gtsam::NavState init_nav_state;
  gtsam::imuBias::ConstantBias bias;
  gtsam::PreintegratedCombinedMeasurements pim;

  gtsam::Vector3 measured_acc;
  gtsam::Vector3 measured_omega;
  double delta_t = 1.0;

  gtsam::Pose3 body_p_cam;
  gtsam::Point3 landmark;
  boost::shared_ptr<gtsam::Cal3_S2> K;
};

TEST_F(GraphManagerTest, SmartFactors)
{
  // Arrange
  SetUp(gtsam::ISAM2Params());

  // Act
  graph_manager->SetInitNavstate(1, init_nav_state, bias, noise_x, noise_v, noise_b);

  auto first_feature = gtsam::PinholeCamera<gtsam::Cal3_S2>(init_nav_state.pose() * body_p_cam, *K).project(landmark);
  graph_manager->InitStructurelessLandmark(1, 1, first_feature, feature_noise, K, body_p_cam);

  std::vector<gtsam::NavState> gt_nav_states = { init_nav_state };

  for (int i = 2; i < 4; ++i)
  {
    pim.integrateMeasurement(measured_acc, measured_omega, delta_t);
    auto pred_nav_state = pim.predict(gt_nav_states.back(), bias);
    graph_manager->AddFrame(i, pim, pred_nav_state, bias);
    pim.resetIntegration();

    gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pred_nav_state.pose() * body_p_cam, *K);
    auto feature = camera.project(landmark);
    graph_manager->AddLandmarkObservation(1, i, feature, K, body_p_cam);

    gt_nav_states.push_back(pred_nav_state);  // We let the IMU govern the ground truth
  }

  auto isam_result = graph_manager->Update();

  // Perform a few incremental updates
  for (int i = 4; i < 7; ++i)
  {
    pim.integrateMeasurement(measured_acc, measured_omega, delta_t);
    auto pred_nav_state = pim.predict(gt_nav_states.back(), bias);
    graph_manager->AddFrame(i, pim, pred_nav_state, bias);
    pim.resetIntegration();

    gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pred_nav_state.pose() * body_p_cam, *K);
    auto feature = camera.project(landmark);
    graph_manager->AddLandmarkObservation(1, i, feature, K, body_p_cam);

    gt_nav_states.push_back(pred_nav_state);  // We let the IMU govern the ground truth

    graph_manager->Update();
  }

  // Assert
  for (int i = 0; i < gt_nav_states.size(); ++i)
  {
    EXPECT_TRUE(gtsam::assert_equal(graph_manager->GetPose(i + 1), gt_nav_states[i].pose(), 1e-2));
    EXPECT_TRUE(
        gtsam::assert_equal(graph_manager->GetValues().at<gtsam::Pose3>(X(i + 1)), gt_nav_states[i].pose(), 1e-2));
  }

  auto landmark_estimate = graph_manager->GetLandmark(1);
  EXPECT_TRUE(landmark_estimate);
  EXPECT_TRUE(gtsam::assert_equal(*landmark_estimate, landmark));
}

TEST_F(GraphManagerTest, ProjectionLandmarks)
{
  // Arrange
  SetUp(gtsam::ISAM2Params());
  gtsam::Point3 offset(0.1, -0.1, 0.1);

  // Act
  graph_manager->SetInitNavstate(1, init_nav_state, bias, noise_x, noise_v, noise_b);

  auto first_feature = gtsam::PinholeCamera<gtsam::Cal3_S2>(init_nav_state.pose() * body_p_cam, *K).project(landmark);
  graph_manager->InitProjectionLandmark(1, 1, first_feature, landmark + offset, feature_noise, K, body_p_cam);

  std::vector<gtsam::NavState> gt_nav_states = { init_nav_state };

  for (int i = 2; i < 4; ++i)
  {
    pim.integrateMeasurement(measured_acc, measured_omega, delta_t);
    auto pred_nav_state = pim.predict(gt_nav_states.back(), bias);
    graph_manager->AddFrame(i, pim, pred_nav_state, bias);
    pim.resetIntegration();

    gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pred_nav_state.pose() * body_p_cam, *K);
    auto feature = camera.project(landmark);
    graph_manager->AddLandmarkObservation(1, i, feature, K, body_p_cam);

    gt_nav_states.push_back(pred_nav_state);  // We let the IMU govern the ground truth
  }

  graph_manager->Update();

  // Perform a few incremental updates
  for (int i = 4; i < 7; ++i)
  {
    pim.integrateMeasurement(measured_acc, measured_omega, delta_t);
    auto pred_nav_state = pim.predict(gt_nav_states.back(), bias);
    graph_manager->AddFrame(i, pim, pred_nav_state, bias);
    pim.resetIntegration();

    gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pred_nav_state.pose() * body_p_cam, *K);
    auto feature = camera.project(landmark);
    graph_manager->AddLandmarkObservation(1, i, feature, K, body_p_cam);

    gt_nav_states.push_back(pred_nav_state);  // We let the IMU govern the ground truth

    graph_manager->Update();
  }

  // Assert
  for (int i = 0; i < gt_nav_states.size(); ++i)
  {
    EXPECT_TRUE(gtsam::assert_equal(graph_manager->GetPose(i + 1), gt_nav_states[i].pose(), 1e-2));
    EXPECT_TRUE(
        gtsam::assert_equal(graph_manager->GetValues().at<gtsam::Pose3>(X(i + 1)), gt_nav_states[i].pose(), 1e-2));
  }

  auto landmark_estimate = graph_manager->GetLandmark(1);
  EXPECT_TRUE(landmark_estimate);
  EXPECT_TRUE(gtsam::assert_equal(*landmark_estimate, landmark, 1e-2));
}

TEST_F(GraphManagerTest, SmartFactorsDogLeg)
{
  // Arrange
  gtsam::ISAM2Params isam2_params;
  isam2_params.optimizationParams = gtsam::ISAM2DoglegParams();
  SetUp(isam2_params);

  // Act
  graph_manager->SetInitNavstate(1, init_nav_state, bias, noise_x, noise_v, noise_b);

  auto first_feature = gtsam::PinholeCamera<gtsam::Cal3_S2>(init_nav_state.pose() * body_p_cam, *K).project(landmark);
  graph_manager->InitStructurelessLandmark(1, 1, first_feature, feature_noise, K, body_p_cam);

  std::vector<gtsam::NavState> gt_nav_states = { init_nav_state };

  for (int i = 2; i < 4; ++i)
  {
    pim.integrateMeasurement(measured_acc, measured_omega, delta_t);
    auto pred_nav_state = pim.predict(gt_nav_states.back(), bias);
    graph_manager->AddFrame(i, pim, pred_nav_state, bias);
    pim.resetIntegration();

    gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pred_nav_state.pose() * body_p_cam, *K);
    auto feature = camera.project(landmark);
    graph_manager->AddLandmarkObservation(1, i, feature, K, body_p_cam);

    gt_nav_states.push_back(pred_nav_state);  // We let the IMU govern the ground truth
  }

  auto isam_result = graph_manager->Update();

  // Perform a few incremental updates
  for (int i = 4; i < 7; ++i)
  {
    pim.integrateMeasurement(measured_acc, measured_omega, delta_t);
    auto pred_nav_state = pim.predict(gt_nav_states.back(), bias);
    graph_manager->AddFrame(i, pim, pred_nav_state, bias);
    pim.resetIntegration();

    gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pred_nav_state.pose() * body_p_cam, *K);
    auto feature = camera.project(landmark);
    graph_manager->AddLandmarkObservation(1, i, feature, K, body_p_cam);

    gt_nav_states.push_back(pred_nav_state);  // We let the IMU govern the ground truth

    graph_manager->Update();
  }

  // Assert
  for (int i = 0; i < gt_nav_states.size(); ++i)
  {
    EXPECT_TRUE(gtsam::assert_equal(graph_manager->GetPose(i + 1), gt_nav_states[i].pose(), 1e-2));
    EXPECT_TRUE(
        gtsam::assert_equal(graph_manager->GetValues().at<gtsam::Pose3>(X(i + 1)), gt_nav_states[i].pose(), 1e-2));
  }

  auto landmark_estimate = graph_manager->GetLandmark(1);
  EXPECT_TRUE(landmark_estimate);
  EXPECT_TRUE(gtsam::assert_equal(*landmark_estimate, landmark));
}
