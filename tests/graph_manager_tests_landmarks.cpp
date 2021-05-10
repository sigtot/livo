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
    noise_v = gtsam::noiseModel::Isotropic::Sigma(3, 0.5);
    noise_b = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
    feature_noise = gtsam::noiseModel::Isotropic::Sigma(2, 0.2);
    feature_m_estimator = gtsam::noiseModel::mEstimator::Huber::Create(15);
    robust_noise = gtsam::noiseModel::Robust::Create(feature_m_estimator, feature_noise);

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
  boost::shared_ptr<gtsam::noiseModel::mEstimator::Base> feature_m_estimator;
  boost::shared_ptr<gtsam::noiseModel::Robust::Base> robust_noise;
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
  graph_manager->InitStructurelessLandmark(1, 1, first_feature, K, body_p_cam, feature_noise);

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

TEST_F(GraphManagerTest, GetDegenerateSmartFactor)
{
  // Arrange
  SetUp(gtsam::ISAM2Params());

  // Act
  graph_manager->SetInitNavstate(1, init_nav_state, bias, noise_x, noise_v, noise_b);

  auto first_feature = gtsam::PinholeCamera<gtsam::Cal3_S2>(init_nav_state.pose() * body_p_cam, *K).project(landmark);
  graph_manager->InitStructurelessLandmark(1, 1, first_feature, K, body_p_cam, feature_noise);

  std::vector<gtsam::NavState> gt_nav_states = { init_nav_state };

  auto isam_result = graph_manager->Update();

  EXPECT_TRUE(gtsam::assert_equal(graph_manager->GetPose(1), gt_nav_states[0].pose(), 1e-2));
  EXPECT_EQ(graph_manager->GetLandmark(1), boost::none);
}

TEST_F(GraphManagerTest, ProjectionLandmarks)
{
  // Arrange
  SetUp(gtsam::ISAM2Params());
  gtsam::Point3 offset(0.1, -0.1, 0.1);

  // Act
  graph_manager->SetInitNavstate(1, init_nav_state, bias, noise_x, noise_v, noise_b);

  auto first_feature = gtsam::PinholeCamera<gtsam::Cal3_S2>(init_nav_state.pose() * body_p_cam, *K).project(landmark);
  graph_manager->InitProjectionLandmark(1, 1, first_feature, landmark + offset, K, body_p_cam, feature_noise);

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
  graph_manager->InitStructurelessLandmark(1, 1, first_feature, K, body_p_cam, feature_noise);

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

TEST_F(GraphManagerTest, RangeFactors)
{
  // Arrange
  gtsam::ISAM2Params isam2_params;
  isam2_params.optimizationParams = gtsam::ISAM2DoglegParams();
  SetUp(isam2_params);
  std::vector<gtsam::Point3> landmarks = { landmark, gtsam::Point3(9.0, -2., 0.5), gtsam::Point3(8.0, 0., 3.) };

  // Offsets for corrupting ground truth. Can't be too large, or the result will deviate more than 1e-2 from gt
  gtsam::Point3 offset(0.1, -0.1, 0.1);

  // Act
  graph_manager->SetInitNavstate(1, init_nav_state, bias, noise_x, noise_v, noise_b);

  std::vector<gtsam::Point2> first_features;
  for (const auto& landmark : landmarks)
  {
    auto feature = gtsam::PinholeCamera<gtsam::Cal3_S2>(init_nav_state.pose() * body_p_cam, *K).project(landmark);
    first_features.push_back(feature);
  }

  // Lmk 1 will be kept as a smart factor, as if no range measurements exist for it
  graph_manager->InitStructurelessLandmark(1, 1, first_features[0], K, body_p_cam, feature_noise, feature_m_estimator);

  // Lmk 2 will be initialized as a smart factor, but then turned into a proj factor, as if range becomes available
  graph_manager->InitStructurelessLandmark(2, 1, first_features[1], K, body_p_cam, feature_noise, feature_m_estimator);

  // Lmk 3 will be initialized as a proj factor from the beginning
  graph_manager->InitProjectionLandmark(3, 1, first_features[2], landmarks[2] + offset, K, body_p_cam, feature_noise,
                                        feature_m_estimator);

  auto range_noise = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
  auto robust_range_noise = gtsam::noiseModel::Robust::Create(feature_m_estimator, range_noise);

  std::vector<gtsam::NavState> gt_nav_states = { init_nav_state };

  for (int i = 2; i < 4; ++i)
  {
    pim.integrateMeasurement(measured_acc, measured_omega, delta_t);
    auto pred_nav_state = pim.predict(gt_nav_states.back(), bias);
    graph_manager->AddFrame(i, pim, pred_nav_state, bias);
    pim.resetIntegration();

    for (int j = 0; j < landmarks.size(); ++j)
    {
      gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pred_nav_state.pose() * body_p_cam, *K);
      auto feature = camera.project(landmarks[j]);
      graph_manager->AddLandmarkObservation(j + 1, i, feature, K, body_p_cam);
    }

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

    if (i == 5)
    {
      // Scenario: At i=5, a point cloud comes in, providing range measurements for landmarks 2, 3 and 4, but not 1.
      // Lmk 2 is a smart factor to begin with, so will be converted to a proj factor
      graph_manager->AddRangeObservation(2, i, pred_nav_state.pose().range(landmarks[1]), range_noise);
      // Lmk 3 is already a proj factor, so this just adds a range measurement to it
      graph_manager->AddRangeObservation(3, i, pred_nav_state.pose().range(landmarks[2]), robust_range_noise);
      // Lmk 4 is a smart factor, but is currently degenerate, and so cannot give us an initial point3
      // This must be obtained from the first smart factor measurement and the given range instead.
      graph_manager->AddRangeObservation(4, i, pred_nav_state.pose().range(landmarks[3]), range_noise);
    }

    for (int j = 0; j < landmarks.size(); ++j)
    {
      gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pred_nav_state.pose() * body_p_cam, *K);
      auto feature = camera.project(landmarks[j]);
      graph_manager->AddLandmarkObservation(j + 1, i, feature, K, body_p_cam);
    }

    if (i == 4)
    {
      // Scenario: At i=4, we obtain a new landmark (lmk 4), we immediately add it as a smart factor.
      // Because it only has this one observation, it will be flagged as degenerate until more observations are added
      landmarks.emplace_back(10.0, 1.5, 1.0);
      gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pred_nav_state.pose() * body_p_cam, *K);
      auto feature = camera.project(landmarks.back());
      graph_manager->InitStructurelessLandmark(static_cast<int>(landmarks.size()), i, feature, K, body_p_cam,
                                               feature_noise);
    }

    gt_nav_states.push_back(pred_nav_state);  // We let the IMU govern the ground truth

    graph_manager->Update();
  }

  // Assert
  for (int i = 0; i < gt_nav_states.size(); ++i)
  {
    EXPECT_TRUE(gtsam::assert_equal(graph_manager->GetPose(i + 1), gt_nav_states[i].pose(), 0.15));
    EXPECT_TRUE(
        gtsam::assert_equal(graph_manager->GetValues().at<gtsam::Pose3>(X(i + 1)), gt_nav_states[i].pose(), 0.15));
  }

  auto landmark_estimates = graph_manager->GetLandmarks();
  for (int j = 0; j < landmarks.size(); ++j)
  {
    auto landmark_estimate = graph_manager->GetLandmark(j + 1);
    EXPECT_TRUE(landmark_estimate);
    EXPECT_TRUE(gtsam::assert_equal(*landmark_estimate, landmarks[j], 0.15));

    EXPECT_TRUE(landmark_estimates[j + 1]);
    EXPECT_TRUE(gtsam::assert_equal(*landmark_estimates[j + 1], landmarks[j], 0.15));
    EXPECT_TRUE(graph_manager->IsLandmarkTracked(j + 1));
  }
  EXPECT_FALSE(graph_manager->IsLandmarkTracked(99));
}
