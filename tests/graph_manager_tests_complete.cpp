#include <gtest/gtest.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/ISAM2Result.h>
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
#include "landmark_result_gtsam.h"
#include "isam2_solver.h"
#include "incremental_fixed_lag_solver.h"

using gtsam::symbol_shorthand::X;

class GraphManagerTestComplete : public ::testing::Test
{
protected:
  GraphManagerTestComplete() = default;
  ~GraphManagerTestComplete() override = default;

  static std::shared_ptr<IncrementalSolver> GetIncrementalSolver(bool fixed_lag, const gtsam::ISAM2Params& isam2_params,
                                                                 double lag)
  {
    if (fixed_lag)
    {
      return std::make_shared<IncrementalFixedLagSolver>(lag, isam2_params);
    }
    return std::make_shared<ISAM2Solver>(isam2_params);
  }

  void SetUp(bool fixed_lag, const gtsam::ISAM2Params& isam2_params = gtsam::ISAM2Params(), double lag = -1.0)
  {
    graph_manager = std::make_shared<GraphManager>(GetIncrementalSolver(fixed_lag, isam2_params, lag),
                                                   gtsam::SmartProjectionParams(), lag);
    noise_x = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector3::Constant(0.001), gtsam::Vector3::Constant(0.001)).finished());
    noise_v = gtsam::noiseModel::Isotropic::Sigma(3, 0.5);
    noise_b = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
    feature_noise = gtsam::noiseModel::Isotropic::Sigma(2, 0.2);
    feature_m_estimator = gtsam::noiseModel::mEstimator::Huber::Create(15);

    init_nav_state = gtsam::NavState(gtsam::Pose3(), gtsam::Vector3(1., 0., 0.));

    pim = PimParams();

    // We assume constant velocity throughout for simplicity
    measured_acc = gtsam::Vector3(0., 0., 9.81);
    measured_omega = gtsam::Vector3(0., 0., 0.);
    delta_t = 1.0;

    body_p_cam = gtsam::Pose3(gtsam::Rot3::Ypr(-M_PI_2, 0., -M_PI_2), gtsam::Point3::Zero());
    landmarks = { gtsam::Point3(8., 1., 1.), gtsam::Point3(9.0, -2., 0.5), gtsam::Point3(8.0, 0., 3.) };
    K = gtsam::make_shared<gtsam::Cal3_S2>(650., 650., 0., 200., 200.);

    // Offsets for corrupting ground truth. Can't be too large, or the result will deviate more than 1e-2 from gt
    gtsam::Point3 offset(0.1, -0.1, 0.1);

    // Act
    graph_manager->SetInitNavstate(1, 0., init_nav_state, bias, noise_x, noise_v, noise_b);

    std::vector<gtsam::Point2> first_features;
    for (const auto& landmark : landmarks)
    {
      auto feature = gtsam::PinholeCamera<gtsam::Cal3_S2>(init_nav_state.pose() * body_p_cam, *K).project(landmark);
      first_features.push_back(feature);
    }

    // Lmk 1 will be kept as a smart factor, as if no range measurements exist for it
    graph_manager->InitStructurelessLandmark(1, 1, 0., first_features[0], K, body_p_cam, feature_noise,
                                             feature_m_estimator);

    // Lmk 2 will be initialized as a smart factor, but then turned into a proj factor, as if range becomes available
    graph_manager->InitStructurelessLandmark(2, 1, 0., first_features[1], K, body_p_cam, feature_noise,
                                             feature_m_estimator);

    // Lmk 3 will be initialized as a proj factor from the beginning
    graph_manager->InitProjectionLandmark(3, 1, 0., first_features[2], landmarks[2] + offset, K, body_p_cam,
                                          feature_noise, feature_m_estimator);

    range_noise = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
    robust_range_noise = gtsam::noiseModel::Robust::Create(feature_m_estimator, range_noise);

    gt_nav_states.push_back(init_nav_state);

    for (int i = 2; i < 4; ++i)
    {
      pim.integrateMeasurement(measured_acc, measured_omega, delta_t);
      auto pred_nav_state = pim.predict(gt_nav_states.back(), bias);
      graph_manager->AddFrame(i, (i - 1) * delta_t, pim, pred_nav_state, bias);
      pim.resetIntegration();

      for (int j = 0; j < landmarks.size(); ++j)
      {
        gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pred_nav_state.pose() * body_p_cam, *K);
        auto feature = camera.project(landmarks[j]);
        graph_manager->AddLandmarkObservation(j + 1, i, (i - 1) * delta_t, feature, K, body_p_cam);
      }

      gt_nav_states.push_back(pred_nav_state);  // We let the IMU govern the ground truth
    }

    auto isam_result = graph_manager->Update();
  }

  void RunIncrementalUpdates()
  {
    for (int i = 4; i < 7; ++i)
    {
      pim.integrateMeasurement(measured_acc, measured_omega, delta_t);
      auto pred_nav_state = pim.predict(gt_nav_states.back(), bias);
      graph_manager->AddFrame(i, (i - 1) * delta_t, pim, pred_nav_state, bias);
      pim.resetIntegration();

      if (i == 5)
      {
        // Scenario: At i=5, a point cloud comes in, providing range measurements for landmarks 2, 3 and 4, but not 1.
        // Lmk 2 is a smart factor to begin with, so will be converted to a proj factor
        ASSERT_FALSE(graph_manager->CanAddRangeObservation(2, i));
        graph_manager->ConvertSmartFactorToProjectionFactor(2, (i - 1) * delta_t, landmarks[1]);
        ASSERT_TRUE(graph_manager->CanAddRangeObservation(2, i));
        graph_manager->AddRangeObservation(2, i, (i - 1) * delta_t, pred_nav_state.pose().range(landmarks[1]), range_noise);

        // Lmk 3 is already a proj factor, so this just adds a range measurement to it
        ASSERT_TRUE(graph_manager->CanAddRangeObservation(3, i));
        graph_manager->AddRangeObservation(3, i, (i - 1) * delta_t, pred_nav_state.pose().range(landmarks[2]), robust_range_noise);

        // Lmk 4 is a smart factor, and is currently degenerate. This makes no difference.
        ASSERT_FALSE(graph_manager->CanAddRangeObservation(4, i));
        graph_manager->ConvertSmartFactorToProjectionFactor(4, (i - 1) * delta_t, landmarks[3]);
        ASSERT_TRUE(graph_manager->CanAddRangeObservation(4, i));
        graph_manager->AddRangeObservation(4, i, (i - 1) * delta_t, pred_nav_state.pose().range(landmarks[3]), range_noise);
      }

      for (int j = 0; j < landmarks.size(); ++j)
      {
        gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pred_nav_state.pose() * body_p_cam, *K);
        auto feature = camera.project(landmarks[j]);
        graph_manager->AddLandmarkObservation(j + 1, i, (i - 1) * delta_t, feature, K, body_p_cam);
      }

      if (i == 4)
      {
        // Scenario: At i=4, we obtain a new landmark (lmk 4), we immediately add it as a smart factor.
        // Because it only has this one observation, it will be flagged as degenerate until more observations are added
        landmarks.emplace_back(10.0, 1.5, 1.0);
        gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pred_nav_state.pose() * body_p_cam, *K);
        auto feature = camera.project(landmarks.back());
        graph_manager->InitStructurelessLandmark(static_cast<int>(landmarks.size()), i, (i - 1) * delta_t, feature, K,
                                                 body_p_cam, feature_noise);
      }

      gt_nav_states.push_back(pred_nav_state);  // We let the IMU govern the ground truth

      graph_manager->Update();
    }
  }

  std::shared_ptr<GraphManager> graph_manager;
  boost::shared_ptr<gtsam::noiseModel::Diagonal> noise_x;
  boost::shared_ptr<gtsam::noiseModel::Isotropic> noise_v;
  boost::shared_ptr<gtsam::noiseModel::Diagonal> noise_b;
  boost::shared_ptr<gtsam::noiseModel::Isotropic> feature_noise;
  boost::shared_ptr<gtsam::noiseModel::Isotropic> range_noise;
  boost::shared_ptr<gtsam::noiseModel::mEstimator::Base> feature_m_estimator;
  boost::shared_ptr<gtsam::noiseModel::Robust::Base> robust_range_noise;
  gtsam::NavState init_nav_state;
  gtsam::imuBias::ConstantBias bias;
  gtsam::PreintegratedCombinedMeasurements pim;

  gtsam::Vector3 measured_acc;
  gtsam::Vector3 measured_omega;
  double delta_t = 1.0;

  gtsam::Pose3 body_p_cam;
  std::vector<gtsam::Point3> landmarks;
  std::vector<gtsam::NavState> gt_nav_states;
  boost::shared_ptr<gtsam::Cal3_S2> K;
};

TEST_F(GraphManagerTestComplete, RegularISAM2)
{
  SetUp(false);

  RunIncrementalUpdates();

  // No fixed lag, we should be able to add observations to all landmarks
  pim.integrateMeasurement(measured_acc, measured_omega, delta_t);
  auto pred_nav_state = pim.predict(gt_nav_states.back(), bias);
  graph_manager->AddFrame(7, 6 * delta_t, pim, pred_nav_state, bias);
  pim.resetIntegration();
  graph_manager->Update();

  for (int i = 1; i <= 4; ++i)
  {
    EXPECT_TRUE(graph_manager->CanAddObservationsForFrame(i, (i - 1) * delta_t));
  }

  for (int i = 0; i < gt_nav_states.size(); ++i)
  {
    EXPECT_TRUE(gtsam::assert_equal(graph_manager->GetPose(i + 1), gt_nav_states[i].pose(), 1e-2));
    EXPECT_TRUE(
        gtsam::assert_equal(graph_manager->GetValues().at<gtsam::Pose3>(X(i + 1)), gt_nav_states[i].pose(), 1e-2));
  }

  std::vector<LandmarkType> expected_landmark_types{ SmartFactorType, ProjectionFactorType, ProjectionFactorType,
                                                     ProjectionFactorType };
  auto landmark_estimates = graph_manager->GetLandmarks();
  for (int j = 0; j < landmarks.size(); ++j)
  {
    auto landmark_estimate = graph_manager->GetLandmark(j + 1);
    EXPECT_TRUE(landmark_estimate);
    EXPECT_TRUE(gtsam::assert_equal((*landmark_estimate).pt, landmarks[j], 1e-2));
    EXPECT_EQ((*landmark_estimate).type, expected_landmark_types[j]);

    EXPECT_TRUE(landmark_estimates[j + 1]);
    EXPECT_TRUE(gtsam::assert_equal((*landmark_estimates[j + 1]).pt, landmarks[j], 1e-2));
    EXPECT_TRUE(graph_manager->IsLandmarkTracked(j + 1));
  }
  EXPECT_FALSE(graph_manager->IsLandmarkTracked(99));
}

TEST_F(GraphManagerTestComplete, FixedLag)
{
  SetUp(true, gtsam::ISAM2Params(), 5.5);

  RunIncrementalUpdates();

  // Add one more frame, which will trigger
  pim.integrateMeasurement(measured_acc, measured_omega, delta_t);
  auto pred_nav_state = pim.predict(gt_nav_states.back(), bias);
  graph_manager->AddFrame(7, 6 * delta_t, pim, pred_nav_state, bias);
  pim.resetIntegration();
  graph_manager->Update();

  // First frame is marginalized, so cannot add to it
  EXPECT_FALSE(graph_manager->CanAddObservationsForFrame(1, 0 * delta_t));
  for (int i = 2; i <= 7; ++i)
  {
    // All other frames are still in the window
    EXPECT_TRUE(graph_manager->CanAddObservationsForFrame(i, (i - 1) * delta_t));
  }

  for (int i = 1; i <= 3; ++i)
  {
    // First landmarks are marginalized, so we cannot add observations
    EXPECT_FALSE(graph_manager->CanAddObservation(i, 7));
  }
  EXPECT_TRUE(graph_manager->CanAddObservation(4, 7));  // lmk 4 was added later than the rest, so still in window

  for (int i = 1; i < gt_nav_states.size(); ++i)
  {
    EXPECT_TRUE(gtsam::assert_equal(graph_manager->GetPose(i + 1), gt_nav_states[i].pose(), 0.1));
    EXPECT_TRUE(
        gtsam::assert_equal(graph_manager->GetValues().at<gtsam::Pose3>(X(i + 1)), gt_nav_states[i].pose(), 0.1));
  }

  std::vector<LandmarkType> expected_landmark_types{ SmartFactorType, ProjectionFactorType, ProjectionFactorType,
                                                     ProjectionFactorType };
  auto landmark_estimates = graph_manager->GetLandmarks();
  EXPECT_EQ(landmark_estimates.size(), 1);
  for (int j = 1; j <= 3; ++j)
  {
    EXPECT_FALSE(graph_manager->IsLandmarkTracked(j));
  }
  EXPECT_TRUE(gtsam::assert_equal(graph_manager->GetLandmark(4)->pt, landmarks[3], 1e-2));
  EXPECT_TRUE(graph_manager->IsLandmarkTracked(4));
  EXPECT_FALSE(graph_manager->IsLandmarkTracked(99));
}
