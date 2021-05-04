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

#include "graph_manager.h"
#include "helpers.h"

using gtsam::symbol_shorthand::X;

TEST(GraphManager, LandmarksWork)
{
  // Arrange
  GraphManager graph_manager((gtsam::ISAM2Params()), (gtsam::SmartProjectionParams()));
  auto noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(0.001), gtsam::Vector3::Constant(0.001)).finished());
  auto noise_v = gtsam::noiseModel::Isotropic::Sigma(3, 1.);
  auto noise_b = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
  auto feature_noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  gtsam::NavState init_nav_state(gtsam::Pose3(), gtsam::Vector3(1., 0., 0.));
  gtsam::imuBias::ConstantBias bias;

  gtsam::PreintegratedCombinedMeasurements pim(PimParams());

  // We assume constant velocity throughout for simplicity
  gtsam::Vector3 measured_acc(0., 0., 9.81);
  gtsam::Vector3 measured_omega(0., 0., 0.);
  double delta_t = 1.0;

  gtsam::Pose3 body_p_cam(gtsam::Rot3::Ypr(-M_PI_2, 0., -M_PI_2), gtsam::Point3::Zero());
  gtsam::Point3 landmark(4., 1., 1.);
  auto K = gtsam::make_shared<gtsam::Cal3_S2>(650., 650., 0., 200., 200.);

  // Act
  graph_manager.SetInitNavstate(1, init_nav_state, bias, noise_x, noise_v, noise_b);

  auto first_feature = gtsam::PinholeCamera<gtsam::Cal3_S2>(init_nav_state.pose() * body_p_cam, *K).project(landmark);
  graph_manager.InitStructurelessLandmark(1, 1, first_feature, feature_noise, K, body_p_cam);

  std::vector<gtsam::NavState> gt_nav_states = {init_nav_state};

  for (int i = 2; i < 4; ++i)
  {
    pim.integrateMeasurement(measured_acc, measured_omega, delta_t);
    auto pred_nav_state = pim.predict(gt_nav_states.back(), bias);
    graph_manager.AddFrame(i, pim, pred_nav_state, bias);
    pim.resetIntegration();

    gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pred_nav_state.pose() * body_p_cam, *K);
    auto feature = camera.project(landmark);
    graph_manager.AddLandmarkObservation(1, i, feature, feature_noise, K, body_p_cam);

    gt_nav_states.push_back(pred_nav_state); // We let the IMU govern the ground truth
  }

  auto isam_result = graph_manager.Update();

  // Assert
  auto pose_estimate = graph_manager.GetPose(1);
  auto pose_estimate_2 = graph_manager.GetPose(2);
  auto landmark_estimate = graph_manager.GetLandmark(1);
  ASSERT_TRUE(gtsam::assert_equal(pose_estimate, gt_nav_states[0].pose()));
  ASSERT_TRUE(gtsam::assert_equal(pose_estimate_2, gt_nav_states[1].pose()));
  ASSERT_TRUE(gtsam::assert_equal(graph_manager.GetValues().at<gtsam::Pose3>(X(1)), pose_estimate));
  ASSERT_TRUE(landmark_estimate.is_initialized());
  ASSERT_TRUE(gtsam::assert_equal(*landmark_estimate, landmark));
}
