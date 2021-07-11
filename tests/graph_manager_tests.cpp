#include <gtest/gtest.h>
#include <gtsam/nonlinear/ISAM2Result.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include "graph_manager.h"
#include "helpers.h"
#include "isam2_solver.h"

using gtsam::symbol_shorthand::X;

TEST(GraphManager, IMUOnlyAddFrame)
{
  GraphManager graph_manager(std::make_shared<ISAM2Solver>(gtsam::ISAM2Params()), (gtsam::SmartProjectionParams()));
  auto noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
  auto noise_v = gtsam::noiseModel::Isotropic::Sigma(3, 1.);
  auto noise_b = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
  gtsam::NavState nav_state;
  gtsam::imuBias::ConstantBias bias;

  gtsam::PreintegratedCombinedMeasurements pim(PimParams());

  gtsam::Vector3 measured_acc(1.0, 0.0, 9.81);
  gtsam::Vector3 measured_omega(0.0, 0.0, 0.0);
  double delta_t = 1.0;
  pim.integrateMeasurement(measured_acc, measured_omega, delta_t);

  graph_manager.SetInitNavstate(1, 0., nav_state, bias, noise_x, noise_v, noise_b);
  graph_manager.AddFrame(2, delta_t, pim, pim.predict(nav_state, bias), bias);

  ASSERT_FALSE(graph_manager.IsFrameTracked(1));
  ASSERT_TRUE(graph_manager.CanAddObservationsForFrame(1, 0.));

  auto isam_result = graph_manager.Update();

  auto pose = *graph_manager.GetPose(1);
  auto pose2 = *graph_manager.GetPose(2);
  auto vel2 = graph_manager.GetVelocity(2);
  ASSERT_TRUE(gtsam::assert_equal(pose, gtsam::Pose3()));
  ASSERT_TRUE(gtsam::assert_equal(pose2, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.5, 0.0, 0.0))));  // x = 1/2 at^2
  ASSERT_TRUE(gtsam::assert_equal(vel2, gtsam::Vector3(1.0, 0.0, 0.0)));                               // v = at
  ASSERT_TRUE(gtsam::assert_equal(graph_manager.GetValues().at<gtsam::Pose3>(X(1)), pose));
  ASSERT_TRUE(gtsam::assert_equal(graph_manager.GetNavState(1), nav_state));
  ASSERT_TRUE(graph_manager.IsFrameTracked(1));
  ASSERT_TRUE(graph_manager.IsFrameTracked(2));
  ASSERT_FALSE(graph_manager.IsFrameTracked(3));
}

TEST(GraphManager, Betweenfactor)
{
  gtsam::ISAM2Params isam2_params;
  isam2_params.relinearizeThreshold = 1e-5;
  isam2_params.relinearizeSkip = 1; // Must set this to 1 otherwise isam will only linearize on the first update
  GraphManager graph_manager(std::make_shared<ISAM2Solver>(isam2_params), (gtsam::SmartProjectionParams()));
  auto noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(0.0001), gtsam::Vector3::Constant(0.0001)).finished());
  auto noise_v = gtsam::noiseModel::Isotropic::Sigma(3, 0.0001);
  auto noise_b = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(0.0001), gtsam::Vector3::Constant(0.0001)).finished());
  auto noise_between = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(0.0000000001), gtsam::Vector3::Constant(0.00000001)).finished());
  gtsam::NavState nav_state;
  gtsam::imuBias::ConstantBias bias;
  gtsam::Pose3 gt_pose_2(gtsam::Rot3(), gtsam::Point3(0.5, 0.0, 0.0));
  auto gt_nav_state_2 = gtsam::NavState(gt_pose_2, gtsam::Vector3());

  gtsam::PreintegratedCombinedMeasurements pim(PimParams());

  gtsam::Vector3 measured_acc(1.01, 0.0, 9.81); // Measured acc slightly off (1.01 instead of 1.0)
  gtsam::Vector3 measured_omega(0.0, 0.0, 0.0);
  double delta_t = 1.0;
  pim.integrateMeasurement(measured_acc, measured_omega, delta_t);

  gtsam::NavState corrupted_nav_state_2(
      gt_nav_state_2.pose().compose(gtsam::Pose3(gtsam::Rot3::Ypr(0.1, 0.2, 0.1), gtsam::Point3(0.1, 0.1, -0.1))),
      gt_nav_state_2.velocity());

  graph_manager.SetInitNavstate(1, 0., nav_state, bias, noise_x, noise_v, noise_b);

  graph_manager.Update();

  graph_manager.AddFrame(2, delta_t, pim, corrupted_nav_state_2, bias);
  graph_manager.AddBetweenFactor(1, 2, nav_state.pose().between(gt_nav_state_2.pose()), noise_between);

  graph_manager.Update();
  graph_manager.Update();
  graph_manager.Update();

  auto pose = *graph_manager.GetPose(1);
  auto pose2 = *graph_manager.GetPose(2);
  auto vel2 = graph_manager.GetVelocity(2);
  EXPECT_TRUE(gtsam::assert_equal(pose, gtsam::Pose3(), 1e-5));
  EXPECT_TRUE(gtsam::assert_equal(pose2, gt_pose_2, 1e-5));
  EXPECT_TRUE(gtsam::assert_equal(vel2, gtsam::Vector3(1.0, 0.0, 0.0), 1e-2));
}
