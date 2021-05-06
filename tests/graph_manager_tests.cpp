#include <gtest/gtest.h>
#include <gtsam/nonlinear/ISAM2.h>
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

using gtsam::symbol_shorthand::X;

TEST(GraphManager, IMUOnlyAddFrame)
{
  // Arrange
  GraphManager graph_manager((gtsam::ISAM2Params()), (gtsam::SmartProjectionParams()));
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

  // Act
  graph_manager.SetInitNavstate(1, nav_state, bias, noise_x, noise_v, noise_b);
  graph_manager.AddFrame(2, pim, pim.predict(nav_state, bias), bias);

  auto isam_result = graph_manager.Update();

  // Assert
  auto pose = graph_manager.GetPose(1);
  auto pose2 = graph_manager.GetPose(2);
  auto vel2 = graph_manager.GetVelocity(2);
  ASSERT_TRUE(gtsam::assert_equal(pose, gtsam::Pose3()));
  ASSERT_TRUE(gtsam::assert_equal(pose2, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.5, 0.0, 0.0))));  // x = 1/2 at^2
  ASSERT_TRUE(gtsam::assert_equal(vel2, gtsam::Vector3(1.0, 0.0, 0.0)));                               // v = at
  ASSERT_TRUE(gtsam::assert_equal(graph_manager.GetValues().at<gtsam::Pose3>(X(1)), pose));
  ASSERT_TRUE(gtsam::assert_equal(graph_manager.GetNavState(1), nav_state));
}
