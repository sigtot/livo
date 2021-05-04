#include <gtest/gtest.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>

#include "graph_manager.h"

TEST(GraphManager, SmokeTest)
{
  GraphManager graph_manager((gtsam::ISAM2Params()));
  auto noise_x = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
  auto noise_v = gtsam::noiseModel::Isotropic::Sigma(3, 1.);
  auto noise_b = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(1.), gtsam::Vector3::Constant(1.)).finished());
  gtsam::NavState nav_state;
  gtsam::imuBias::ConstantBias bias;
  graph_manager.SetInitNavstate(1, nav_state, bias, noise_x, noise_v, noise_b);
  graph_manager.Update();
}
