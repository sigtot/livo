#include <gtest/gtest.h>

#include "incremental_fixed_lag_smoother_patched.h"
#include "helpers.h"

#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/geometry/Cal3_S2.h>

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;

TEST(IFLPatchedTest, CanMarginalizeSmartFactor)
{
  double lag = 4.5;
  gtsam::ISAM2Params isam2_params;
  isam2_params.findUnusedFactorSlots = true;
  auto smoother = IncrementalFixedLagSmootherPatched(lag, isam2_params);


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
    smoother.update(graph, values, timestamps, smart_factor_new_affected_keys);
    graph.resize(0);
    values.clear();
    timestamps.clear();
  }
}
