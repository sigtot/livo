#include "incremental_fixed_lag_solver.h"
#include "incremental_fixed_lag_smoother_patched.h"

#include <gtsam/nonlinear/ISAM2Result.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/navigation/ImuBias.h>

IncrementalFixedLagSolver::IncrementalFixedLagSolver(double lag, const gtsam::ISAM2Params& isam2_params)
  : fixed_lag_smoother_(std::make_shared<IncrementalFixedLagSmootherPatched>(lag, isam2_params))
{
}

gtsam::ISAM2Result IncrementalFixedLagSolver::Update(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values, const gtsam::KeyTimestampMap& timestamps)
{
  return Update(graph, values, timestamps, boost::none);
}

gtsam::ISAM2Result IncrementalFixedLagSolver::Update(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values, const gtsam::KeyTimestampMap& timestamps,
    const boost::optional<gtsam::FastMap<gtsam::FactorIndex, gtsam::FastSet<gtsam::Key>>>& newAffectedKeys)
{
  fixed_lag_smoother_->update(graph, values, timestamps, newAffectedKeys);
  return fixed_lag_smoother_->getISAM2Result();
}

gtsam::Values IncrementalFixedLagSolver::CalculateEstimate()
{
  return fixed_lag_smoother_->calculateEstimate();
}

bool IncrementalFixedLagSolver::ValueExists(gtsam::Key key)
{
  auto timestamp_map = fixed_lag_smoother_->timestamps();
  return timestamp_map.count(key);
}

gtsam::Pose3 IncrementalFixedLagSolver::CalculateEstimatePose3(gtsam::Key key)
{
  return fixed_lag_smoother_->calculateEstimate<gtsam::Pose3>(key);
}

gtsam::Point3 IncrementalFixedLagSolver::CalculateEstimatePoint3(gtsam::Key key)
{
  return fixed_lag_smoother_->calculateEstimate<gtsam::Point3>(key);
}

gtsam::Vector3 IncrementalFixedLagSolver::CalculateEstimateVector3(gtsam::Key key)
{
  return fixed_lag_smoother_->calculateEstimate<gtsam::Vector3>(key);
}

gtsam::imuBias::ConstantBias IncrementalFixedLagSolver::CalculateEstimateBias(gtsam::Key key)
{
  return fixed_lag_smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(key);
}
