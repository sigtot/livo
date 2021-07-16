#include "isam2_solver.h"

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Result.h>
#include <gtsam/nonlinear/ISAM2UpdateParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/navigation/ImuBias.h>

ISAM2Solver::ISAM2Solver(const gtsam::ISAM2Params& isam2_params) : isam2_(std::make_shared<gtsam::ISAM2>(isam2_params))
{
}

gtsam::ISAM2Result ISAM2Solver::Update(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
                                       const gtsam::KeyTimestampMap& _)
{
  return Update(graph, values, _, gtsam::ISAM2UpdateParams());
}

gtsam::ISAM2Result ISAM2Solver::Update(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values, const gtsam::KeyTimestampMap& timestamps,
    const gtsam::ISAM2UpdateParams& params)
{
  return isam2_->update(graph, values, params);
}

gtsam::Values ISAM2Solver::CalculateEstimate()
{
  return isam2_->calculateEstimate();
}

gtsam::NonlinearFactorGraph ISAM2Solver::GetFactorsUnsafe()
{
  return isam2_->getFactorsUnsafe();
}

gtsam::VectorValues ISAM2Solver::GetDelta()
{
  return isam2_->getDelta();
}

bool ISAM2Solver::ValueExists(gtsam::Key key)
{
  return isam2_->valueExists(key);
}

gtsam::Pose3 ISAM2Solver::CalculateEstimatePose3(gtsam::Key key)
{
  return isam2_->calculateEstimate<gtsam::Pose3>(key);
}

gtsam::Point3 ISAM2Solver::CalculateEstimatePoint3(gtsam::Key key)
{
  return isam2_->calculateEstimate<gtsam::Point3>(key);
}

gtsam::Vector3 ISAM2Solver::CalculateEstimateVector3(gtsam::Key key)
{
  return isam2_->calculateEstimate<gtsam::Vector3>(key);
}

gtsam::imuBias::ConstantBias ISAM2Solver::CalculateEstimateBias(gtsam::Key key)
{
  return isam2_->calculateEstimate<gtsam::imuBias::ConstantBias>(key);
}

Eigen::MatrixXd ISAM2Solver::MarginalCovariance(gtsam::Key key)
{
  return isam2_->marginalCovariance(key);
}
