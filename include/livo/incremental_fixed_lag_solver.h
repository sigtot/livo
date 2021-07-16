#ifndef ORB_TEST_SRC_INCREMENTAL_FIXED_LAG_SOLVER_H_
#define ORB_TEST_SRC_INCREMENTAL_FIXED_LAG_SOLVER_H_

#include "incremental_solver.h"

#include <memory>
#include <boost/optional.hpp>

class IncrementalFixedLagSmootherPatched;

namespace gtsam
{
class ISAM2Params;

template <typename KEY, typename VALUE>
class FastMap;

typedef uint64_t FactorIndex;

template <typename VALUE>
class FastSet;
}  // namespace gtsam

class IncrementalFixedLagSolver : public IncrementalSolver
{
private:
public:
  explicit IncrementalFixedLagSolver(double lag, const gtsam::ISAM2Params& isam2_params);

private:
  std::shared_ptr<IncrementalFixedLagSmootherPatched> fixed_lag_smoother_;

public:
  gtsam::ISAM2Result Update(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
                            const gtsam::KeyTimestampMap& timestamps) override;
  gtsam::ISAM2Result Update(
      const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values, const gtsam::KeyTimestampMap& timestamps,
      const gtsam::ISAM2UpdateParams& params) override;
  gtsam::Values CalculateEstimate() override;
  gtsam::NonlinearFactorGraph GetFactorsUnsafe() override;
  gtsam::VectorValues GetDelta() override;
  bool ValueExists(gtsam::Key key) override;
  gtsam::Pose3 CalculateEstimatePose3(gtsam::Key key) override;
  gtsam::Point3 CalculateEstimatePoint3(gtsam::Key key) override;
  gtsam::Vector3 CalculateEstimateVector3(gtsam::Key key) override;
  gtsam::imuBias::ConstantBias CalculateEstimateBias(gtsam::Key key) override;
  Eigen::MatrixXd MarginalCovariance(gtsam::Key key) override;
};

#endif  // ORB_TEST_SRC_INCREMENTAL_FIXED_LAG_SOLVER_H_
