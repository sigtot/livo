#ifndef ORB_TEST_SRC_ISAM2_SOLVER_H_
#define ORB_TEST_SRC_ISAM2_SOLVER_H_

#include "incremental_solver.h"
#include <memory>

namespace gtsam
{
class ISAM2Params;
class ISAM2;
}  // namespace gtsam

class ISAM2Solver : public IncrementalSolver
{
private:
  std::shared_ptr<gtsam::ISAM2> isam2_;

public:
  explicit ISAM2Solver(const gtsam::ISAM2Params& isam2_params);
  gtsam::ISAM2Result Update(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
                            const gtsam::KeyTimestampMap& _) override;
  gtsam::Values CalculateEstimate() override;
  bool ValueExists(gtsam::Key key) override;
  gtsam::Pose3 CalculateEstimatePose3(gtsam::Key key) override;
  gtsam::Point3 CalculateEstimatePoint3(gtsam::Key key) override;
  gtsam::Vector3 CalculateEstimateVector3(gtsam::Key key) override;
  gtsam::imuBias::ConstantBias CalculateEstimateBias(gtsam::Key key) override;
};

#endif  // ORB_TEST_SRC_ISAM2_SOLVER_H_
