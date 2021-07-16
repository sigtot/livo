#ifndef ORB_TEST_INCLUDE_LIVO_INCREMENTAL_SOLVER_H_
#define ORB_TEST_INCLUDE_LIVO_INCREMENTAL_SOLVER_H_

#include <map>
#include <Eigen/Core>
#include <boost/optional.hpp>
#include <gtsam/base/FastVector.h>
#include <gtsam/base/FastSet.h>

namespace gtsam
{
class NonlinearFactorGraph;
class Values;
class ISAM2Result;
class ISAM2UpdateParams;
class VectorValues;

class Pose3;

typedef Eigen::Vector3d Vector3;
typedef Vector3 Point3;
typedef std::uint64_t Key;
typedef std::map<Key, double> KeyTimestampMap;

template <typename KEY, typename VALUE>
class FastMap;

typedef uint64_t FactorIndex;
typedef FastVector<FactorIndex> FactorIndices;

namespace imuBias
{
class ConstantBias;
}
}  // namespace gtsam

class IncrementalSolver
{
public:
  virtual gtsam::ISAM2Result Update(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
                                    const gtsam::KeyTimestampMap& timestamps) = 0;

  virtual gtsam::ISAM2Result Update(
      const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values, const gtsam::KeyTimestampMap& timestamps,
      const gtsam::ISAM2UpdateParams& params) = 0;

  virtual gtsam::Values CalculateEstimate() = 0;
  virtual gtsam::NonlinearFactorGraph GetFactorsUnsafe() = 0;
  virtual gtsam::VectorValues GetDelta() = 0;

  virtual bool ValueExists(gtsam::Key key) = 0;

  // Pure virtual template functions not allowed in C++. Help?
  virtual gtsam::Pose3 CalculateEstimatePose3(gtsam::Key key) = 0;
  virtual gtsam::Point3 CalculateEstimatePoint3(gtsam::Key key) = 0;
  virtual gtsam::Vector3 CalculateEstimateVector3(gtsam::Key key) = 0;
  virtual gtsam::imuBias::ConstantBias CalculateEstimateBias(gtsam::Key key) = 0;
  virtual Eigen::MatrixXd MarginalCovariance(gtsam::Key key) = 0;
};

#endif  // ORB_TEST_INCLUDE_LIVO_INCREMENTAL_SOLVER_H_
