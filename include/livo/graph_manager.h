#ifndef ORB_TEST_SRC_GRAPH_MANAGER_H_
#define ORB_TEST_SRC_GRAPH_MANAGER_H_

#include <map>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

namespace gtsam
{
class Cal3_S2;
class ISAM2;
class ISAM2Result;
class NonlinearFactorGraph;
class Values;
class Pose3;
class Point3;
class NavState;
class CombinedImuFactor;
class ISAM2Params;
class PreintegratedCombinedMeasurements;

class SmartProjectionParams;

template <class T>
class SmartProjectionPoseFactor;

template <class POSE, class LANDMARK, class CALIBRATION>
class GenericProjectionFactor;

template <typename A1, typename A2, typename T>
class RangeFactor;

typedef std::uint64_t FactorIndex;
typedef std::uint64_t Key;
typedef Eigen::Vector3d Vector3;

namespace noiseModel
{
class Isotropic;
class Diagonal;
}  // namespace noiseModel
namespace imuBias
{
class ConstantBias;
}
}  // namespace gtsam

typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> SmartFactor;
typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> ProjectionFactor;
typedef gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3, double> RangeFactor;

class GraphManager
{
private:
  std::map<int, boost::shared_ptr<SmartFactor>> smart_factors_;
  std::shared_ptr<gtsam::ISAM2> isam2_;
  std::shared_ptr<gtsam::Values> values_;
  std::shared_ptr<gtsam::NonlinearFactorGraph> graph_;
  int last_frame_id_ = -1;

public:
  explicit GraphManager(const gtsam::ISAM2Params& isam2_params);

public:
  void SetInitNavstate(int first_frame_id, const gtsam::NavState& nav_state, const gtsam::imuBias::ConstantBias& bias,
                       const boost::shared_ptr<gtsam::noiseModel::Diagonal>& noise_x,
                       const boost::shared_ptr<gtsam::noiseModel::Isotropic>& noise_v,
                       const boost::shared_ptr<gtsam::noiseModel::Diagonal>& noise_b);
  gtsam::ISAM2Result Update();
  void AddFrame(int id, const gtsam::PreintegratedCombinedMeasurements& pim, const gtsam::NavState& initial_navstate,
                const gtsam::imuBias::ConstantBias& initial_bias);
  gtsam::Pose3 GetPose(int frame_id);
  gtsam::Vector3 GetVelocity(int frame_id);
  gtsam::imuBias::ConstantBias GetBias(int frame_id);
};

#endif  // ORB_TEST_SRC_GRAPH_MANAGER_H_
