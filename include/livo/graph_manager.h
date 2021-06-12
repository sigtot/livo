#ifndef ORB_TEST_SRC_GRAPH_MANAGER_H_
#define ORB_TEST_SRC_GRAPH_MANAGER_H_

#include "landmark_in_smoother.h"
#include "incremental_solver.h"

#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>
#include <Eigen/Core>
#include <gtsam/inference/Factor.h>

struct LandmarkResultGtsam;

namespace gtsam
{
class Cal3_S2;
class ISAM2;
class ISAM2Result;
class NonlinearFactorGraph;
class Values;
class Pose3;
class NavState;
class CombinedImuFactor;
class ISAM2Params;
class ISAM2UpdateParams;
class PreintegratedCombinedMeasurements;

class SmartProjectionParams;

template <class T>
class SmartProjectionPoseFactor;

template <class POSE, class LANDMARK, class CALIBRATION>
class GenericProjectionFactor;

template <typename A1, typename A2, typename T>
class RangeFactor;

typedef std::uint64_t Key;
typedef Eigen::Vector2d Vector2;
typedef Eigen::Vector3d Vector3;
typedef Vector2 Point2;
typedef Vector3 Point3;

typedef uint64_t FactorIndex;
typedef FastVector<FactorIndex> FactorIndices;

namespace noiseModel
{
class Isotropic;
class Base;
namespace mEstimator
{
class Base;
}
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
  // Dependencies
  std::shared_ptr<IncrementalSolver> incremental_solver_;

  // Datastructures for holding the everything related to an uncommitted update
  std::shared_ptr<gtsam::Values> values_;
  std::shared_ptr<gtsam::NonlinearFactorGraph> graph_;
  std::shared_ptr<gtsam::KeyTimestampMap> timestamps_;
  std::shared_ptr<gtsam::FastMap<gtsam::FactorIndex, gtsam::FastSet<gtsam::Key>>> new_affected_keys_;
  std::shared_ptr<gtsam::FactorIndices> factors_to_remove_;
  std::map<gtsam::FactorIndex, int> new_factor_indices_to_lmk_;

  // Configuration
  std::shared_ptr<gtsam::SmartProjectionParams> smart_factor_params_;
  double lag_;

  // Bookkeeping
  std::map<int, LandmarkInSmoother> added_landmarks_;
  int last_frame_id_ = -1;
  double last_timestamp_ = -1;

  void SetLandmarkFactorInSmootherIndices(const gtsam::FactorIndices& new_indices_in_smoother);
  void SetSmartFactorIdxInIsam(const gtsam::ISAM2Result& result);

  bool ExistsInSolverOrValues(gtsam::Key key) const;
  bool WithinLag(double timestamp) const;

public:
  GraphManager(std::shared_ptr<IncrementalSolver> incremental_solver,
               const gtsam::SmartProjectionParams& smart_factor_params, double lag = -1.);

  void SetInitNavstate(int first_frame_id, double timestamp, const gtsam::NavState& nav_state,
                       const gtsam::imuBias::ConstantBias& bias,
                       const boost::shared_ptr<gtsam::noiseModel::Base>& noise_x,
                       const boost::shared_ptr<gtsam::noiseModel::Base>& noise_v,
                       const boost::shared_ptr<gtsam::noiseModel::Base>& noise_b);
  void AddFrame(int id, double timestamp, const gtsam::PreintegratedCombinedMeasurements& pim,
                const gtsam::NavState& initial_navstate, const gtsam::imuBias::ConstantBias& initial_bias);
  void InitStructurelessLandmark(
      int lmk_id, int frame_id, double timestamp, const gtsam::Point2& feature,
      const boost::shared_ptr<gtsam::Cal3_S2>& K, const gtsam::Pose3& body_p_cam,
      const boost::shared_ptr<gtsam::noiseModel::Isotropic>& feature_noise,
      const boost::optional<boost::shared_ptr<gtsam::noiseModel::mEstimator::Base>>& m_estimator = boost::none);
  void InitProjectionLandmark(
      int lmk_id, int frame_id, double timestamp, const gtsam::Point2& feature, const gtsam::Point3& initial_estimate,
      const boost::shared_ptr<gtsam::Cal3_S2>& K, const gtsam::Pose3& body_p_cam,
      const boost::shared_ptr<gtsam::noiseModel::Isotropic>& feature_noise,
      const boost::optional<boost::shared_ptr<gtsam::noiseModel::mEstimator::Base>>& m_estimator = boost::none);
  void AddLandmarkObservation(int lmk_id, int frame_id, double timestamp, const gtsam::Point2& feature,
                              const boost::shared_ptr<gtsam::Cal3_S2>& K, const gtsam::Pose3& body_p_cam);
  /**
   * Adds a range observation to an existing landmark. If a landmark is currently represented as a smart factor, it will
   * be converted to a regular projection factor.\n
   *
   * WARNING: Must not be called after a call to AddLandmarkObservation
   * @param lmk_id id of landmark to add range factor to
   * @param frame_id if of frame in which range measurement was obtained
   * @param range measured distance from body to landmark
   * @param range_noise noise model for the range factor
   */
  void AddRangeObservation(int lmk_id, int frame_id, double timestamp, double range,
                           const boost::shared_ptr<gtsam::noiseModel::Base>& range_noise);
  void ConvertSmartFactorToProjectionFactor(int lmk_id, double timestamp, const gtsam::Point3& initial_estimate);
  void AddBetweenFactor(int frame_id_1, int frame_id_2, const gtsam::Pose3& pose,
                        const boost::shared_ptr<gtsam::noiseModel::Base>& between_noise);
  void RemoveLandmark(int lmk_id);
  gtsam::ISAM2Result Update();

  gtsam::Pose3 GetPose(int frame_id) const;
  gtsam::Vector3 GetVelocity(int frame_id) const;
  gtsam::NavState GetNavState(int frame_id) const;
  gtsam::imuBias::ConstantBias GetBias(int frame_id) const;
  gtsam::Values GetValues() const;
  boost::optional<LandmarkResultGtsam> GetLandmark(int lmk_id) const;
  std::map<int, boost::optional<LandmarkResultGtsam>> GetLandmarks() const;
  bool IsLandmarkTracked(int lmk_id) const;
  bool IsFrameTracked(int frame_id) const;
  bool CanAddObservation(int lmk_id, int frame_id) const;
  bool CanAddObservationsForFrame(int frame_id, double frame_timestamp) const;
  bool CanAddRangeObservation(int lmk_id, int frame_id) const;
  bool IsSmartFactorLandmark(int lmk_id) const;
};

#endif  // ORB_TEST_SRC_GRAPH_MANAGER_H_
