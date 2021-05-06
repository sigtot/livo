#ifndef ORB_TEST_SRC_GRAPH_MANAGER_H_
#define ORB_TEST_SRC_GRAPH_MANAGER_H_

#include "landmark_in_smoother.h"

#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>
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
class Point2;
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
  // We make each landmark use the same noise model for all observations.
  // GTSAM forces this for smart factors, we also enforce it for regular projection factors.
  std::map<int, LandmarkInSmoother> added_landmarks_;

  std::shared_ptr<gtsam::ISAM2> isam2_;
  std::shared_ptr<gtsam::Values> values_;
  std::shared_ptr<gtsam::NonlinearFactorGraph> graph_;
  std::shared_ptr<gtsam::SmartProjectionParams> smart_factor_params_;
  int last_frame_id_ = -1;

public:
  explicit GraphManager(const gtsam::ISAM2Params& isam2_params,
                        const gtsam::SmartProjectionParams& smart_factor_params);

public:
  void SetInitNavstate(int first_frame_id, const gtsam::NavState& nav_state, const gtsam::imuBias::ConstantBias& bias,
                       const boost::shared_ptr<gtsam::noiseModel::Diagonal>& noise_x,
                       const boost::shared_ptr<gtsam::noiseModel::Isotropic>& noise_v,
                       const boost::shared_ptr<gtsam::noiseModel::Diagonal>& noise_b);
  void AddFrame(int id, const gtsam::PreintegratedCombinedMeasurements& pim, const gtsam::NavState& initial_navstate,
                const gtsam::imuBias::ConstantBias& initial_bias);
  void InitStructurelessLandmark(int lmk_id, int frame_id, const gtsam::Point2& feature,
                                 const boost::shared_ptr<gtsam::noiseModel::Isotropic>& feature_noise,
                                 const boost::shared_ptr<gtsam::Cal3_S2>& K, const gtsam::Pose3& body_p_cam);
  void InitProjectionLandmark(int lmk_id, int frame_id, const gtsam::Point2& feature,
                              const gtsam::Point3& initial_estimate,
                              const boost::shared_ptr<gtsam::noiseModel::Isotropic>& feature_noise,
                              const boost::shared_ptr<gtsam::Cal3_S2>& K, const gtsam::Pose3& body_p_cam);
  void AddLandmarkObservation(int lmk_id, int frame_id, const gtsam::Point2& feature,
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
  void AddRangeObservation(int lmk_id, int frame_id, double range,
                           const boost::shared_ptr<gtsam::noiseModel::Isotropic>& range_noise);
  gtsam::ISAM2Result Update();

  gtsam::Pose3 GetPose(int frame_id) const;
  gtsam::Vector3 GetVelocity(int frame_id) const;
  gtsam::imuBias::ConstantBias GetBias(int frame_id) const;
  gtsam::Values GetValues() const;
  boost::optional<gtsam::Point3> GetLandmark(int lmk_id) const;
  std::map<int, boost::optional<gtsam::Point3>> GetLandmarks() const;
};

#endif  // ORB_TEST_SRC_GRAPH_MANAGER_H_
