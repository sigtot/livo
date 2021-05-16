#include "graph_manager.h"
#include "depth_triangulation.h"
#include "landmark_result_gtsam.h"
#include "smart_factor_in_smoother.h"

#include <memory>
#include <utility>
#include <gtsam/base/Vector.h>
#include <gtsam/base/make_shared.h>
#include <gtsam/nonlinear/ISAM2UpdateParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/SmartFactorParams.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/linear/LossFunctions.h>

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::L;  // Landmarks (x,y,z)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

GraphManager::GraphManager(std::shared_ptr<IncrementalSolver> incremental_solver,
                           const gtsam::SmartProjectionParams& smart_factor_params, double lag)
  : incremental_solver_(std::move(incremental_solver))
  , graph_(std::make_shared<gtsam::NonlinearFactorGraph>())
  , values_(std::make_shared<gtsam::Values>())
  , timestamps_(std::make_shared<gtsam::KeyTimestampMap>())
  , new_affected_keys_(std::make_shared<gtsam::FastMap<gtsam::FactorIndex, gtsam::FastSet<gtsam::Key>>>())
  , smart_factor_params_(std::make_shared<gtsam::SmartProjectionParams>(smart_factor_params))
  , lag_(lag)
{
}

void GraphManager::SetInitNavstate(int first_frame_id, double timestamp, const gtsam::NavState& nav_state,
                                   const gtsam::imuBias::ConstantBias& bias,
                                   const boost::shared_ptr<gtsam::noiseModel::Base>& noise_x,
                                   const boost::shared_ptr<gtsam::noiseModel::Base>& noise_v,
                                   const boost::shared_ptr<gtsam::noiseModel::Base>& noise_b)
{
  values_->insert(X(first_frame_id), nav_state.pose());
  values_->insert(V(first_frame_id), nav_state.velocity());
  values_->insert(B(first_frame_id), bias);

  graph_->addPrior(X(first_frame_id), nav_state.pose(), noise_x);
  graph_->addPrior(V(first_frame_id), nav_state.velocity(), noise_v);
  graph_->addPrior(B(first_frame_id), bias, noise_b);

  timestamps_->insert({ X(first_frame_id), timestamp });
  timestamps_->insert({ V(first_frame_id), timestamp });
  timestamps_->insert({ B(first_frame_id), timestamp });

  last_frame_id_ = first_frame_id;
  last_timestamp_ = timestamp;
}

gtsam::ISAM2Result GraphManager::Update()
{
  for (const auto& affected_key : *new_affected_keys_)
  {
    std::cout << affected_key.first << " affects keys ";
    for (const auto& key : affected_key.second)
    {
      std::cout << gtsam::_defaultKeyFormatter(key) << " ";
    }
    std::cout << std::endl;
  }
  auto result = incremental_solver_->Update(*graph_, *values_, *timestamps_, *new_affected_keys_);
  graph_->resize(0);
  values_->clear();
  timestamps_->clear();
  new_affected_keys_->clear();

  for (auto& lmk_in_smoother : added_landmarks_)
  {
    if (lmk_in_smoother.second.smart_factor_in_smoother &&
        !lmk_in_smoother.second.smart_factor_in_smoother->idx_in_isam)
    {
      auto idx_in_new_factors = lmk_in_smoother.second.smart_factor_in_smoother->idx_in_new_factors;
      lmk_in_smoother.second.smart_factor_in_smoother->idx_in_isam = result.newFactorsIndices[idx_in_new_factors];
      std::cout << "Set lmk " << lmk_in_smoother.first << " idx to " << result.newFactorsIndices[idx_in_new_factors]
                << std::endl;
    }
  }

  return result;
}

void GraphManager::AddFrame(int id, double timestamp, const gtsam::PreintegratedCombinedMeasurements& pim,
                            const gtsam::NavState& initial_navstate, const gtsam::imuBias::ConstantBias& initial_bias)
{
  values_->insert(X(id), initial_navstate.pose());
  values_->insert(V(id), initial_navstate.velocity());
  values_->insert(B(id), initial_bias);

  gtsam::CombinedImuFactor imu_factor(X(last_frame_id_), V(last_frame_id_), X(id), V(id), B(last_frame_id_), B(id),
                                      pim);
  graph_->add(imu_factor);

  timestamps_->insert({ X(id), timestamp });
  timestamps_->insert({ V(id), timestamp });
  timestamps_->insert({ B(id), timestamp });

  last_frame_id_ = id;
  last_timestamp_ = timestamp;

  // RemoveExpiredSmartFactors();
}

void GraphManager::InitStructurelessLandmark(
    int lmk_id, int frame_id, double timestamp, const gtsam::Point2& feature,
    const boost::shared_ptr<gtsam::Cal3_S2>& K, const gtsam::Pose3& body_p_cam,
    const boost::shared_ptr<gtsam::noiseModel::Isotropic>& feature_noise,
    const boost::optional<boost::shared_ptr<gtsam::noiseModel::mEstimator::Base>>& m_estimator)
{
  LandmarkInSmoother landmark_in_smoother;
  landmark_in_smoother.newest_frame_id_seen = frame_id;
  landmark_in_smoother.first_frame_id_seen = frame_id;
  landmark_in_smoother.init_timestamp = timestamp;
  landmark_in_smoother.noise_model = feature_noise;
  if (m_estimator)
  {
    landmark_in_smoother.robust_noise_model = gtsam::noiseModel::Robust::Create(*m_estimator, feature_noise);
  }
  SmartFactorInSmoother smart_factor_in_smoother;

  smart_factor_in_smoother.smart_factor =
      gtsam::make_shared<SmartFactor>(feature_noise, K, body_p_cam, *smart_factor_params_);
  smart_factor_in_smoother.smart_factor->add(feature, X(frame_id));
  smart_factor_in_smoother.idx_in_new_factors = graph_->size();
  graph_->add(smart_factor_in_smoother.smart_factor);
  landmark_in_smoother.smart_factor_in_smoother = smart_factor_in_smoother;
  added_landmarks_[lmk_id] = landmark_in_smoother;
}

void GraphManager::InitProjectionLandmark(
    int lmk_id, int frame_id, double timestamp, const gtsam::Point2& feature, const gtsam::Point3& initial_estimate,
    const boost::shared_ptr<gtsam::Cal3_S2>& K, const gtsam::Pose3& body_p_cam,
    const boost::shared_ptr<gtsam::noiseModel::Isotropic>& feature_noise,
    const boost::optional<boost::shared_ptr<gtsam::noiseModel::mEstimator::Base>>& m_estimator)
{
  LandmarkInSmoother landmark_in_smoother;
  landmark_in_smoother.newest_frame_id_seen = frame_id;
  landmark_in_smoother.first_frame_id_seen = frame_id;
  landmark_in_smoother.init_timestamp = timestamp;
  landmark_in_smoother.noise_model = feature_noise;
  if (m_estimator)
  {
    landmark_in_smoother.robust_noise_model = gtsam::noiseModel::Robust::Create(*m_estimator, feature_noise);
  }
  ProjectionFactor proj_factor(
      feature, landmark_in_smoother.robust_noise_model ? *landmark_in_smoother.robust_noise_model : feature_noise,
      X(frame_id), L(lmk_id), K, body_p_cam);
  graph_->add(proj_factor);
  values_->insert(L(lmk_id), initial_estimate);
  timestamps_->insert({ L(lmk_id), timestamp });
  added_landmarks_[lmk_id] = landmark_in_smoother;
}

void GraphManager::AddLandmarkObservation(int lmk_id, int frame_id, const gtsam::Point2& feature,
                                          const boost::shared_ptr<gtsam::Cal3_S2>& K, const gtsam::Pose3& body_p_cam)
{
  auto landmark_in_smoother = added_landmarks_.find(lmk_id);
  if (landmark_in_smoother == added_landmarks_.end())
  {
    std::cout << "WARN (graph manager): attempted to add observation for non-existing landmark " << lmk_id << std::endl;
    return;
  }
  if (landmark_in_smoother->second.first_frame_id_seen > frame_id)
  {
    std::cout << "WARN (graph manager): attempted to add past observation of landmark " << lmk_id;
    std::cout << " (added in frame " << landmark_in_smoother->second.first_frame_id_seen << " and requested to add in ";
    std::cout << frame_id << std::endl;
    return;
  }
  landmark_in_smoother->second.newest_frame_id_seen =
      std::max(landmark_in_smoother->second.newest_frame_id_seen, frame_id);
  if (landmark_in_smoother->second.smart_factor_in_smoother)
  {
    landmark_in_smoother->second.smart_factor_in_smoother->smart_factor->add(feature, X(frame_id));
    if (landmark_in_smoother->second.smart_factor_in_smoother->idx_in_isam)
    {
      (*new_affected_keys_)[*landmark_in_smoother->second.smart_factor_in_smoother->idx_in_isam].insert(X(frame_id));
    }
  }
  else
  {
    auto noise_model = landmark_in_smoother->second.robust_noise_model ?
                           *landmark_in_smoother->second.robust_noise_model :
                           landmark_in_smoother->second.noise_model;
    ProjectionFactor proj_factor(feature, noise_model, X(frame_id), L(lmk_id), K, body_p_cam);
    graph_->add(proj_factor);
  }
}

void GraphManager::AddRangeObservation(int lmk_id, int frame_id, double range,
                                       const boost::shared_ptr<gtsam::noiseModel::Base>& range_noise)
{
  auto landmark_in_smoother = added_landmarks_.find(lmk_id);
  if (landmark_in_smoother == added_landmarks_.end())
  {
    std::cout << "WARN (graph manager): attempted to add range for non-existing landmark " << lmk_id << std::endl;
    return;
  }
  landmark_in_smoother->second.newest_frame_id_seen =
      std::max(landmark_in_smoother->second.newest_frame_id_seen, frame_id);

  RangeFactor range_factor(X(frame_id), L(lmk_id), range, range_noise);
  graph_->add(range_factor);
}

void GraphManager::ConvertSmartFactorToProjectionFactor(int lmk_id, const gtsam::Point3& initial_estimate)
{
  assert(added_landmarks_.count(lmk_id) && added_landmarks_[lmk_id].smart_factor_in_smoother);
  auto landmark_in_smoother = added_landmarks_[lmk_id];
  boost::shared_ptr<SmartFactor> smart_factor = landmark_in_smoother.smart_factor_in_smoother->smart_factor;
  values_->insert(L(lmk_id), initial_estimate);
  timestamps_->insert({ L(lmk_id), landmark_in_smoother.init_timestamp });
  for (int i = 0; i < smart_factor->keys().size(); ++i)
  {
    auto feature = smart_factor->measured()[i];
    auto frame_key = smart_factor->keys()[i];
    auto K = smart_factor->calibration();
    auto body_p_cam = smart_factor->body_P_sensor();
    auto noise_model = landmark_in_smoother.robust_noise_model ? *landmark_in_smoother.robust_noise_model :
                                                                 landmark_in_smoother.noise_model;
    ProjectionFactor proj_factor(feature, noise_model, frame_key, L(lmk_id), K, body_p_cam);
    graph_->add(proj_factor);
  }
  added_landmarks_[lmk_id].smart_factor_in_smoother->smart_factor.reset();  // Makes the shared pointer within isam a
                                                                            // nullptr, which causes isam drop the
                                                                            // factor
  assert(added_landmarks_[lmk_id].smart_factor_in_smoother->smart_factor == nullptr);
  added_landmarks_[lmk_id].smart_factor_in_smoother = boost::none;  // Remove nullptr smart factor from our bookkeeping
}

void GraphManager::RemoveExpiredSmartFactors()
{
  for (const auto& lmk_in_smoother : added_landmarks_)
  {
    if (!WithinLag(lmk_in_smoother.second.init_timestamp) && lmk_in_smoother.second.smart_factor_in_smoother)
    {
      auto smart_factor = *lmk_in_smoother.second.smart_factor_in_smoother->smart_factor;
      std::cout << "Converting smart factor to proj factor " << lmk_in_smoother.first << std::endl;
      auto lmk = GetLandmark(lmk_in_smoother.first);
      if (lmk)
      {
        ConvertSmartFactorToProjectionFactor(lmk_in_smoother.first, lmk->pt);
      }
      else
      {
        std::cout << "ERR: cannot convert degenerate smart factor to proj factor!! Do something else." << std::endl;
      }

      // We don't remove the lmk_in_smoother, because this would cause us to keep re-adding and removing the factor
      // until the last observation. This will likely hurt performance a lot.
      // added_landmarks_.erase(lmk_in_smoother.first);
    }
  }
}

gtsam::Pose3 GraphManager::GetPose(int frame_id) const
{
  return incremental_solver_->CalculateEstimatePose3(X(frame_id));
}

gtsam::Vector3 GraphManager::GetVelocity(int frame_id) const
{
  return incremental_solver_->CalculateEstimateVector3(V(frame_id));
}

gtsam::NavState GraphManager::GetNavState(int frame_id) const
{
  return gtsam::NavState(GetPose(frame_id), GetVelocity(frame_id));
}

gtsam::imuBias::ConstantBias GraphManager::GetBias(int frame_id) const
{
  return incremental_solver_->CalculateEstimateBias(B(frame_id));
}

boost::optional<LandmarkResultGtsam> GraphManager::GetLandmark(int lmk_id) const
{
  auto landmark_in_smoother = added_landmarks_.find(lmk_id);
  if (landmark_in_smoother == added_landmarks_.end())
  {
    std::cout << "WARN (graph manager): attempted to get point for non-existing landmark " << lmk_id << std::endl;
    return boost::none;
  }
  auto active = landmark_in_smoother->second.newest_frame_id_seen >= last_frame_id_;
  if (landmark_in_smoother->second.smart_factor_in_smoother)
  {
    if (!landmark_in_smoother->second.smart_factor_in_smoother->smart_factor->isValid())
    {
      return boost::none;
    }
    return LandmarkResultGtsam{ *(landmark_in_smoother->second.smart_factor_in_smoother->smart_factor->point()),
                                SmartFactorType, active };
  }
  return LandmarkResultGtsam{ incremental_solver_->CalculateEstimatePoint3(L(lmk_id)), ProjectionFactorType, active };
}

std::map<int, boost::optional<LandmarkResultGtsam>> GraphManager::GetLandmarks() const
{
  std::map<int, boost::optional<LandmarkResultGtsam>> landmarks;
  for (const auto& landmark_in_smoother : added_landmarks_)
  {
    if (IsLandmarkTracked(landmark_in_smoother.first))
    {
      landmarks[landmark_in_smoother.first] = GetLandmark(landmark_in_smoother.first);
    }
  }
  return landmarks;
}

bool GraphManager::ExistsInSolverOrValues(gtsam::Key key) const
{
  return incremental_solver_->ValueExists(key) || values_->exists(key);
}

gtsam::Values GraphManager::GetValues() const
{
  return incremental_solver_->CalculateEstimate();
}

bool GraphManager::IsLandmarkTracked(int lmk_id) const
{
  auto lmk_in_smoother = added_landmarks_.find(lmk_id);
  return lmk_in_smoother != added_landmarks_.end() &&
         ((lmk_in_smoother->second.smart_factor_in_smoother &&
           ExistsInSolverOrValues(X(lmk_in_smoother->second.first_frame_id_seen))) ||
          ExistsInSolverOrValues(L(lmk_id)));
}

bool GraphManager::IsFrameTracked(int frame_id) const
{
  return incremental_solver_->ValueExists(X(frame_id));
}

bool GraphManager::WithinLag(double timestamp) const
{
  if (lag_ < 0)
  {
    return true;  // Fixed lag smoothing disabled, so all timestamps are within lag
  }
  return timestamp > last_timestamp_ - lag_;
}

bool GraphManager::CanAddObservationsForFrame(int frame_id, double frame_timestamp) const
{
  return ExistsInSolverOrValues(X(frame_id)) && WithinLag(frame_timestamp);
}

bool GraphManager::CanAddObservation(int lmk_id, int frame_id) const
{
  auto lmk_in_smoother = added_landmarks_.find(lmk_id);
  if (lmk_in_smoother == added_landmarks_.end())
  {
    return false;
  }

  return IsLandmarkTracked(lmk_id) && ExistsInSolverOrValues(X(frame_id)) &&
         WithinLag(lmk_in_smoother->second.init_timestamp);
}

bool GraphManager::CanAddRangeObservation(int lmk_id, int frame_id) const
{
  auto lmk_in_smoother = added_landmarks_.find(lmk_id);
  if (lmk_in_smoother == added_landmarks_.end())
  {
    return false;
  }
  return CanAddObservation(lmk_id, frame_id) && !lmk_in_smoother->second.smart_factor_in_smoother;
}

bool GraphManager::IsSmartFactorLandmark(int lmk_id) const
{
  auto lmk_in_smoother = added_landmarks_.find(lmk_id);
  if (lmk_in_smoother == added_landmarks_.end())
  {
    return false;
  }
  return lmk_in_smoother->second.smart_factor_in_smoother.is_initialized();
}
