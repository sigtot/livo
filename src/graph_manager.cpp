#include "graph_manager.h"
#include "depth_triangulation.h"

#include <memory>
#include <gtsam/base/Vector.h>
#include <gtsam/base/make_shared.h>
#include <gtsam/nonlinear/ISAM2.h>
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

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::L;  // Landmarks (x,y,z)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

GraphManager::GraphManager(const gtsam::ISAM2Params& isam2_params,
                           const gtsam::SmartProjectionParams& smart_factor_params)
  : isam2_(std::make_shared<gtsam::ISAM2>(isam2_params))
  , graph_(std::make_shared<gtsam::NonlinearFactorGraph>())
  , values_(std::make_shared<gtsam::Values>())
  , smart_factor_params_(std::make_shared<gtsam::SmartProjectionParams>(smart_factor_params))
{
}
void GraphManager::SetInitNavstate(int first_frame_id, const gtsam::NavState& nav_state,
                                   const gtsam::imuBias::ConstantBias& bias,
                                   const boost::shared_ptr<gtsam::noiseModel::Diagonal>& noise_x,
                                   const boost::shared_ptr<gtsam::noiseModel::Isotropic>& noise_v,
                                   const boost::shared_ptr<gtsam::noiseModel::Diagonal>& noise_b)
{
  values_->insert(X(first_frame_id), nav_state.pose());
  values_->insert(V(first_frame_id), nav_state.velocity());
  values_->insert(B(first_frame_id), bias);

  graph_->addPrior(X(first_frame_id), nav_state.pose(), noise_x);
  graph_->addPrior(V(first_frame_id), nav_state.velocity(), noise_v);
  graph_->addPrior(B(first_frame_id), bias, noise_b);

  last_frame_id_ = first_frame_id;
}

gtsam::ISAM2Result GraphManager::Update()
{
  return Update(gtsam::ISAM2UpdateParams());
}

gtsam::ISAM2Result GraphManager::Update(const gtsam::ISAM2UpdateParams& update_params)
{
  auto result = isam2_->update(*graph_, *values_, update_params);
  graph_->resize(0);
  values_->clear();
  return result;
}

void GraphManager::AddFrame(int id, const gtsam::PreintegratedCombinedMeasurements& pim,
                            const gtsam::NavState& initial_navstate, const gtsam::imuBias::ConstantBias& initial_bias)
{
  values_->insert(X(id), initial_navstate.pose());
  values_->insert(V(id), initial_navstate.velocity());
  values_->insert(B(id), initial_bias);

  gtsam::CombinedImuFactor imu_factor(X(last_frame_id_), V(last_frame_id_), X(id), V(id), B(last_frame_id_), B(id),
                                      pim);
  graph_->add(imu_factor);

  last_frame_id_ = id;
}

void GraphManager::InitStructurelessLandmark(int lmk_id, int frame_id, const gtsam::Point2& feature,
                                             const boost::shared_ptr<gtsam::noiseModel::Isotropic>& feature_noise,
                                             const boost::shared_ptr<gtsam::Cal3_S2>& K, const gtsam::Pose3& body_p_cam)
{
  LandmarkInSmoother landmark_in_smoother;
  landmark_in_smoother.smart_factor =
      gtsam::make_shared<SmartFactor>(feature_noise, K, body_p_cam, *smart_factor_params_);
  landmark_in_smoother.noise_model = feature_noise;
  (*landmark_in_smoother.smart_factor)->add(feature, X(frame_id));
  graph_->add(*landmark_in_smoother.smart_factor);
  added_landmarks_[lmk_id] = landmark_in_smoother;
}

void GraphManager::InitProjectionLandmark(int lmk_id, int frame_id, const gtsam::Point2& feature,
                                          const gtsam::Point3& initial_estimate,
                                          const boost::shared_ptr<gtsam::noiseModel::Isotropic>& feature_noise,
                                          const boost::shared_ptr<gtsam::Cal3_S2>& K, const gtsam::Pose3& body_p_cam)
{
  LandmarkInSmoother landmark_in_smoother;
  landmark_in_smoother.noise_model = feature_noise;
  ProjectionFactor proj_factor(feature, feature_noise, X(frame_id), L(lmk_id), K, body_p_cam);
  graph_->add(proj_factor);
  values_->insert(L(lmk_id), initial_estimate);
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
  if (landmark_in_smoother->second.smart_factor)
  {
    (*landmark_in_smoother->second.smart_factor)->add(feature, X(frame_id));
  }
  else
  {
    ProjectionFactor proj_factor(feature, landmark_in_smoother->second.noise_model, X(frame_id), L(lmk_id), K,
                                 body_p_cam);
    graph_->add(proj_factor);
  }
}

void GraphManager::AddRangeObservation(int lmk_id, int frame_id, double range,
                                       const boost::shared_ptr<gtsam::noiseModel::Isotropic>& range_noise)
{
  auto landmark_in_smoother = added_landmarks_.find(lmk_id);
  if (landmark_in_smoother == added_landmarks_.end())
  {
    std::cout << "WARN (graph manager): attempted to add range for non-existing landmark " << lmk_id << std::endl;
    return;
  }
  if (landmark_in_smoother->second.smart_factor)
  {
    // aka "ConvertSmartFactorToProjectionFactor(lmk_id)"
    boost::shared_ptr<SmartFactor> smart_factor = *landmark_in_smoother->second.smart_factor;
    if (smart_factor->isValid())
    {
      values_->insert(L(lmk_id), *smart_factor->point());
    }
    else
    {
      auto measurement = smart_factor->measured().front();
      auto camera = smart_factor->cameras(GetValues()).front();  // fails if AddLandmarkObservation called first
      auto initial_point_estimate_from_range = DepthTriangulation::PixelAndDepthToPoint3(measurement, range, camera);
      values_->insert(L(lmk_id), initial_point_estimate_from_range);
    }
    for (int i = 0; i < smart_factor->keys().size(); ++i)
    {
      auto feature = smart_factor->measured()[i];
      auto frame_key = smart_factor->keys()[i];
      auto K = smart_factor->calibration();
      auto body_p_cam = smart_factor->body_P_sensor();
      ProjectionFactor proj_factor(feature, landmark_in_smoother->second.noise_model, frame_key, L(lmk_id), K,
                                   body_p_cam);
      graph_->add(proj_factor);
    }
    smart_factor.reset();          // Makes the shared pointer within isam a nullptr, which makes isam drop the factor
    landmark_in_smoother->second.smart_factor = boost::none; // Remove nullptr smart factor from our bookkeeping
  }

  RangeFactor range_factor(X(frame_id), L(lmk_id), range, range_noise);
  graph_->add(range_factor);
}

gtsam::Pose3 GraphManager::GetPose(int frame_id) const
{
  return isam2_->calculateEstimate<gtsam::Pose3>(X(frame_id));
}

gtsam::Vector3 GraphManager::GetVelocity(int frame_id) const
{
  return isam2_->calculateEstimate<gtsam::Vector3>(V(frame_id));
}

gtsam::NavState GraphManager::GetNavState(int frame_id) const
{
  return gtsam::NavState(GetPose(frame_id), GetVelocity(frame_id));
}

gtsam::imuBias::ConstantBias GraphManager::GetBias(int frame_id) const
{
  return isam2_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(frame_id));
}

boost::optional<gtsam::Point3> GraphManager::GetLandmark(int lmk_id) const
{
  auto landmark_in_smoother = added_landmarks_.find(lmk_id);
  if (landmark_in_smoother == added_landmarks_.end())
  {
    std::cout << "WARN (graph manager): attempted to get point for non-existing landmark " << lmk_id << std::endl;
    return boost::none;
  }
  if (landmark_in_smoother->second.smart_factor)
  {
    if (!(*landmark_in_smoother->second.smart_factor)->isValid())
    {
      return boost::none;
    }
    return *(*landmark_in_smoother->second.smart_factor)->point();
  }
  return isam2_->calculateEstimate<gtsam::Point3>(L(lmk_id));
}

std::map<int, boost::optional<gtsam::Point3>> GraphManager::GetLandmarks() const
{
  std::map<int, boost::optional<gtsam::Point3>> landmarks;
  for (const auto& landmark_in_smoother : added_landmarks_)
  {
    landmarks[landmark_in_smoother.first] = GetLandmark(landmark_in_smoother.first);
  }
  return landmarks;
}

gtsam::Values GraphManager::GetValues() const
{
  return isam2_->calculateEstimate();
}

bool GraphManager::IsLandmarkTracked(int lmk_id) const
{
  return added_landmarks_.count(lmk_id) != 0;
}

bool GraphManager::IsFrameTracked(int frame_id) const
{
  return isam2_->valueExists(X(frame_id));
}

bool GraphManager::CanAddObservationsForFrame(int frame_id) const
{
  return IsFrameTracked(frame_id) || values_->exists(X(frame_id));
}
