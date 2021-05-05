#include "graph_manager.h"
#include "depth_triangulation.h"

#include <memory>
#include <gtsam/base/Vector.h>
#include <gtsam/base/make_shared.h>
#include <gtsam/nonlinear/ISAM2.h>
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
  auto result = isam2_->update(*graph_, *values_);
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
  auto smart_factor = gtsam::make_shared<SmartFactor>(feature_noise, K, body_p_cam, *smart_factor_params_);
  smart_factor->add(feature, X(frame_id));
  graph_->add(smart_factor);
  smart_factors_[lmk_id] = std::move(smart_factor);
  landmark_noise_models_[lmk_id] = feature_noise;
}

void GraphManager::InitProjectionLandmark(int lmk_id, int frame_id, const gtsam::Point2& feature,
                                          const gtsam::Point3& initial_estimate,
                                          const boost::shared_ptr<gtsam::noiseModel::Isotropic>& feature_noise,
                                          const boost::shared_ptr<gtsam::Cal3_S2>& K, const gtsam::Pose3& body_p_cam)
{
  ProjectionFactor proj_factor(feature, feature_noise, X(frame_id), L(lmk_id), K, body_p_cam);
  graph_->add(proj_factor);
  values_->insert(L(lmk_id), initial_estimate);
  landmark_noise_models_[lmk_id] = feature_noise;
}

void GraphManager::AddLandmarkObservation(int lmk_id, int frame_id, const gtsam::Point2& feature,
                                          const boost::shared_ptr<gtsam::Cal3_S2>& K, const gtsam::Pose3& body_p_cam)
{
  if (smart_factors_.count(lmk_id))
  {
    smart_factors_[lmk_id]->add(feature, X(frame_id));
  }
  else
  {
    ProjectionFactor proj_factor(feature, landmark_noise_models_[lmk_id], X(frame_id), L(lmk_id), K, body_p_cam);
    graph_->add(proj_factor);
  }
}

void GraphManager::AddRangeObservation(int lmk_id, int frame_id, double range,
                                       const boost::shared_ptr<gtsam::noiseModel::Isotropic>& range_noise)
{
  if (smart_factors_.count(lmk_id))
  {
    // aka "ConvertSmartFactorToProjectionFactor(lmk_id)"
    auto smart_factor = smart_factors_[lmk_id];
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
      ProjectionFactor proj_factor(feature, landmark_noise_models_[lmk_id], frame_key, L(lmk_id), K, body_p_cam);
      graph_->add(proj_factor);
    }
    smart_factor.reset();          // Makes the shared pointer within isam a nullptr, which makes isam drop the factor
    smart_factors_.erase(lmk_id);  // Removes the nullptr from our bookkeeping
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

gtsam::imuBias::ConstantBias GraphManager::GetBias(int frame_id) const
{
  return isam2_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(frame_id));
}

boost::optional<gtsam::Point3> GraphManager::GetLandmark(int lmk_id) const
{
  if (smart_factors_.count(lmk_id))
  {
    return smart_factors_.find(lmk_id)->second->point();
  }
  else if (isam2_->valueExists(L(lmk_id)))
  {
    return isam2_->calculateEstimate<gtsam::Point3>(L(lmk_id));
  }
  return boost::none;
}

gtsam::Values GraphManager::GetValues() const
{
  return isam2_->calculateEstimate();
}
