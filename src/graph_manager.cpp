#include "graph_manager.h"

#include <memory>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/CombinedImuFactor.h>

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::L;  // Landmarks (x,y,z)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

GraphManager::GraphManager(const gtsam::ISAM2Params& isam2_params)
  : isam2_(std::make_shared<gtsam::ISAM2>(isam2_params))
  , graph_(std::make_shared<gtsam::NonlinearFactorGraph>())
  , values_(std::make_shared<gtsam::Values>())
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

gtsam::Pose3 GraphManager::GetPose(int frame_id)
{
  return isam2_->calculateEstimate<gtsam::Pose3>(X(frame_id));
}

gtsam::Vector3 GraphManager::GetVelocity(int frame_id)
{
  return isam2_->calculateEstimate<gtsam::Vector3>(V(frame_id));
}

gtsam::imuBias::ConstantBias GraphManager::GetBias(int frame_id)
{
  return isam2_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(frame_id));
}
