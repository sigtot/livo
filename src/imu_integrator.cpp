#include "imu_integrator.h"
#include "gtsam_conversions.h"

#include <utility>
#include <thread>
#include <chrono>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/TangentPreintegration.h>

IMUIntegrator::IMUIntegrator(std::shared_ptr<IMUQueue> imu_queue,
                             const boost::shared_ptr<gtsam::PreintegrationCombinedParams>& pim_params,
                             const gtsam::imuBias::ConstantBias& init_bias)
  : imu_queue_(std::move(imu_queue))
  , pim_(std::make_shared<gtsam::PreintegratedCombinedMeasurements>(pim_params, init_bias))
{
}

void IMUIntegrator::WaitAndIntegrate(double timestamp1, double timestamp2)
{
  while (!imu_queue_->hasMeasurementsInRange(timestamp1, timestamp2))
  {
    std::cout << "No IMU measurements in time range " << std::setprecision(17) << timestamp1 << " -> " << timestamp2
              << std::endl;
    std::cout << "Waiting 1 ms" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  imu_queue_->integrateIMUMeasurements(pim_, timestamp1, timestamp2);
}

void IMUIntegrator::ResetIntegration()
{
  pim_->resetIntegration();
}

gtsam::NavState IMUIntegrator::PredictNavState(const gtsam::NavState& prev_nav_state,
                                               const gtsam::imuBias::ConstantBias& prev_bias) const
{
  return pim_->predict(prev_nav_state, prev_bias);
}

std::shared_ptr<gtsam::PreintegrationType> IMUIntegrator::GetPim() const
{
  return pim_;
}

gtsam::Rot3 IMUIntegrator::RefineInitialAttitude(double start, double end, const gtsam::Rot3& initial_attitude) const
{
  return ToGtsamRot(imu_queue_->RefineInitialAttitude(start, end, ToRot(initial_attitude)));
}
