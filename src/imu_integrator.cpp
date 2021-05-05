#include "imu_integrator.h"

#include <utility>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <thread>
#include <chrono>

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
