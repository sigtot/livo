#include "imu_integrator.h"

#include <utility>
#include <gtsam/navigation/CombinedImuFactor.h>

IMUIntegrator::IMUIntegrator(std::shared_ptr<IMUQueue> imu_queue,
                             const boost::shared_ptr<gtsam::PreintegrationCombinedParams>& pim_params,
                             const gtsam::imuBias::ConstantBias& init_bias)
  : imu_queue_(std::move(imu_queue))
  , pim_(std::make_shared<gtsam::PreintegratedCombinedMeasurements>(pim_params, init_bias))
{
}
