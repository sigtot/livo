#ifndef ORB_TEST_SRC_IMU_INTEGRATOR_H_
#define ORB_TEST_SRC_IMU_INTEGRATOR_H_

#include "imu_queue.h"

#include <memory>

namespace gtsam
{
class TangentPreintegration;
typedef TangentPreintegration PreintegrationType;
class PreintegrationCombinedParams;
namespace imuBias
{
class ConstantBias;
}
}  // namespace gtsam

class IMUIntegrator
{
private:
  std::shared_ptr<IMUQueue> imu_queue_;
  std::shared_ptr<gtsam::PreintegrationType> pim_;

public:
  IMUIntegrator(std::shared_ptr<IMUQueue> imu_queue,
                const boost::shared_ptr<gtsam::PreintegrationCombinedParams>& pim_params,
                const gtsam::imuBias::ConstantBias& init_bias);
};

#endif  // ORB_TEST_SRC_IMU_INTEGRATOR_H_
