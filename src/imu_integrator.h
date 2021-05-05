#ifndef ORB_TEST_SRC_IMU_INTEGRATOR_H_
#define ORB_TEST_SRC_IMU_INTEGRATOR_H_

#include "imu_queue.h"

#include <memory>

namespace gtsam
{
class TangentPreintegration;
class PreintegrationCombinedParams;
class NavState;

typedef TangentPreintegration PreintegrationType;

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
  void WaitAndIntegrate(double timestamp1, double timestamp2);
  void ResetIntegration();
  gtsam::NavState PredictNavState(const gtsam::NavState& prev_nav_state,
                                  const gtsam::imuBias::ConstantBias& prev_bias) const;
  std::shared_ptr<gtsam::PreintegrationType> GetPim() const;
};

#endif  // ORB_TEST_SRC_IMU_INTEGRATOR_H_
