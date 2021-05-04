#include "helpers.h"

boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> PimParams()
{
  auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
  p->gyroscopeCovariance = 0.0001 * 0.0001 * gtsam::I_3x3;
  p->accelerometerCovariance = 0.0001 * 0.0001 * gtsam::I_3x3;
  p->integrationCovariance = 0.0001 * gtsam::I_3x3;
  return p;
}
