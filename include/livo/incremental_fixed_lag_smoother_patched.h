#ifndef ORB_TEST_SRC_INCREMENTAL_FIXED_LAG_SMOOTHER_PATCHED_H_
#define ORB_TEST_SRC_INCREMENTAL_FIXED_LAG_SMOOTHER_PATCHED_H_

#include <gtsam/nonlinear/ISAM2UpdateParams.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

class IncrementalFixedLagSmootherPatched : public gtsam::IncrementalFixedLagSmoother
{
public:
  explicit IncrementalFixedLagSmootherPatched(double smootherLag = 0.0,
                                              const gtsam::ISAM2Params& parameters = DefaultISAM2Params())
    : gtsam::IncrementalFixedLagSmoother(smootherLag, parameters)
  {
  }

  ~IncrementalFixedLagSmootherPatched() override = default;

  FixedLagSmoother::Result update(
      const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta, const KeyTimestampMap& timestamps,
      const gtsam::ISAM2UpdateParams& params = gtsam::ISAM2UpdateParams());

  bool valueExists(gtsam::Key key);
};

#endif  // ORB_TEST_SRC_INCREMENTAL_FIXED_LAG_SMOOTHER_PATCHED_H_
