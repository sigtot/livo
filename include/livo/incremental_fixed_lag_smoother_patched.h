#ifndef ORB_TEST_SRC_INCREMENTAL_FIXED_LAG_SMOOTHER_PATCHED_H_
#define ORB_TEST_SRC_INCREMENTAL_FIXED_LAG_SMOOTHER_PATCHED_H_

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
      const boost::optional<gtsam::FastMap<gtsam::FactorIndex, gtsam::FastSet<gtsam::Key>>>& newAffectedKeys = boost::none,
      const gtsam::FactorIndices& factorsToRemove = gtsam::FactorIndices());

  bool valueExists(gtsam::Key key);
};

#endif  // ORB_TEST_SRC_INCREMENTAL_FIXED_LAG_SMOOTHER_PATCHED_H_
