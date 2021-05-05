#include "new_smoother.h"
#include "global_params.h"

#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/slam/SmartFactorParams.h>
#include <gtsam/base/make_shared.h>
#include <gtsam/geometry/Cal3_S2.h>

gtsam::ISAM2Params MakeISAM2Params()
{
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = GlobalParams::IsamRelinearizeThresh();
  params.relinearizeSkip = 1;
  params.evaluateNonlinearError = true;
  if (GlobalParams::UseDogLeg())
  {
    params.optimizationParams =
        gtsam::ISAM2DoglegParams(1.0, 1e-05, gtsam::DoglegOptimizerImpl::ONE_STEP_PER_ITERATION, false);
  }
  else
  {
    params.optimizationParams = gtsam::ISAM2GaussNewtonParams();
  }
  return params;
}

NewSmoother::NewSmoother()
  : K_(gtsam::make_shared<gtsam::Cal3_S2>(GlobalParams::CamFx(), GlobalParams::CamFy(), 0.0, GlobalParams::CamU0(),
                          GlobalParams::CamV0()))
  , graph_manager(GraphManager(
        MakeISAM2Params(), gtsam::SmartProjectionParams(gtsam::HESSIAN, gtsam::IGNORE_DEGENERACY, false, true, 1e-5)))
{
}
