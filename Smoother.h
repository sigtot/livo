#ifndef ORB_TEST_SMOOTHER_H
#define ORB_TEST_SMOOTHER_H

#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include "FeatureExtractor.h"

using namespace gtsam;

class Smoother {
private:
public:
    explicit Smoother();
    void update();
};


#endif //ORB_TEST_SMOOTHER_H
