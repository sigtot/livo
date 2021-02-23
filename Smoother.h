#ifndef ORB_TEST_SMOOTHER_H
#define ORB_TEST_SMOOTHER_H

#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include "FeatureExtractor.h"

using namespace gtsam;

typedef IncrementalFixedLagSmoother::KeyTimestampMap TimestampMap;

class Smoother {
private:
    IncrementalFixedLagSmoother fixedLagSmoother;
public:
    explicit Smoother();
    void update(shared_ptr<Frame> frame);
    void initializeFirstTwoPoses(const shared_ptr<Frame>& firstFrame, const shared_ptr<Frame>& secondFrame);
};


#endif //ORB_TEST_SMOOTHER_H
