#ifndef ORB_TEST_SMOOTHER_H
#define ORB_TEST_SMOOTHER_H

#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/geometry/Cal3_S2.h>

#include "FeatureExtractor.h"

using namespace gtsam;

typedef IncrementalFixedLagSmoother::KeyTimestampMap TimestampMap;
typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;

class Smoother {
private:
    IncrementalFixedLagSmoother fixedLagSmoother;
    map<int, SmartFactor::shared_ptr> smartFactors;

    SmartFactor::shared_ptr getNewOrExistingFactor(int landmarkId, NonlinearFactorGraph &graph);
    Values estimate;
public:
    explicit Smoother();
    void update(const shared_ptr<Frame>& frame);
    void initializeFirstTwoPoses(const shared_ptr<Frame>& firstFrame, const shared_ptr<Frame>& secondFrame);
    void updateBatch(const shared_ptr<Frame> &frame);
};


#endif //ORB_TEST_SMOOTHER_H
