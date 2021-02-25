#ifndef ORB_TEST_SMOOTHER_H
#define ORB_TEST_SMOOTHER_H

#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "FeatureExtractor.h"

using namespace gtsam;

typedef IncrementalFixedLagSmoother::KeyTimestampMap TimestampMap;
typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;

class Smoother {
private:
    ISAM2 isam2;
    IncrementalFixedLagSmoother fixedLagSmoother;
    map<int, SmartFactor::shared_ptr> smartFactors;
    NonlinearFactorGraph graph;

    Values estimate;
public:
    explicit Smoother();
    void update(const shared_ptr<Frame>& frame);
    void initializeFirstTwoPoses(const shared_ptr<Frame>& firstFrame, const shared_ptr<Frame>& secondFrame);
    Pose3 updateBatch(const shared_ptr<Frame> &frame);
    vector<Point3> getLandmarkEstimates();
};


#endif //ORB_TEST_SMOOTHER_H
