#include "Smoother.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>

Smoother::Smoother() : fixedLagSmoother(7.0, ISAM2Params()) {}

void Smoother::update(shared_ptr<Frame> frame) {
    NonlinearFactorGraph newFactors;
    Values newValues;
    TimestampMap newTimestamps;
}

void Smoother::initializeFirstTwoPoses(const shared_ptr<Frame>& firstFrame, const shared_ptr<Frame>& secondFrame) {
    NonlinearFactorGraph graph;
    Values values;
    TimestampMap newTimestamps;
    auto noise = noiseModel::Diagonal::Sigmas(
            (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished());

    // Implicitly defines the first pose as the origin
    graph.addPrior(0, Pose3(Rot3(), Point3()), noise);

    // Implicitly defines the scale in the scene by specifying the distance between the first two poses
    graph.addPrior(1, Pose3(Rot3(), Point3(0.1, 0, 0)), noise);

    values.insert(0, Pose3(Rot3(), Point3()));
    values.insert(1, Pose3(Rot3(), Point3(0.1, 0, 0)));

    newTimestamps[firstFrame->id] = firstFrame->timeStamp;
    newTimestamps[secondFrame->id] = secondFrame->timeStamp;

    fixedLagSmoother.update(graph, values, newTimestamps);
    cout << "initialized first two poses" << endl;
}
