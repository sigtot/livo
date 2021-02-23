#include "Smoother.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>

Smoother::Smoother() : fixedLagSmoother(7.0, ISAM2Params()) {}

void Smoother::update(const shared_ptr<Frame>& frame) {
    NonlinearFactorGraph newFactors;
    Values newValues;
    TimestampMap newTimestamps;

    /* TODO Useful for debugging, but remove after refactoring dupe removal code in FeatureExtractor
    for (int i = 0; i < frame->keyPointObservations.size(); ++i) {
        for (int j = 0; j < frame->keyPointObservations.size(); ++j) {
            if (i == j) {
                continue;
            }
            auto landmark1 = frame->keyPointObservations[i]->landmark.lock();
            auto landmark2 = frame->keyPointObservations[j]->landmark.lock();
            if (landmark1 && landmark2 && landmark1->id == landmark2->id) {
                cout << "Oh no, duplicate landmark observation in frame " << frame->id << ": landmarks " << landmark1->id << " " << landmark2->id << endl;
            }
        }
    }
    */

    for (const auto& observation : frame->keyPointObservations) {
        auto landmark = observation->landmark.lock();
        if (!landmark) {
            continue;
        }

        auto smartFactor = getNewOrExistingFactor(landmark->id, newFactors);
        Point2 point(observation->keyPoint.pt.x, observation->keyPoint.pt.y);
        smartFactor->add(point, frame->id);
    }
    newTimestamps[frame->id] = frame->timeStamp;
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

SmartFactor::shared_ptr Smoother::getNewOrExistingFactor(int landmarkId, NonlinearFactorGraph &graph) {
    auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v
    Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0)); // TODO fix

    auto existingFactorIt = smartFactors.find(landmarkId);
    if (existingFactorIt != smartFactors.end()) {
        return existingFactorIt->second;
    } else {
        SmartFactor::shared_ptr smartFactor(new SmartFactor(measurementNoise, K));
        smartFactors[landmarkId] = smartFactor;
        graph.add(smartFactor);
        return smartFactor;
    }
}
