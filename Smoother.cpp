#include "Smoother.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>


ISAM2Params getIsam2Params() {
    ISAM2Params isam2Params;
    isam2Params.relinearizeThreshold = 0.1;
    isam2Params.factorization = ISAM2Params::QR;
    return isam2Params;
}

Smoother::Smoother() : fixedLagSmoother(7.0, ISAM2Params()), isam2(getIsam2Params()) {}

void Smoother::update(const shared_ptr<Frame>& frame) {
    graph.resize(0);
    Values newValues;

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

        auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v
        Cal3_S2::shared_ptr K(new Cal3_S2(215.69369557, 215.12480881, 0.0, 213.7203901, 119.26347434));

        auto existingFactorIt = smartFactors.find(landmark->id);
        SmartFactor::shared_ptr smartFactor;
        if (existingFactorIt != smartFactors.end()) {
            smartFactor = existingFactorIt->second;
            Point2 point(observation->keyPoint.pt.x, observation->keyPoint.pt.y);
            smartFactor->add(point, frame->id);
        } else {
            if (landmark->keyPointObservations.size() > 2) {
                smartFactor = SmartFactor::shared_ptr(new SmartFactor(measurementNoise, K));
                smartFactors[landmark->id] = smartFactor;
                graph.add(smartFactor);
                for (const auto &landmarkObservation : landmark->keyPointObservations) {
                    Point2 point(landmarkObservation->keyPoint.pt.x, landmarkObservation->keyPoint.pt.y);
                    auto landmarkObservationFrame = landmarkObservation->frame.lock();
                    if (landmarkObservationFrame) {
                        smartFactor->add(point, landmarkObservationFrame->id);
                    } else {
                        cout << "Failed to lock landmarkObservation->frame! This can possibly lead to a indeterminate system." << endl;
                    }
                }
            }
        }

    }

    auto lastPoseDelta = isam2.calculateEstimate<Pose3>(frame->id - 2)
            .between(isam2.calculateEstimate<Pose3>(frame->id - 1));

    auto motionPredictedPose = lastPoseDelta.compose(isam2.calculateEstimate<Pose3>(frame->id - 1));

    newValues.insert(frame->id, motionPredictedPose);

    isam2.update(graph, newValues);
}

Pose3 Smoother::updateBatch(const shared_ptr<Frame> &frame) {
    graph.resize(0);
    auto noise = noiseModel::Diagonal::Sigmas(
            (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished());

    // Implicitly defines the first pose as the origin
    graph.addPrior(0, Pose3(Rot3(), Point3::Zero()), noise);

    // Implicitly defines the scale in the scene by specifying the distance between the first two poses
    graph.addPrior(1, Pose3(Rot3(), Point3(0.1, 0, 0)), noise);

    for (const auto& observation : frame->keyPointObservations) {
        auto landmark = observation->landmark.lock();
        if (!landmark) {
            continue;
        }

        auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v
        Cal3_S2::shared_ptr K(new Cal3_S2(215.69369557, 215.12480881, 0.0, 213.7203901, 119.26347434));

        auto existingFactorIt = smartFactors.find(landmark->id);
        SmartFactor::shared_ptr smartFactor;
        if (existingFactorIt != smartFactors.end()) {
            smartFactor = existingFactorIt->second;
            Point2 point(observation->keyPoint.pt.x, observation->keyPoint.pt.y);
            smartFactor->add(point, frame->id);
        } else {
            smartFactor = SmartFactor::shared_ptr(new SmartFactor(measurementNoise, K));
            smartFactors[landmark->id] = smartFactor;
            for (const auto &landmarkObservation : landmark->keyPointObservations) {
                Point2 point(landmarkObservation->keyPoint.pt.x, landmarkObservation->keyPoint.pt.y);
                auto landmarkObservationFrame = landmarkObservation->frame.lock();
                if (landmarkObservationFrame) {
                    smartFactor->add(point, landmarkObservationFrame->id);
                } else {
                    cout << "Failed to lock landmarkObservation->frame! This can possibly lead to a indeterminate system." << endl;
                }
            }
        }

    }

    for (auto &smartFactor : smartFactors) {
        graph.add(smartFactor.second);
    }

    if (frame->id == 2) {
        estimate.insert(0, Pose3(Rot3(), Point3::Zero()));
        estimate.insert(1, Pose3(Rot3(), Point3(0.1, 0, 0)));
        estimate.insert(2, Pose3(Rot3(), Point3(0.2, 0, 0)));
    } else {
        auto lastPoseDelta = estimate.at<Pose3>(frame->id - 2)
                .between(estimate.at<Pose3>(frame->id - 1));

        auto motionPredictedPose = lastPoseDelta.compose(estimate.at<Pose3>(frame->id - 1));

        estimate.insert(frame->id, motionPredictedPose);
    }

    LevenbergMarquardtOptimizer optimizer(graph, estimate);
    Values result = optimizer.optimize();
    estimate = result;
    return result.at<Pose3>(frame->id);
}

void Smoother::initializeFirstTwoPoses(const shared_ptr<Frame>& firstFrame, const shared_ptr<Frame>& secondFrame) {
    Values values;
    auto noise = noiseModel::Diagonal::Sigmas(
            (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished());

    // Implicitly defines the first pose as the origin
    graph.addPrior(0, Pose3(Rot3(), Point3::Zero()), noise);

    // Implicitly defines the scale in the scene by specifying the distance between the first two poses
    graph.addPrior(1, Pose3(Rot3(), Point3(0.1, 0, 0)), noise);

    values.insert(0, Pose3(Rot3(), Point3::Zero()));
    values.insert(1, Pose3(Rot3(), Point3(0.1, 0, 0)));

    isam2.update(graph, values);
    cout << "initialized first two poses" << endl;
}

vector<Point3> Smoother::getLandmarkEstimates() {
    vector<Point3> points;
    for (const auto &smartFactor : smartFactors) {
        auto point = smartFactor.second->point();
        if (point) {
            points.push_back(*point);
        }
    }
    return points;
}
