#include "smoother.h"
#include "key_point_observation.h"
#include "landmark.h"
#include "frame.h"
#include "newer_college_ground_truth.h"
#include "gtsam_conversions.h"

#include <iostream>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> SmartFactor;

void Smoother::SmoothBatch(
    const std::vector<std::shared_ptr<Frame>>& frames,
    const std::vector<std::shared_ptr<Landmark>>& landmarks) {
  std::cout << "Let's process those" << landmarks.size() << " landmarks"
            << std::endl;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values estimate;

  auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1),
       gtsam::Vector3::Constant(0.1))
          .finished());

  // Add prior on first pose
  Pose3 gt_pose_0 = NewerCollegeGroundTruth::At(frames[0]->timestamp);
  graph.addPrior(0, ToGtsamPose(gt_pose_0), prior_noise);
  ToGtsamPose(gt_pose_0).print("prior 0");

  // Add prior on 50th pose to define scale
  Pose3 gt_pose_50 = NewerCollegeGroundTruth::At(frames[50]->timestamp);
  ToGtsamPose(gt_pose_50).print("prior 50");
  graph.addPrior(50, ToGtsamPose(gt_pose_50), prior_noise);

  // san raf RESIZE_FACTOR=0.5
  // Cal3_S2::shared_ptr K(new Cal3_S2(593.690871957, 593.74699226, 0.0,
  // 388.42480338, 274.84471313));

  // newer college RESIZE_FACTOR=1
  gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(
      431.38739114, 430.24961762, 0.0, 427.4407802, 238.52694868));

  auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  int smart_factor_count = 0;
  for (auto& landmark : landmarks) {
    if (landmark->keypoint_observations.size() < 10) {
      continue;
    }
    std::cout << "adding landmark with "
              << landmark->keypoint_observations.size() << " observations"
              << std::endl;
    SmartFactor::shared_ptr smart_factor(new SmartFactor(measurementNoise, K));
    for (auto& obs : landmark->keypoint_observations) {
      auto pt = obs->keypoint.pt;
      smart_factor->add(gtsam::Point2(pt.x, pt.y), obs->frame->id);
    }
    graph.add(smart_factor);
    smart_factor_count++;
  }
  std::cout << "Added " << smart_factor_count << " smart factors" << std::endl;

  // initialize values off from ground truth
  gtsam::Pose3 gt_offset(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25),
                         gtsam::Point3(0.05, -0.10, 0.20));
  for (auto& frame : frames) {
    Pose3 gt_pose = NewerCollegeGroundTruth::At(frame->timestamp);
    estimate.insert(frame->id, ToGtsamPose(gt_pose).compose(gt_offset));
  }

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, estimate);
  gtsam::Values result = optimizer.optimize();
  result.print("result");
}
