#include "smoother.h"
#include "key_point_observation.h"
#include "frame.h"
#include "gtsam_conversions.h"

#include <iostream>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/Values.h>
#include <pose3_stamped.h>
#include <global_params.h>

void Smoother::Initialize(const std::vector<std::shared_ptr<Frame>>& frames, std::vector<shared_ptr<Track>> tracks,
                          std::vector<Pose3Stamped>& pose_estimates, std::vector<Point3>& landmark_estimates)
{
  std::cout << "Let's process those" << tracks.size() << " tracks!" << std::endl;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values estimate;

  auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.1)).finished());

  // Add prior on first pose
  gtsam::Pose3 gtsam_prior_0(gtsam::Rot3(), gtsam::Point3::Zero());
  graph.addPrior(0, gtsam_prior_0, prior_noise);
  gtsam_prior_0.print("prior 0");

  // Add another prior on the 2nd pose to define scale
  gtsam::Pose3 gtsam_prior_1(gtsam::Rot3(), gtsam::Point3(0.1, 0, 0));
  graph.addPrior(1, gtsam_prior_1, prior_noise);
  gtsam_prior_1.print("prior 1");

  // san raf RESIZE_FACTOR=0.5
  // Cal3_S2::shared_ptr K(new Cal3_S2(593.690871957, 593.74699226, 0.0,
  // 388.42480338, 274.84471313));

  // newer college RESIZE_FACTOR=1
  gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(GlobalParams::CamFx(), GlobalParams::CamFy(), 0.0,
                                                  GlobalParams::CamU0(), GlobalParams::CamV0()));

  auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  auto body_P_sensor = gtsam::Pose3(gtsam::Rot3::Ypr(-M_PI / 2, 0, -M_PI / 2), gtsam::Point3::Zero());
  for (auto& track : tracks)
  {
    if (track->features.size() < 30)
    {
      continue;
    }
    std::cout << "adding landmark with " << track->features.size() << " observations" << std::endl;
    SmartFactor::shared_ptr smart_factor(new SmartFactor(measurementNoise, K, body_P_sensor));
    for (auto& feature : track->features)
    {
      auto pt = feature.pt;
      smart_factor->add(gtsam::Point2(pt.x, pt.y), feature.frame->id);
    }
    smart_factors_[track->id] = smart_factor;
    graph.add(smart_factor);
  }
  std::cout << "Added " << smart_factors_.size() << " smart factors" << std::endl;

  // initialize values off from ground truth
  gtsam::Pose3 gt_offset(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25), gtsam::Point3(0.05, -0.10, 0.20));
  // or exactly on ground truth?
  // gtsam::Pose3 gt_offset(gtsam::Rot3(), gtsam::Point3::Zero());
  std::vector<gtsam::Pose3> initial_poses;
  initial_poses.push_back(gtsam_prior_0);
  initial_poses.push_back(gtsam_prior_1);
  gtsam::Pose3 increment(gtsam::Rot3(), gtsam::Point3(0.1, 0, 0));
  for (auto& frame : frames)
  {
    auto gtsam_pose = initial_poses.back().compose(increment);
    estimate.insert(frame->id, gtsam_pose);
    initial_poses.push_back(gtsam_pose);
  }

  auto result = isam2.update(graph, estimate);
  result.print("result");

  for (auto& frame : frames)
  {
    Pose3Stamped poseStamped{ .pose = ToPose(isam2.calculateEstimate<gtsam::Pose3>(frame->id)),
                              .stamp = frame->timestamp };
    pose_estimates.push_back(poseStamped);
  }

  for (const auto& smart_factor_pair : smart_factors_)
  {
    boost::optional<gtsam::Point3> point = smart_factor_pair.second->point();
    if (point)
    {  // ignore if boost::optional returns nullptr
      landmark_estimates.push_back(ToPoint(*point));
    }
  }
}

Smoother::Smoother() : isam2()
{
}
