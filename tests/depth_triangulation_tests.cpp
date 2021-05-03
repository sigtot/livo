#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>

#include "depth_triangulation.h"

TEST(PixelAndDepthToPoint3, ReturnsCorrectPoint3)
{
  // Set up scene
  gtsam::Point3 ground_truth_point(7.0, 0.0, 1.0);
  gtsam::Pose3 camera_pose(gtsam::Rot3::Ypr(0.5, 0.1, 0.1), gtsam::Point3(5.0, 0.0, 1.0));
  gtsam::Pose3 body_p_cam(gtsam::Rot3::Ypr(-M_PI_2, 0.0, -M_PI_2), gtsam::Point3::Zero());
  boost::shared_ptr<gtsam::Cal3_S2> K(new gtsam::Cal3_S2(431.38739114, 430.24961762, 0.0, 427.4407802, 238.52694868));
  gtsam::PinholeCamera<gtsam::Cal3_S2> camera(camera_pose * body_p_cam, *K);

  // Obtain pixel and depth measurements
  gtsam::Point2 pixel = camera.project(ground_truth_point);
  double depth = ground_truth_point.distance(camera_pose.translation());

  std::cout << pixel << std::endl;
  std::cout << "d = " << depth << std::endl;

  gtsam::Point3 estimated_point = DepthTriangulation::PixelAndDepthToPoint3(pixel, depth, K, body_p_cam, camera_pose);

  ASSERT_EQ(estimated_point, ground_truth_point);
}
