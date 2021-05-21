#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>

#include "gtsam_helpers.h"

TEST(ComputeParallaxWithOpenCV, TranslationOnly_GivesPixelParallax)
{
  auto body_p_cam = gtsam::Pose3(gtsam::Rot3::Ypr(-M_PI_2, 0., -M_PI_2), gtsam::Point3::Zero());
  auto K = gtsam::make_shared<gtsam::Cal3_S2>(650., 650., 0., 315., 315.);

  gtsam::Point3 landmark(3, 1., 1.);
  gtsam::Pose3 pose1;
  gtsam::Pose3 pose2(gtsam::Rot3(), gtsam::Point3(1.0, 0., 0.));

  auto point1 = gtsam::PinholeCamera<gtsam::Cal3_S2>(pose1 * body_p_cam, *K).project(landmark);
  auto point2 = gtsam::PinholeCamera<gtsam::Cal3_S2>(pose2 * body_p_cam, *K).project(landmark);

  auto b1_R_b2 = pose1.between(pose2).rotation();

  auto parallax = ComputeParallaxWithOpenCV(point1, point2, b1_R_b2, K, body_p_cam.rotation());
  EXPECT_NEAR(parallax, (point1 - point2).norm(), 1e-5);
}

TEST(ComputeParallaxWithOpenCV, RotationOnly_GivesZeroParallax)
{
  auto body_p_cam = gtsam::Pose3(gtsam::Rot3::Ypr(-M_PI_2, 0., -M_PI_2), gtsam::Point3::Zero());
  auto K = gtsam::make_shared<gtsam::Cal3_S2>(650., 650., 0., 315., 315.);

  gtsam::Point3 landmark(3, 1., 1.);
  gtsam::Pose3 pose1;                                                          // w_p_b1
  gtsam::Pose3 pose2(gtsam::Rot3::Ypr(0.5, 0.0, 0.0), gtsam::Point3::Zero());  // w_p_b2

  auto point1 = gtsam::PinholeCamera<gtsam::Cal3_S2>(pose1 * body_p_cam, *K).project(landmark);
  auto point2 = gtsam::PinholeCamera<gtsam::Cal3_S2>(pose2 * body_p_cam, *K).project(landmark);

  auto b1_R_b2 = pose1.between(pose2).rotation();

  auto parallax = ComputeParallaxWithOpenCV(point1, point2, b1_R_b2, K, body_p_cam.rotation());
  EXPECT_NEAR(parallax, 0.0, 1e-4);
}

TEST(ComputeParallaxWithOpenCV, FarPoints_GiveLowParallax)
{
  auto body_p_cam = gtsam::Pose3(gtsam::Rot3::Ypr(-M_PI_2, 0., -M_PI_2), gtsam::Point3::Zero());
  auto K = gtsam::make_shared<gtsam::Cal3_S2>(650., 650., 0., 315., 315.);

  gtsam::Point3 landmark_close(3, 1., 1.);
  gtsam::Point3 landmark_far(30, 1., 1.);
  gtsam::Pose3 pose1;                                                               // w_p_b1
  gtsam::Pose3 pose2(gtsam::Rot3::Ypr(0.5, 0.0, 0.0), gtsam::Point3(1.0, 0., 0.));  // w_p_b2

  auto close_point1 = gtsam::PinholeCamera<gtsam::Cal3_S2>(pose1 * body_p_cam, *K).project(landmark_close);
  auto close_point2 = gtsam::PinholeCamera<gtsam::Cal3_S2>(pose2 * body_p_cam, *K).project(landmark_close);

  auto far_point1 = gtsam::PinholeCamera<gtsam::Cal3_S2>(pose1 * body_p_cam, *K).project(landmark_far);
  auto far_point2 = gtsam::PinholeCamera<gtsam::Cal3_S2>(pose2 * body_p_cam, *K).project(landmark_far);

  auto b1_R_b2 = pose1.between(pose2).rotation();

  auto close_parallax = ComputeParallaxWithOpenCV(close_point1, close_point2, b1_R_b2, K, body_p_cam.rotation());
  auto far_parallax = ComputeParallaxWithOpenCV(far_point1, far_point2, b1_R_b2, K, body_p_cam.rotation());

  EXPECT_GT(close_parallax, 120.);
  EXPECT_LT(far_parallax, 2.);
}
