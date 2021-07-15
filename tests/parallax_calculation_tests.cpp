#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>

#include "gtsam_helpers.h"

#include <cmath>

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

TEST(ComputeParallaxAngle, TranslationOnly_GivesSimplePythagoreanParallax)
{
  auto body_p_cam = gtsam::Pose3(gtsam::Rot3::Ypr(-M_PI_2, 0., -M_PI_2), gtsam::Point3::Zero());
  auto K = gtsam::make_shared<gtsam::Cal3_S2>(650., 650., 0., 315., 315.);

  gtsam::Point3 landmark(2.0, 0, 0);
  gtsam::Pose3 pose1;
  gtsam::Pose3 pose2(gtsam::Rot3(), gtsam::Point3(0., -1., 0.));

  auto camera1 = gtsam::PinholeCamera<gtsam::Cal3_S2>(pose1 * body_p_cam, *K);
  auto camera2 = gtsam::PinholeCamera<gtsam::Cal3_S2>(pose2 * body_p_cam, *K);

  // The setup forms a right triangle with baseline = 1 and distance from c1 to l = 2, hence parallax is atan2(1, 2).
  //   l
  //   . .
  // 2 .  .
  //   .   .
  //   c1 . c2
  //      1
  auto expected_parallax_rad = std::atan2(1.0, 2.0);
  auto expected_parallax = expected_parallax_rad * 180. / M_PI;

  auto point1 = camera1.project(landmark);
  auto point2 = camera2.project(landmark);

  auto parallax = ComputeParallaxAngle(point1, point2, pose1, pose2, K, body_p_cam);
  EXPECT_NEAR(parallax, expected_parallax, 1e-5);
}

TEST(ComputeParallaxAngle, RotationIsCancelled)
{
  auto body_p_cam = gtsam::Pose3(gtsam::Rot3::Ypr(-M_PI_2, 0., -M_PI_2), gtsam::Point3::Zero());
  auto K = gtsam::make_shared<gtsam::Cal3_S2>(650., 650., 0., 315., 315.);

  gtsam::Point3 landmark(2.0, 0, 0);
  gtsam::Pose3 pose1;
  // Add a little rotation to see if it's properly cancelled in the computation
  gtsam::Pose3 pose2(gtsam::Rot3::Ypr(-0.2, 0.1, -0.1), gtsam::Point3(0., -1., 0.));

  auto camera1 = gtsam::PinholeCamera<gtsam::Cal3_S2>(pose1 * body_p_cam, *K);
  auto camera2 = gtsam::PinholeCamera<gtsam::Cal3_S2>(pose2 * body_p_cam, *K);

  // The setup forms a right triangle with baseline = 1 and distance from c1 to l = 2, hence parallax is atan2(1, 2).
  //   l
  //   . .
  // 2 .  .
  //   .   .
  //   c1 . c2
  //      1
  // The result is the same as in the zero-rotation case because the rotation is cancelled
  auto expected_parallax_rad = std::atan2(1.0, 2.0);
  auto expected_parallax = expected_parallax_rad * 180. / M_PI;

  auto point1 = camera1.project(landmark);
  auto point2 = camera2.project(landmark);

  auto parallax = ComputeParallaxAngle(point1, point2, pose1, pose2, K, body_p_cam);
  EXPECT_NEAR(parallax, expected_parallax, 1e-5);
}

TEST(ComputeParallaxAngle, PureRotation_GivesZeroParallax)
{
  auto body_p_cam = gtsam::Pose3(gtsam::Rot3::Ypr(-M_PI_2, 0., -M_PI_2), gtsam::Point3::Zero());
  auto K = gtsam::make_shared<gtsam::Cal3_S2>(650., 650., 0., 315., 315.);

  gtsam::Point3 landmark(2.0, 0, 0);
  gtsam::Pose3 pose1;
  // Add only rotation. Zero baseline => zero parallax
  gtsam::Pose3 pose2(gtsam::Rot3::Ypr(-0.2, 0.1, -0.1), gtsam::Point3::Zero());

  auto camera1 = gtsam::PinholeCamera<gtsam::Cal3_S2>(pose1 * body_p_cam, *K);
  auto camera2 = gtsam::PinholeCamera<gtsam::Cal3_S2>(pose2 * body_p_cam, *K);

  auto point1 = camera1.project(landmark);
  auto point2 = camera2.project(landmark);

  auto parallax = ComputeParallaxAngle(point1, point2, pose1, pose2, K, body_p_cam);
  EXPECT_NEAR(parallax, 0., 1e-5);
}

TEST(ComputeParallaxAngle, FarPoint_GivesLowParallax)
{
  auto body_p_cam = gtsam::Pose3(gtsam::Rot3::Ypr(-M_PI_2, 0., -M_PI_2), gtsam::Point3::Zero());
  auto K = gtsam::make_shared<gtsam::Cal3_S2>(650., 650., 0., 315., 315.);

  gtsam::Point3 close_lmk(2.0, 0, 0);
  gtsam::Point3 far_lmk(30.0, 0, 0);
  gtsam::Pose3 pose1;
  gtsam::Pose3 pose2(gtsam::Rot3(), gtsam::Point3(0., -1., 0.));

  auto camera1 = gtsam::PinholeCamera<gtsam::Cal3_S2>(pose1 * body_p_cam, *K);
  auto camera2 = gtsam::PinholeCamera<gtsam::Cal3_S2>(pose2 * body_p_cam, *K);

  auto close_point1 = camera1.project(close_lmk);
  auto close_point2 = camera2.project(close_lmk);

  auto far_point1 = camera1.project(far_lmk);
  auto far_point2 = camera2.project(far_lmk);

  auto close_parallax = ComputeParallaxAngle(close_point1, close_point2, pose1, pose2, K, body_p_cam);
  auto far_parallax = ComputeParallaxAngle(far_point1, far_point2, pose1, pose2, K, body_p_cam);
  EXPECT_LT(far_parallax, close_parallax);
}
