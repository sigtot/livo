#ifndef ORB_TEST_SRC_GTSAM__CONVERSIONS_H
#define ORB_TEST_SRC_GTSAM__CONVERSIONS_H

#include "pose3.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <opencv2/core/core.hpp>

gtsam::Pose3 ToGtsamPose(Pose3 pose);

Pose3 ToPose(const gtsam::Pose3& gtsam_pose);

Point3 ToPoint(const gtsam::Point3& gtsam_point);

gtsam::Matrix3 ToMatrix3(const cv::Mat& mat);

#endif
