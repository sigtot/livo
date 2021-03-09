#ifndef ORB_TEST_SRC_GTSAM__CONVERSIONS_H
#define ORB_TEST_SRC_GTSAM__CONVERSIONS_H

#include "pose3.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>

gtsam::Pose3 ToGtsamPose(Pose3 pose);

Pose3 ToPose(const gtsam::Pose3& gtsam_pose);

Point3 ToPoint(const gtsam::Point3& gtsam_point);

Pose3 OrientAlongZ(const Pose3& pose);

#endif
