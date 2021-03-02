#ifndef ORB_TEST_SRC_GTSAM__CONVERSIONS_H
#define ORB_TEST_SRC_GTSAM__CONVERSIONS_H

#include "pose3.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>

gtsam::Pose3 ToGtsamPose(Pose3 pose) {
  return gtsam::Pose3(
      gtsam::Rot3(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z),
      gtsam::Point3(pose.point.x, pose.point.y, pose.point.z));
}

Pose3 ToPose(const gtsam::Pose3& gtsam_pose) {
  return Pose3{
      .point =
          Point3{.x = gtsam_pose.x(), .y = gtsam_pose.y(), .z = gtsam_pose.z()},
      .rot =
          Rot3{
              .x = gtsam_pose.rotation().toQuaternion().x(),
              .y = gtsam_pose.rotation().toQuaternion().y(),
              .z = gtsam_pose.rotation().toQuaternion().z(),
              .w = gtsam_pose.rotation().toQuaternion().w(),
          },
  };
}

#endif
