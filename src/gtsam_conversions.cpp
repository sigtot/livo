#include "gtsam_conversions.h"

#include <opencv2/core/eigen.hpp>
gtsam::Pose3 ToGtsamPose(Pose3 pose)
{
  return gtsam::Pose3(gtsam::Rot3::Quaternion(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z),
                      gtsam::Point3(pose.point.x, pose.point.y, pose.point.z));
}
Pose3 ToPose(const gtsam::Pose3& gtsam_pose)
{
  return Pose3{
    .point = Point3{ .x = gtsam_pose.x(), .y = gtsam_pose.y(), .z = gtsam_pose.z() },
    .rot =
        Rot3{
            .x = gtsam_pose.rotation().toQuaternion().x(),
            .y = gtsam_pose.rotation().toQuaternion().y(),
            .z = gtsam_pose.rotation().toQuaternion().z(),
            .w = gtsam_pose.rotation().toQuaternion().w(),
        },
  };
}
Point3 ToPoint(const gtsam::Point3& gtsam_point)
{
  return Point3{ .x = gtsam_point.x(), .y = gtsam_point.y(), .z = gtsam_point.z() };
}

gtsam::Matrix3 ToMatrix3(const cv::Mat& mat)
{
  gtsam::Matrix3 matrix3;
  cv::cv2eigen(mat, matrix3);
  return matrix3;
}
