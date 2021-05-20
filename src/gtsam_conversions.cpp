#include "gtsam_conversions.h"

#include <opencv2/core/eigen.hpp>
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

gtsam::Rot3 ToGtsamRot(Rot3 rot)
{
  return gtsam::Rot3::Quaternion(rot.w, rot.x, rot.y, rot.z);
}

gtsam::Point3 ToGtsamPoint(Point3 point)
{
  return { point.x, point.y, point.z };
}

gtsam::Pose3 ToGtsamPose(Pose3 pose)
{
  return gtsam::Pose3(ToGtsamRot(pose.rot), ToGtsamPoint(pose.point));
}

Rot3 ToRot(const gtsam::Rot3& gtsam_rot)
{
  return Rot3{
    .x = gtsam_rot.toQuaternion().x(),
    .y = gtsam_rot.toQuaternion().y(),
    .z = gtsam_rot.toQuaternion().z(),
    .w = gtsam_rot.toQuaternion().w(),
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

cv::Mat FromMatrix3(const gtsam::Matrix3& matrix3)
{
  cv::Mat mat;
  cv::eigen2cv(matrix3, mat);
  return mat;
}

cv::Point2f ToCvPoint(const gtsam::Point2& point2)
{
  cv::Point2f cv_pt;
  cv_pt.x = static_cast<float>(point2.x());
  cv_pt.y = static_cast<float>(point2.y());
  return cv_pt;
}

