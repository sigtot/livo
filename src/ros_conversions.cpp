#include "ros_conversions.h"

geometry_msgs::Point ToPointMsg(Point3 point)
{
  geometry_msgs::Point point_msg;

  point_msg.x = point.x;
  point_msg.y = point.y;
  point_msg.z = point.z;

  return point_msg;
}
geometry_msgs::Pose ToPoseMsg(Pose3 pose)
{
  geometry_msgs::Pose pose_msg;

  pose_msg.orientation.x = pose.rot.x;
  pose_msg.orientation.y = pose.rot.y;
  pose_msg.orientation.z = pose.rot.z;
  pose_msg.orientation.w = pose.rot.w;

  pose_msg.position = ToPointMsg(pose.point);

  return pose_msg;
}
