#ifndef ORB_TEST_SRC_ROS_CONVERSIONS_H_
#define ORB_TEST_SRC_ROS_CONVERSIONS_H_

#include <geometry_msgs/Pose.h>
#include "pose3.h"

geometry_msgs::Pose ToPoseMsg(Pose3 pose) {
  geometry_msgs::Pose pose_msg;

  pose_msg.orientation.x = pose.rot.x;
  pose_msg.orientation.y = pose.rot.y;
  pose_msg.orientation.z = pose.rot.z;
  pose_msg.orientation.w = pose.rot.w;

  pose_msg.position.x = pose.point.x;
  pose_msg.position.y = pose.point.y;
  pose_msg.position.z = pose.point.z;

  return pose_msg;
}

#endif  // ORB_TEST_SRC_ROS_CONVERSIONS_H_
