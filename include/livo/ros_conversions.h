#ifndef ORB_TEST_SRC_ROS_CONVERSIONS_H_
#define ORB_TEST_SRC_ROS_CONVERSIONS_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "pose3.h"
#include "point3.h"

geometry_msgs::Point ToPointMsg(Point3 point);

geometry_msgs::Pose ToPoseMsg(Pose3 pose);

#endif  // ORB_TEST_SRC_ROS_CONVERSIONS_H_
