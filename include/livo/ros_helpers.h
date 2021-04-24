#ifndef ORB_TEST_INCLUDE_LIVO_ROS_HELPERS_H_
#define ORB_TEST_INCLUDE_LIVO_ROS_HELPERS_H_

#include <ros/ros.h>

#include <vector>
#include <map>
#include "pose3_stamped.h"
#include "point3.h"

namespace ros_helpers
{
void PublishPoses(const std::vector<Pose3Stamped>& poses, const ros::Publisher& publisher,
                  const ros::Publisher& pose_pub);

void PublishLandmarks(const std::map<int, Point3>& landmarks, double timestamp, const ros::Publisher& publisher);

}  // namespace ros_helpers
#endif  // ORB_TEST_INCLUDE_LIVO_ROS_HELPERS_H_
