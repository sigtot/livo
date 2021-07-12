#ifndef ORB_TEST_INCLUDE_LIVO_ROS_HELPERS_H_
#define ORB_TEST_INCLUDE_LIVO_ROS_HELPERS_H_

#include <ros/ros.h>

#include <vector>
#include <map>
#include <tf2_ros/transform_broadcaster.h>
#include "pose3_stamped.h"
#include "point3.h"
#include "landmark_result.h"

namespace ros_helpers
{
void PublishPathAndPoseArray(const std::vector<Pose3Stamped>& poses, const ros::Publisher& path_publisher,
                             const ros::Publisher& pose_array_publisher);

void PublishPoseArray(const std::vector<Pose3Stamped>& poses, const ros::Publisher& publisher);

void PublishPath(const std::vector<Pose3Stamped>& poses, const ros::Publisher& publisher);

void PublishLandmarks(const std::map<int, LandmarkResult>& landmarks, double timestamp,
                      const ros::Publisher& publisher);

void PublishTransform(const Pose3Stamped& pose_stamped, const std::string& frame_id, const std::string& child_frame_id,
                      tf2_ros::TransformBroadcaster& broadcaster);

}  // namespace ros_helpers
#endif  // ORB_TEST_INCLUDE_LIVO_ROS_HELPERS_H_
