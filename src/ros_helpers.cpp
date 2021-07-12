#include "ros_helpers.h"
#include "ros_conversions.h"
#include "landmark_result.h"

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>

namespace ros_helpers
{
void PublishPathAndPoseArray(const std::vector<Pose3Stamped>& poses, const ros::Publisher& path_publisher,
                             const ros::Publisher& pose_array_publisher)
{
  PublishPath(poses, path_publisher);
  PublishPoseArray(poses, pose_array_publisher);
}

void PublishPoseArray(const std::vector<Pose3Stamped>& poses, const ros::Publisher& publisher)
{
  if (poses.empty())
  {
    return;  // Nothing to publish
  }
  geometry_msgs::PoseArray pose_array_msg;
  for (const auto& pose_stamped : poses)
  {
    geometry_msgs::PoseStamped stamped_pose_msg;
    stamped_pose_msg.pose = ToPoseMsg(pose_stamped.pose);
    stamped_pose_msg.header.stamp = ros::Time(pose_stamped.stamp);
    stamped_pose_msg.header.frame_id = "world";
    pose_array_msg.poses.push_back(stamped_pose_msg.pose);
  }
  pose_array_msg.header.frame_id = "world";
  pose_array_msg.header.stamp = ros::Time(poses.back().stamp);
  publisher.publish(pose_array_msg);
}

void PublishPath(const std::vector<Pose3Stamped>& poses, const ros::Publisher& publisher)
{
  if (poses.empty())
  {
    return;  // Nothing to publish
  }
  nav_msgs::Path path_msg;
  for (const auto& pose_stamped : poses)
  {
    geometry_msgs::PoseStamped stamped_pose_msg;
    stamped_pose_msg.pose = ToPoseMsg(pose_stamped.pose);
    stamped_pose_msg.header.stamp = ros::Time(pose_stamped.stamp);
    stamped_pose_msg.header.frame_id = "world";
    path_msg.poses.push_back(stamped_pose_msg);
  }
  path_msg.header.frame_id = "world";
  path_msg.header.stamp = ros::Time(poses.back().stamp);
  publisher.publish(path_msg);
}

void PublishLandmarks(const std::map<int, LandmarkResult>& landmarks, double timestamp, const ros::Publisher& publisher)
{
  if (landmarks.empty())
  {
    return;  // Nothing to publish
  }
  visualization_msgs::MarkerArray markerArray;
  std::cout << "got " << landmarks.size() << " landmarks from smoother" << std::endl;
  {
    visualization_msgs::Marker marker;
    marker.id = 0;
    marker.ns = "landmarks";
    marker.action = visualization_msgs::Marker::DELETEALL;
    markerArray.markers.push_back(marker);
  }
  for (auto& landmark_pair : landmarks)
  {
    auto landmark = landmark_pair.second;

    visualization_msgs::Marker marker;
    marker.pose.position = ToPointMsg(landmark.pt);

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 0.0f;
    marker.color.g = landmark_pair.second.type == SmartFactorType ? 1.0f : 0.0f;
    marker.color.b = landmark_pair.second.type == SmartFactorType ? 0.0f : 1.0f;
    marker.color.a = landmark_pair.second.active ? 1.0f : 0.15f;

    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.id = landmark_pair.first;
    marker.ns = "landmarks";
    marker.header.stamp = ros::Time(timestamp);
    marker.header.frame_id = "world";
    markerArray.markers.push_back(marker);
  }
  publisher.publish(markerArray);
}

void PublishTransform(const Pose3Stamped& pose_stamped, const std::string& frame_id, const std::string& child_frame_id,
                      tf2_ros::TransformBroadcaster& broadcaster)
{
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = frame_id;
  transformStamped.child_frame_id = child_frame_id;
  transformStamped.transform.translation.x = pose_stamped.pose.point.x;
  transformStamped.transform.translation.y = pose_stamped.pose.point.y;
  transformStamped.transform.translation.z = pose_stamped.pose.point.z;

  tf2::Quaternion q;
  transformStamped.transform.rotation.x = pose_stamped.pose.rot.x;
  transformStamped.transform.rotation.y = pose_stamped.pose.rot.y;
  transformStamped.transform.rotation.z = pose_stamped.pose.rot.z;
  transformStamped.transform.rotation.w = pose_stamped.pose.rot.w;

  broadcaster.sendTransform(transformStamped);
}

}  // namespace ros_helpers
