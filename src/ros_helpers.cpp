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
void PublishPoses(const std::vector<Pose3Stamped>& poses, const ros::Publisher& publisher, const ros::Publisher& pa_pub)
{
  if (poses.empty())
  {
    return;  // Nothing to publish
  }
  nav_msgs::Path pathMsg;
  geometry_msgs::PoseArray pose_arr;
  for (auto& pose_stamped : poses)
  {
    geometry_msgs::PoseStamped stampedPoseMsg;
    stampedPoseMsg.pose = ToPoseMsg(pose_stamped.pose);
    stampedPoseMsg.header.stamp = ros::Time(pose_stamped.stamp);
    stampedPoseMsg.header.frame_id = "world";
    pathMsg.poses.push_back(stampedPoseMsg);
    pose_arr.poses.push_back(stampedPoseMsg.pose);
  }
  pathMsg.header.frame_id = "world";
  pathMsg.header.stamp = ros::Time(poses.back().stamp);
  pose_arr.header.frame_id = "world";
  pose_arr.header.stamp = pathMsg.header.stamp;
  publisher.publish(pathMsg);
  pa_pub.publish(pose_arr);
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
