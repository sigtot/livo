#include "controller.h"
#include "frame.h"
#include "ros_conversions.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pose3_stamped.h>

#include <global_params.h>

Controller::Controller(FeatureExtractor& frontend, Smoother& backend, ros::Publisher& path_publisher,
                       ros::Publisher& landmark_publisher)
  : frontend(frontend), backend(backend), path_publisher_(path_publisher), landmark_publisher_(landmark_publisher)
{
}

void Controller::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  auto new_frame = frontend.lkCallback(msg);
  std::cout << "frame " << new_frame->id << std::endl;
  if (frontend.ReadyForInitialization())
  {
    std::cout << "R_H good. Ready for initialization!" << std::endl;
  }

  if (backend.GetStatus() != kLandmarksInitialized && frontend.ReadyForInitialization())
  {
    {
      std::vector<Pose3Stamped> pose_estimates;
      std::map<int, Point3> landmark_estimates;

      backend.InitializeLandmarks(frontend.GetKeyframeTransforms(), frontend.GetHighParallaxTracks(), pose_estimates,
                                  landmark_estimates);

      PublishPoses(pose_estimates);
      PublishLandmarks(landmark_estimates, new_frame->timestamp);
    }
    {
      std::vector<Pose3Stamped> pose_estimates;
      std::map<int, Point3> landmark_estimates;

      backend.InitializeIMU(frontend.GetKeyframeTransforms(), pose_estimates, landmark_estimates);

      PublishPoses(pose_estimates);
      PublishLandmarks(landmark_estimates, new_frame->timestamp);
    }
  }
  else if (backend.GetStatus() == kLandmarksInitialized &&
           frontend.GetNewestKeyframeTransform().frame1->id == backend.GetLastFrameId() &&
           frontend.GetNewestKeyframeTransform().frame2->id != backend.GetLastFrameId())
  {
    std::vector<Pose3Stamped> pose_estimates;
    std::map<int, Point3> landmark_estimates;
    backend.Update(frontend.GetNewestKeyframeTransform(), frontend.GetActiveHighParallaxTracks(), pose_estimates,
                   landmark_estimates);
    PublishPoses(pose_estimates);
    PublishLandmarks(landmark_estimates, new_frame->timestamp);
  }
}

void Controller::PublishPoses(const std::vector<Pose3Stamped>& poses)
{
  nav_msgs::Path pathMsg;
  for (auto& pose_stamped : poses)
  {
    geometry_msgs::PoseStamped stampedPoseMsg;
    stampedPoseMsg.pose = ToPoseMsg(pose_stamped.pose);
    stampedPoseMsg.header.stamp = ros::Time(pose_stamped.stamp);
    stampedPoseMsg.header.frame_id = "world";
    pathMsg.poses.push_back(stampedPoseMsg);
  }
  pathMsg.header.frame_id = "world";
  pathMsg.header.stamp = ros::Time(poses.back().stamp);
  path_publisher_.publish(pathMsg);
}

void Controller::PublishLandmarks(const std::map<int, Point3>& landmarks, double timestamp)
{
  visualization_msgs::MarkerArray markerArray;
  std::cout << "got " << landmarks.size() << " landmarks from smoother" << std::endl;
  for (auto& landmark_pair : landmarks)
  {
    auto landmark = landmark_pair.second;

    visualization_msgs::Marker marker;
    marker.pose.position = ToPointMsg(landmark);

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.id = landmark_pair.first;
    marker.ns = "landmarks";
    marker.header.stamp = ros::Time(timestamp);
    marker.header.frame_id = "world";
    markerArray.markers.push_back(marker);
  }
  landmark_publisher_.publish(markerArray);
}
