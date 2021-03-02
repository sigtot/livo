#include "controller.h"
#include "frame.h"
#include "ros_conversions.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pose3_stamped.h>

#include <chrono>
#include <thread>

Controller::Controller(FeatureExtractor& frontend,
                       ros::Publisher& posePublisher,
                       ros::Publisher& landmarkPublisher)
    : frontend(frontend),
      pose_publisher_(posePublisher),
      landmark_publisher_(landmarkPublisher) {}

void Controller::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
  shared_ptr<Frame> new_frame = frontend.imageCallback(msg);
  frontend.PublishLandmarkTracksImage();
  std::cout << "Landmark count: " << frontend.GetLandmarkCount() << std::endl;

  if (frontend.GetFrameCount() > 100) {
    std::vector<Pose3Stamped> pose_estimates;
    std::vector<Point3> landmark_estimates;
    Smoother::SmoothBatch(frontend.GetFrames(), frontend.GetLandmarks(),
                          pose_estimates, landmark_estimates);
    std::cout << "poses:" << std::endl;
    for (auto& pose_stamped : pose_estimates) {
      std::cout << "[" << pose_stamped.pose.point.x << ", "
                << pose_stamped.pose.point.y << ", "
                << pose_stamped.pose.point.z << "]" << std::endl;
      nav_msgs::Odometry odometry_msg;
      odometry_msg.header.stamp = ros::Time(pose_stamped.stamp);
      odometry_msg.pose.pose = ToPoseMsg(pose_stamped.pose);
      odometry_msg.header.frame_id = "map";
      pose_publisher_.publish(odometry_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "landmarks:" << std::endl;
    visualization_msgs::MarkerArray markerArray;
    for (int i = 0; i < landmark_estimates.size(); ++i) {
      auto landmark = landmark_estimates[i];
      std::cout << "[" << landmark.x << ", " << landmark.y << ", " << landmark.z
                << "]" << std::endl;

      visualization_msgs::Marker marker;
      marker.pose.position = ToPointMsg(landmark);

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;

      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;

      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.id = i;
      marker.ns = "landmarks";
      marker.header.stamp = ros::Time(new_frame->timestamp);
      marker.header.frame_id = "map";
      markerArray.markers.push_back(marker);
    }
    landmark_publisher_.publish(markerArray);
    exit(0);
  }
}
