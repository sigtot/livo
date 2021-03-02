#include "controller.h"
#include "frame.h"
#include "ros_conversions.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
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
      std::cout << "[" << pose_stamped.pose.point.x << ", " << pose_stamped.pose.point.y << ", "
                << pose_stamped.pose.point.z << "]" << std::endl;
      nav_msgs::Odometry odometry_msg;
      odometry_msg.header.stamp = ros::Time(pose_stamped.stamp);
      odometry_msg.pose.pose = ToPoseMsg(pose_stamped.pose);
      odometry_msg.header.frame_id = "map";
      pose_publisher_.publish(odometry_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "landmarks:" << std::endl;
    for (auto& landmark : landmark_estimates) {
      std::cout << "[" << landmark.x << ", " << landmark.y << ", " << landmark.z
                << "]" << std::endl;
    }
    exit(0);
  }
}
