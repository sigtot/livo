#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>
#include "Controller.h"
#include "FeatureExtractor.h"

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "orb_test");
  ros::NodeHandle nh;

  auto matches_pub = nh.advertise<sensor_msgs::Image>("/matches_image", 1000);
  auto tracks_pub = nh.advertise<sensor_msgs::Image>("/tracks_image", 1000);
  auto pose_pub = nh.advertise<nav_msgs::Odometry>("/pose", 1000);
  auto landmarks_pub =
      nh.advertise<visualization_msgs::MarkerArray>("/landmarks", 1000);
  FeatureExtractor feature_extractor(matches_pub, tracks_pub, 20);
  Controller controller(feature_extractor, pose_pub, landmarks_pub);
  // san raf
  // auto sub = nh.subscribe("/camera/image_mono", 1000,
  // &Controller::imageCallback, &controller); newer college
  auto sub = nh.subscribe("/camera/infra1/image_rect_raw", 1000,
                          &Controller::imageCallback, &controller);

  ROS_INFO("Starting up");

  ros::spin();
  return 0;
}
