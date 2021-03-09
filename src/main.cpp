#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>
#include <newer_college_ground_truth.h>
#include <thread>
#include <chrono>
#include "controller.h"
#include "feature_extractor.h"
#include "global_params.h"
#include "ros_conversions.h"

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "orb_test");
  ros::NodeHandle nh;

  GlobalParams::LoadParams(nh);
  NewerCollegeGroundTruth::LoadFromFile(GlobalParams::GroundTruthFile());

  auto matches_pub = nh.advertise<sensor_msgs::Image>("/matches_image", 1000);
  auto tracks_pub = nh.advertise<sensor_msgs::Image>("/tracks_image", 1000);
  auto pose_pub = nh.advertise<nav_msgs::Odometry>("/pose", 1000);
  auto gt_pub = nh.advertise<nav_msgs::Odometry>("/ground_truth", 1000);
  auto landmarks_pub = nh.advertise<visualization_msgs::MarkerArray>("/landmarks", 1000);
  FeatureExtractor feature_extractor(matches_pub, tracks_pub, 20);
  Controller controller(feature_extractor, pose_pub, landmarks_pub);
  // san raf
  // auto sub = nh.subscribe("/camera/image_mono", 1000,
  // &Controller::imageCallback, &controller); newer college
  auto sub = nh.subscribe("output/image", 1000, &Controller::imageCallback, &controller);

  ROS_INFO("Starting up");

  for (auto& gt_pose : NewerCollegeGroundTruth::GetAllPoses())
  {
    if (gt_pose.first > 1583836735.573982) {
      break;
    }
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.stamp = ros::Time(gt_pose.first);
    odometry_msg.header.frame_id = "world";
    odometry_msg.pose.pose = ToPoseMsg(gt_pose.second);
    gt_pub.publish(odometry_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
  }

  ros::spin();
  return 0;
}
