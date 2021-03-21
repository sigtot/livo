#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>
#include <newer_college_ground_truth.h>
#include <chrono>
#include <imu_queue.h>
#include "controller.h"
#include "feature_extractor.h"
#include "global_params.h"

#include <memory>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "orb_test");
  ros::NodeHandle nh;

  GlobalParams::LoadParams(nh);
  NewerCollegeGroundTruth::LoadFromFile(GlobalParams::GroundTruthFile());

  std::shared_ptr<IMUQueue> imu_queue = std::make_shared<IMUQueue>();
  auto imu_sub = nh.subscribe(GlobalParams::IMUSubTopic(), 1000, &IMUQueue::addMeasurement, &*imu_queue);

  auto matches_pub = nh.advertise<sensor_msgs::Image>("/matches_image", 1000);
  auto tracks_pub = nh.advertise<sensor_msgs::Image>("/tracks_image", 1000);
  auto pose_pub = nh.advertise<nav_msgs::Odometry>("/pose", 1000);
  auto gt_pub = nh.advertise<nav_msgs::Odometry>("/ground_truth", 1000);
  auto landmarks_pub = nh.advertise<visualization_msgs::MarkerArray>("/landmarks", 1000);
  FeatureExtractor feature_extractor(matches_pub, tracks_pub, 20);
  Smoother smoother(imu_queue);
  Controller controller(feature_extractor, smoother, pose_pub, landmarks_pub);
  // san raf
  // auto sub = nh.subscribe("/camera/image_mono", 1000,
  // &Controller::imageCallback, &controller); newer college
  auto sub = nh.subscribe(GlobalParams::CameraSubTopic(), 1000, &Controller::imageCallback, &controller);

  ROS_INFO("Starting up");

  ros::spin();
  return 0;
}
