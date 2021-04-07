#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>
#include <newer_college_ground_truth.h>
#include <thread>
#include <chrono>
#include <imu_queue.h>
#include "controller.h"
#include "feature_extractor.h"
#include "global_params.h"
#include "queued_measurement_processor.h"
#include "ros_helpers.h"
#include "debug_image_publisher.h"


#include <memory>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "orb_test");
  ros::NodeHandle nh;

  GlobalParams::LoadParams(nh);
  NewerCollegeGroundTruth::LoadFromFile(GlobalParams::GroundTruthFile());

  auto debug_added_landmarks_image_pub = nh.advertise<sensor_msgs::Image>("/debug_added_landmarks_image", 1000);
  DebugImagePublisher::SetPublishers(debug_added_landmarks_image_pub);

  std::shared_ptr<IMUQueue> imu_queue = std::make_shared<IMUQueue>();
  auto imu_sub = nh.subscribe(GlobalParams::IMUSubTopic(), 1000, &IMUQueue::addMeasurement, &*imu_queue);

  auto matches_pub = nh.advertise<sensor_msgs::Image>("/matches_image", 1000);
  auto tracks_pub = nh.advertise<sensor_msgs::Image>("/tracks_image", 1000);
  auto path_pub = nh.advertise<nav_msgs::Path>("/pose", 1000);
  auto gt_pub = nh.advertise<nav_msgs::Path>("/ground_truth", 1000);
  auto landmarks_pub = nh.advertise<visualization_msgs::MarkerArray>("/landmarks", 1000);
  FeatureExtractor feature_extractor(matches_pub, tracks_pub, 20);
  Smoother smoother(imu_queue);
  Controller controller(feature_extractor, smoother, path_pub, landmarks_pub);

  QueuedMeasurementProcessor<boost::shared_ptr<sensor_msgs::Image>> queued_measurement_processor(
      std::bind(&Controller::imageCallback, &controller, std::placeholders::_1), 4);
  auto sub = nh.subscribe(GlobalParams::CameraSubTopic(), 1000,
                          &QueuedMeasurementProcessor<boost::shared_ptr<sensor_msgs::Image>>::addMeasurement,
                          &queued_measurement_processor);

  ROS_INFO("Starting up");

  auto gt_poses_map = NewerCollegeGroundTruth::GetAllPoses();
  std::vector<Pose3Stamped> gt_poses_vec;
  for (auto& pose_pair : gt_poses_map)
  {
    Pose3Stamped stamped{ .pose = pose_pair.second, .stamp = pose_pair.first };
    gt_poses_vec.push_back(stamped);
  }
  while (gt_pub.getNumSubscribers() == 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ros_helpers::PublishPoses(gt_poses_vec, gt_pub);

  ros::spin();
  return 0;
}
