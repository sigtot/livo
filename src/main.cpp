#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>
#include <newer_college_ground_truth.h>
#include <euroc_ground_truth_provider.h>
#include <chrono>
#include <imu_queue.h>
#include "controller.h"
#include "feature_extractor.h"
#include "global_params.h"
#include "queued_measurement_processor.h"
#include "ros_helpers.h"
#include "debug_image_publisher.h"
#include "debug_value_publisher.h"
#include "imu_ground_truth_smoother.h"

#include <memory>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "orb_test");
  ros::NodeHandle nh;

  GlobalParams::LoadParams(nh);
  if (!GlobalParams::GroundTruthFile().empty())
  {
    if(GlobalParams::GroundTruthProvider() == "newer_college")
    {
      auto ground_truth_provider = NewerCollegeGroundTruth(GlobalParams::GroundTruthFile());
      GroundTruth::Load(ground_truth_provider);
    }
    else if (GlobalParams::GroundTruthProvider() == "euroc")
    {
      auto ground_truth_provider = EurocGroundTruthProvider(GlobalParams::GroundTruthFile());
      GroundTruth::Load(ground_truth_provider);
    }
  }

  auto debug_added_landmarks_image_pub = nh.advertise<sensor_msgs::Image>("/debug_added_landmarks_image", 1000);
  auto reprojection_error_image_pub = nh.advertise<sensor_msgs::Image>("/debug_reprojection_error_image", 1000);
  DebugImagePublisher::SetPublishers(debug_added_landmarks_image_pub, reprojection_error_image_pub);

  DebugValuePublisher::SetPublishers(nh);

  std::shared_ptr<IMUQueue> imu_queue = std::make_shared<IMUQueue>();
  auto imu_sub = nh.subscribe(GlobalParams::IMUSubTopic(), 1000, &IMUQueue::addMeasurement, &*imu_queue);

  auto matches_pub = nh.advertise<sensor_msgs::Image>("/matches_image", 1000);
  auto tracks_pub = nh.advertise<sensor_msgs::Image>("/tracks_image", 1000);
  auto path_pub = nh.advertise<nav_msgs::Path>("/pose", 1000);
  auto posearr_pub = nh.advertise<geometry_msgs::PoseArray>("/pose_array", 1000);
  auto gt_posearr_pub = nh.advertise<geometry_msgs::PoseArray>("/gt_pose_array", 1000, true);
  auto gt_pub = nh.advertise<nav_msgs::Path>("/ground_truth", 1000, true);
  auto landmarks_pub = nh.advertise<visualization_msgs::MarkerArray>("/landmarks", 1000);
  FeatureExtractor feature_extractor(matches_pub, tracks_pub, 20);
  Smoother smoother(imu_queue);
  IMUGroundTruthSmoother imu_ground_truth_smoother(imu_queue);
  Controller controller(feature_extractor, smoother, imu_ground_truth_smoother, path_pub, posearr_pub, landmarks_pub);

  QueuedMeasurementProcessor<boost::shared_ptr<sensor_msgs::Image>> queued_measurement_processor(
      std::bind(&Controller::imageCallback, &controller, std::placeholders::_1), 4);
  auto sub = nh.subscribe(GlobalParams::CameraSubTopic(), 1000,
                          &QueuedMeasurementProcessor<boost::shared_ptr<sensor_msgs::Image>>::addMeasurement,
                          &queued_measurement_processor);

  ROS_INFO("Starting up");
  if (!GlobalParams::GroundTruthFile().empty())
  {
    auto gt_poses_map = GroundTruth::GetAllPoses();
    std::vector<Pose3Stamped> gt_poses_vec;
    int i = 0;
    for (auto& pose_pair : gt_poses_map)
    {
      if (i % 50 == 0)
      {
        Pose3Stamped stamped{ .pose = pose_pair.second, .stamp = pose_pair.first };
        gt_poses_vec.push_back(stamped);
      }
      i++;
    }
    ros_helpers::PublishPoses(gt_poses_vec, gt_pub, gt_posearr_pub);
  }

  ROS_INFO("Ready and spinning");

  ros::spin();
  return 0;
}
