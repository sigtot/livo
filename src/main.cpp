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
#include "new_smoother.h"
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
    if (GlobalParams::GroundTruthProvider() == "newer_college")
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
  auto depth_image_pub = nh.advertise<sensor_msgs::Image>("/debug_depth_image", 1000);
  DebugImagePublisher::SetPublishers(debug_added_landmarks_image_pub, reprojection_error_image_pub, depth_image_pub);

  DebugValuePublisher::SetPublishers(nh);

  std::shared_ptr<IMUQueue> imu_queue = std::make_shared<IMUQueue>();
  auto imu_sub = nh.subscribe(GlobalParams::IMUSubTopic(), 1000, &IMUQueue::addMeasurement, &*imu_queue);

  auto tracks_pub = nh.advertise<sensor_msgs::Image>("/tracks_image", 1000);
  auto path_pub = nh.advertise<nav_msgs::Path>("/pose", 1000);
  auto posearr_pub = nh.advertise<geometry_msgs::PoseArray>("/pose_array", 1000);
  auto gt_posearr_pub = nh.advertise<geometry_msgs::PoseArray>("/gt_pose_array", 1000, true);
  auto gt_pub = nh.advertise<nav_msgs::Path>("/ground_truth", 1000, true);
  auto landmarks_pub = nh.advertise<visualization_msgs::MarkerArray>("/landmarks", 1000);
  LidarFrameManager lidar_frame_manager;
  FeatureExtractor feature_extractor(tracks_pub, lidar_frame_manager);
  Smoother smoother(imu_queue);
  NewSmoother new_smoother(imu_queue);
  IMUGroundTruthSmoother imu_ground_truth_smoother(imu_queue);
  Controller controller(feature_extractor, lidar_frame_manager, smoother, new_smoother, imu_ground_truth_smoother, path_pub,
                        posearr_pub, landmarks_pub);

  QueuedMeasurementProcessor<boost::shared_ptr<sensor_msgs::Image>> image_messages_processor(
      std::bind(&Controller::imageCallback, &controller, std::placeholders::_1), 10);
  auto img_sub = nh.subscribe(GlobalParams::CameraSubTopic(), 1000,
                              &QueuedMeasurementProcessor<boost::shared_ptr<sensor_msgs::Image>>::addMeasurement,
                              &image_messages_processor);

  QueuedMeasurementProcessor<boost::shared_ptr<sensor_msgs::PointCloud2>> lidar_messages_processor(
      std::bind(&Controller::LidarCallback, &controller, std::placeholders::_1), 1);
  auto lidar_sub =
      nh.subscribe(GlobalParams::LidarSubTopic(), 1000,
                   &QueuedMeasurementProcessor<boost::shared_ptr<sensor_msgs::PointCloud2>>::addMeasurement,
                   &lidar_messages_processor);

  ROS_INFO("Starting up");
  if (!GlobalParams::GroundTruthFile().empty())
  {
    auto gt_poses_map = GroundTruth::GetAllPoses();
    std::vector<Pose3Stamped> gt_poses_vec;
    int i = 0;
    for (auto& pose_pair : gt_poses_map)
    {
      if (i % 3 == 0)
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
