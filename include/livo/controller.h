#ifndef ORB_TEST_CONTROLLER_H
#define ORB_TEST_CONTROLLER_H

#include <ros/ros.h>
#include "feature_extractor.h"
#include "smoother.h"
#include "point3.h"
#include "pose3_stamped.h"
#include "imu_ground_truth_smoother.h"

class Controller
{
private:
  FeatureExtractor& frontend_;
  Smoother& backend_;
  IMUGroundTruthSmoother& imu_ground_truth_smoother_;
  ros::Publisher path_publisher_;
  ros::Publisher landmark_publisher_;
  ros::Publisher pose_arr_publisher_;

public:
  explicit Controller(FeatureExtractor& frontend, Smoother& backend, IMUGroundTruthSmoother& imu_ground_truth_smoother,
                      ros::Publisher& path_publisher, ros::Publisher& pose_arr_publisher,
                      ros::Publisher& landmark_publisher);

  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
  void PublishPoses(const std::vector<Pose3Stamped>& poses);
  void PublishLandmarks(const std::map<int, Point3>& landmarks, double timestamp);
};

#endif
