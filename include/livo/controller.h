#ifndef ORB_TEST_CONTROLLER_H
#define ORB_TEST_CONTROLLER_H

#include <ros/ros.h>
#include "feature_extractor.h"
#include "smoother.h"

class Controller
{
private:
  FeatureExtractor& frontend;
  Smoother& backend;
  ros::Publisher pose_publisher_;
  ros::Publisher landmark_publisher_;

public:
  explicit Controller(FeatureExtractor& frontend, Smoother& backend, ros::Publisher& posePublisher, ros::Publisher& landmarkPublisher);

  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
};

#endif
