#ifndef ORB_TEST_CONTROLLER_H
#define ORB_TEST_CONTROLLER_H

#include <ros/ros.h>
#include "feature_extractor.h"
#include "smoother.h"

class Controller {
 private:
  FeatureExtractor& frontend;
  ros::Publisher pose_publisher_;
  ros::Publisher landmark_publisher_;

 public:
  explicit Controller(FeatureExtractor& frontend, ros::Publisher& posePublisher,
                      ros::Publisher& landmarkPublisher);

  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
};

#endif
