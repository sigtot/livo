#ifndef ORB_TEST_CONTROLLER_H
#define ORB_TEST_CONTROLLER_H

#include <ros/ros.h>
#include "FeatureExtractor.h"
#include "Smoother.h"

class Controller {
 private:
  FeatureExtractor& frontend;
  Smoother& backend;
  ros::Publisher posePublisher;
  ros::Publisher landmarkPublisher;

 public:
  explicit Controller(FeatureExtractor& frontend, Smoother& backend,
                      ros::Publisher& posePublisher,
                      ros::Publisher& landmarkPublisher);

  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
};

#endif
