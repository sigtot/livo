#ifndef ORB_TEST_FEATUREEXTRACTOR_CPP_FRAME_H_
#define ORB_TEST_FEATUREEXTRACTOR_CPP_FRAME_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

struct Frame
{
  cv::Mat image;
  int id;
  double timestamp;
  bool stationary;
};

#endif
