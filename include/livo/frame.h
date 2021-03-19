#ifndef ORB_TEST_FEATUREEXTRACTOR_CPP_FRAME_H_
#define ORB_TEST_FEATUREEXTRACTOR_CPP_FRAME_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include "key_point_observation.h"

struct Frame
{
  std::vector<std::shared_ptr<KeyPointObservation>> GetUnmatchedObservations();

  cv::Mat image;
  std::vector<std::shared_ptr<KeyPointObservation>> keypoint_observations;
  // Landmarks added in this frame. These were first observed in this frame.
  std::vector<std::weak_ptr<Landmark>> new_landmarks;
  int id;
  double timestamp;
  bool stationary;
};

#endif
