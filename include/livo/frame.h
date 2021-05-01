#ifndef ORB_TEST_FEATUREEXTRACTOR_CPP_FRAME_H_
#define ORB_TEST_FEATUREEXTRACTOR_CPP_FRAME_H_

#include "lidar_frame.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <map>
#include <memory>
#include <boost/optional.hpp>

// Forward declaration b/c of circular dependency
struct Feature;

typedef std::pair<std::weak_ptr<Feature>, std::weak_ptr<Feature>> FeatureMatch;

struct Frame
{
  cv::Mat image;
  int id;
  std::map<int, std::weak_ptr<Feature>> features;
  double timestamp;
  bool stationary;

  std::vector<FeatureMatch> GetFeatureMatches(const std::shared_ptr<Frame>& target);
};

#endif
