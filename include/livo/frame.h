#ifndef ORB_TEST_FEATUREEXTRACTOR_CPP_FRAME_H_
#define ORB_TEST_FEATUREEXTRACTOR_CPP_FRAME_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <map>
#include <memory>

// Forward declaration b/c of circular dependency
struct Feature;

struct Frame
{
  cv::Mat image;
  int id;
  std::map<int, std::weak_ptr<Feature>> features;
  double timestamp;
  bool stationary;

  void GetFeatureMatches(const std::shared_ptr<Frame>& target,
                         std::vector<std::pair<std::shared_ptr<Feature>, std::shared_ptr<Feature>>>& matches);
};

#endif
