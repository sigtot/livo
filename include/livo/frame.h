#ifndef ORB_TEST_FEATUREEXTRACTOR_CPP_FRAME_H_
#define ORB_TEST_FEATUREEXTRACTOR_CPP_FRAME_H_

#include "homography_decomposition_result.h"
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
  std::map<int, std::weak_ptr<Feature>> features; // map indexed on track id
  double timestamp;
  bool stationary;
  bool is_keyframe = false;
  boost::optional<HomographyDecompositionResult> homography_decomposition_result;

  std::vector<FeatureMatch> GetFeatureMatches(const std::shared_ptr<Frame>& target);
  bool HasDepth();
};

#endif
