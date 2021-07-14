#ifndef ORB_TEST_INCLUDE_LIVO_FEATURE_H_
#define ORB_TEST_INCLUDE_LIVO_FEATURE_H_

#include <utility>
#include <boost/optional.hpp>

#include "lidar_depth_result.h"
#include "frame.h"

// Forward declaration for circular reference
struct Track;

struct Feature
{
  Feature(const cv::Point2f& pt, int frame_id, double timestamp, const std::shared_ptr<Track>& track = nullptr)
    : pt(pt), frame_id(frame_id), timestamp(timestamp), track(track)
  {
  }
  std::weak_ptr<Track> track;
  int frame_id;
  double timestamp;
  cv::Point2f pt;
  boost::optional<LidarDepthResult> depth;
  bool in_smoother = false; // CURRENTLY NOT USED. BUT MIGHT USE AGAIN FOR DRAWING PURPOSES?
};

#endif  // ORB_TEST_INCLUDE_LIVO_FEATURE_H_
