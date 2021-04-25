#ifndef ORB_TEST_INCLUDE_LIVO_FEATURE_H_
#define ORB_TEST_INCLUDE_LIVO_FEATURE_H_

#include <utility>

#include "frame.h"

// Forward declaration for circular reference
struct Track;

struct Feature
{
  Feature(std::shared_ptr<Frame> frame, const cv::Point2f& pt, const std::shared_ptr<Track>& track = nullptr)
    : frame(std::move(frame)), pt(pt), track(track)
  {
  }
  std::weak_ptr<Track> track;
  std::shared_ptr<Frame> frame;
  cv::Point2f pt;
  bool in_smoother = false;
};

#endif  // ORB_TEST_INCLUDE_LIVO_FEATURE_H_
