#ifndef ORB_TEST_INCLUDE_LIVO_BACKEND_FEATURE_H_
#define ORB_TEST_INCLUDE_LIVO_BACKEND_FEATURE_H_

#include "lidar_depth_result.h"

#include <boost/optional.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace backend
{
struct Feature
{
  int track_id;
  int frame_id;
  double timestamp;
  cv::Point2f pt;
  boost::optional<LidarDepthResult> depth;
};
}  // namespace backend

#endif  // ORB_TEST_INCLUDE_LIVO_BACKEND_FEATURE_H_
