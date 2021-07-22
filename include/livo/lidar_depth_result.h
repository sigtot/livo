#ifndef ORB_TEST_INCLUDE_LIVO_LIDAR_DEPTH_RESULT_H_
#define ORB_TEST_INCLUDE_LIVO_LIDAR_DEPTH_RESULT_H_

#include <stddef.h>

struct LidarDepthResult {
  /// Depth result passes all checks (std dev check etc.) and can be used as a measurement
  bool valid;
  /// In camera frame
  double depth;
  double std_dev;
  size_t neighbors;
};

#endif  // ORB_TEST_INCLUDE_LIVO_LIDAR_DEPTH_RESULT_H_
