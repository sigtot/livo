#ifndef ORB_TEST_INCLUDE_LIVO_LIDAR_DEPTH_RESULT_H_
#define ORB_TEST_INCLUDE_LIVO_LIDAR_DEPTH_RESULT_H_

#include <stddef.h>

struct LidarDepthResult {
  double depth;
  double std_dev;
  size_t neighbors;
};

#endif  // ORB_TEST_INCLUDE_LIVO_LIDAR_DEPTH_RESULT_H_
