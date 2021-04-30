#ifndef ORB_TEST_INCLUDE_LIVO_LIDAR_FRAME_H_
#define ORB_TEST_INCLUDE_LIVO_LIDAR_FRAME_H_

#include <opencv2/core/core.hpp>

struct LidarFrame {
  cv::Mat depth_image;
  double timestamp;
};

#endif  // ORB_TEST_INCLUDE_LIVO_LIDAR_FRAME_H_
