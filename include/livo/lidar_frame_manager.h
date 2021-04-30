#ifndef ORB_TEST_SRC_LIDAR_FRAME_MANAGER_H_
#define ORB_TEST_SRC_LIDAR_FRAME_MANAGER_H_

#include "lidar_frame.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <map>

class LidarFrameManager
{
private:
  std::map<double, LidarFrame> lidar_frames_;  // <timestamp, frame>
public:
  void LidarCallback(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double timestamp);
};

#endif  // ORB_TEST_SRC_LIDAR_FRAME_MANAGER_H_
