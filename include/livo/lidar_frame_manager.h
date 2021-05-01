#ifndef ORB_TEST_SRC_LIDAR_FRAME_MANAGER_H_
#define ORB_TEST_SRC_LIDAR_FRAME_MANAGER_H_

#include "lidar_frame.h"

#include <map>
#include <boost/optional.hpp>
#include <memory>

// Forward declarations to avoid referencing pcl in header
namespace pcl
{
template <class T>
class PointCloud;

class PointXYZI;
}  // namespace pcl

class LidarFrameManager
{
private:
  std::map<double, std::shared_ptr<LidarFrame>> lidar_frames_;  // <timestamp, frame>
public:
  void LidarCallback(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud, double timestamp);
  boost::optional<std::shared_ptr<LidarFrame>> At(double timestamp) const;
};

#endif  // ORB_TEST_SRC_LIDAR_FRAME_MANAGER_H_
