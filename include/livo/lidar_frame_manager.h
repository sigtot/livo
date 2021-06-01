#ifndef ORB_TEST_SRC_LIDAR_FRAME_MANAGER_H_
#define ORB_TEST_SRC_LIDAR_FRAME_MANAGER_H_

#include "lidar_frame.h"
#include "time_offset_provider.h"
#include "refined_camera_matrix_provider.h"

#include <Eigen/Core>
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
  // Dependencies
  std::shared_ptr<TimeOffsetProvider> lidar_time_offset_provider_;

  // Members
  std::map<double, std::shared_ptr<LidarFrame>> lidar_frames_;  // <timestamp, frame>

  Eigen::Matrix3d K_;

  // Configuration
  double timestamp_thresh_ = 0.05;

public:
  LidarFrameManager(double timestamp_thresh, std::shared_ptr<TimeOffsetProvider> lidar_time_offset_provider,
                    const std::shared_ptr<RefinedCameraMatrixProvider>& K_provider);
  void LidarCallback(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud, double timestamp);
  boost::optional<std::shared_ptr<LidarFrame>> At(double timestamp) const;
};

#endif  // ORB_TEST_SRC_LIDAR_FRAME_MANAGER_H_
