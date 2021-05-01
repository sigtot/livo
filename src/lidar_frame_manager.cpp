#include "lidar_frame_manager.h"
#include "debug_image_publisher.h"
#include "lidar-depth.h"
#include "global_params.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void LidarFrameManager::LidarCallback(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud, double timestamp)
{
  cv::Mat depth_image = cv::Mat::zeros(GlobalParams::ImageHeight(), GlobalParams::ImageWidth(), CV_32FC1);
  projectPCLtoImgFrame(cloud, getLidar2CameraTF(), depth_image);
  lidar_frames_[timestamp] = std::make_shared<LidarFrame>(LidarFrame{ depth_image, timestamp });
  DebugImagePublisher::PublishDepthImage(depth_image, timestamp);
}

boost::optional<std::shared_ptr<LidarFrame>> LidarFrameManager::At(double timestamp) const
{
  double THRESH = 0.05;
  // logarithmic in size of container
  auto it = lidar_frames_.upper_bound(timestamp);
  if (it != lidar_frames_.end())
  {
    auto ts_after = it->first;
    auto frame_after = it->second;
    if (it == lidar_frames_.begin())
    {
      return frame_after;
    }
    --it;
    auto ts_before = it->first;
    auto frame_before = it->second;
    auto closest_frame = std::abs(timestamp - ts_after) < std::abs(timestamp - ts_before) ? frame_after : frame_before;
    if (std::abs(timestamp - closest_frame->timestamp) < THRESH)
    {
      return closest_frame;
    }
  }
  return boost::none;
}
