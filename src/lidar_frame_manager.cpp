#include "lidar_frame_manager.h"
#include "debug_image_publisher.h"
#include "lidar-depth.h"
#include "global_params.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/eigen.hpp>

LidarFrameManager::LidarFrameManager(double timestamp_thresh,
                                     std::shared_ptr<TimeOffsetProvider> lidar_time_offset_provider,
                                     const std::shared_ptr<RefinedCameraMatrixProvider>& refined_camera_matrix_provider)
  : timestamp_thresh_(timestamp_thresh), lidar_time_offset_provider_(std::move(lidar_time_offset_provider))
{
  cv::cv2eigen(refined_camera_matrix_provider->GetRefinedCameraMatrix(), K_);
}

void LidarFrameManager::LidarCallback(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud, double timestamp)
{
  auto compensated_timestamp = timestamp - lidar_time_offset_provider_->GetOffset(timestamp);
  cv::Mat depth_image = cv::Mat::zeros(GlobalParams::ImageHeight(), GlobalParams::ImageWidth(), CV_32FC1);
  projectPCLtoImgFrame(cloud, getLidar2CameraTF(), K_, depth_image);
  lidar_frames_[compensated_timestamp] = std::make_shared<LidarFrame>(LidarFrame{ depth_image, compensated_timestamp });
  //DebugImagePublisher::PublishDepthImage(depth_image, compensated_timestamp);

  if (lidar_frames_.size() > GlobalParams::LidarMaxMessagesRetained())
  {
    lidar_frames_.erase(lidar_frames_.begin());
  }
}

boost::optional<std::shared_ptr<LidarFrame>> LidarFrameManager::At(double timestamp) const
{
  // logarithmic in size of container
  auto it = lidar_frames_.upper_bound(timestamp);
  if (it != lidar_frames_.end())
  {
    auto ts_after = it->first;
    auto frame_after = it->second;
    if (it == lidar_frames_.begin())
    {
      if (std::abs(timestamp - frame_after->timestamp) < timestamp_thresh_)
      {
        return frame_after;
      }
      else {
        return boost::none;
      }
    }
    --it;
    auto ts_before = it->first;
    auto frame_before = it->second;
    auto closest_frame = std::abs(timestamp - ts_after) < std::abs(timestamp - ts_before) ? frame_after : frame_before;
    if (std::abs(timestamp - closest_frame->timestamp) < timestamp_thresh_)
    {
      return closest_frame;
    }
  }
  return boost::none;
}
