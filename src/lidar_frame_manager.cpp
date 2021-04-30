#include "lidar_frame_manager.h"
#include "debug_image_publisher.h"
#include "lidar-depth.h"
#include "global_params.h"

void LidarFrameManager::LidarCallback(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double timestamp)
{
  cv::Mat depth_image = cv::Mat::zeros(GlobalParams::ImageHeight(), GlobalParams::ImageWidth(), CV_32FC1);
  projectPCLtoImgFrame(cloud, getLidar2CameraTF(), depth_image);
  lidar_frames_[timestamp] = LidarFrame{ .depth_image = depth_image, .timestamp = timestamp };
  DebugImagePublisher::PublishDepthImage(depth_image, timestamp);
}
