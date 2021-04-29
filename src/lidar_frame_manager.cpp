#include "lidar_frame_manager.h"
#include "debug_image_publisher.h"
#include "lidar-depth.h"

void LidarFrameManager::LidarCallback(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double timestamp)
{
  cv::Mat dImg;
  projectPCLtoImgFrame(cloud, getLidar2CameraTF(), dImg);
  DebugImagePublisher::PublishDepthImage(dImg, timestamp);
}
