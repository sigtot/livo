#include "lidar_frame_manager.h"
#include "debug_image_publisher.h"
#include "lidar-depth.h"
#include "global_params.h"

void LidarFrameManager::LidarCallback(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double timestamp)
{
  cv::Mat dImg = cv::Mat::zeros(GlobalParams::ImageHeight(), GlobalParams::ImageWidth(), CV_32FC1);
  projectPCLtoImgFrame(cloud, getLidar2CameraTF(), dImg);
  DebugImagePublisher::PublishDepthImage(dImg, timestamp);
}
