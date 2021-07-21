#ifndef ORB_TEST_INCLUDE_LIVO_LIDAR_DEPTH_H_
#define ORB_TEST_INCLUDE_LIVO_LIDAR_DEPTH_H_

#include "lidar_depth_result.h"

#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/optional/optional.hpp>
#include <tf/tf.h>

// Taken from Rovio: https://github.com/ethz-asl/rovio/blob/master/src/Camera.cpp
// The boost::optional will be none if the bearing vector points in the negative camera direction (i.e. z <= 0)
boost::optional<cv::Point2f> bearingToPixel(const Eigen::Vector3d& vec, const Eigen::Matrix3d& K);

tf::Transform getLidar2CameraTF();

/** \brief Project LiDAR points from LiDAR frame to Image frame
 *
 *  @param cloud - LiDAR pointcloud
 *  @param lidar2cameraTF - LiDAR-to-Camera transform (Obtained from camera2LiDAR calibration)
 *  @param dImg - Depth Image created by projecting valid LiDAR points into camera frame
 */
void projectPCLtoImgFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const tf::Transform& lidar2cameraTF,
                          const Eigen::Matrix3d& camera_matrix, cv::Mat& dImg);

/** \brief Get LiDAR depth from a patch surrounding a camera feature from the depth image created by
 * projectPCLtoImgFrame
 *
 *  @param ptf - Pixel Coordinates of camera feature
 *  @param depthImg - Depth Image for depth look-up
 *  @return A boost::optional LidarDepthResult. Will be boost::none if no depth is available. Will be a LidarDepthResult
 *  with valid=false if depth is available, but it doesn't pass checks on std.dev etc.
 */
boost::optional<LidarDepthResult> getFeatureDirectDepth(const cv::Point2i& ptf, const cv::Mat& depthImg);

#endif  // ORB_TEST_INCLUDE_LIVO_LIDAR_DEPTH_H_
