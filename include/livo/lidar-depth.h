#ifndef ORB_TEST_INCLUDE_LIVO_LIDAR_DEPTH_H_
#define ORB_TEST_INCLUDE_LIVO_LIDAR_DEPTH_H_

#include <opencv2/core/core.hpp>
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
                          cv::Mat& dImg);

/** \brief Get LiDAR depth using a Patch surrounding a camera feature
 *
 *  @param depthPatch - Patch containing depth values surrounding the feature location
 *  @param ROInonZeroLoc - Vector of non-zero depth locations in the depthPatch
 *  @param tol - Tolereance of Std.Dev of depth patch
 *  @return - Mean Patch Depth, -1 if invalid patch
 */
float getDirectDepthFromPatch(const cv::Mat& depthPatch, const std::vector<cv::Point2i>& ROInonZeroLoc, float tol = 0.1);

/** \brief Get LiDAR depth using a LINE nearest to the camera feature
 *
 *  @param depthPatch - Patch containing depth values surrounding the feature location
 *  @param ROInonZeroLoc - Vector of non-zero depth locations in the depthPatch
 *  @param tol - Tolereance of Max depth difference between consective points on the line
 *  @return - Mean Line Depth, -1 if invalid
 */
float getDirectDepthFromLine(const cv::Mat& depthPatch, const std::vector<cv::Point2i>& ROInonZeroLoc, float tol = 0.1);

/** \brief Get LiDAR depth surrounding a camera feature from the depth image created by projectPCLtoImgFrame
 *
 *  @param ptf - Pixel Coordinates of camera feature
 *  @param depthImg - Depth Image for depth look-up
 *  @param feaID - ID of feature
 *  @param tol - Tolereance of Std.Dev of depth patch
 *  @return - Mean Patch Depth, -1 if invalid patch
 */
float getFeatureDirectDepth(const cv::Point2i& ptf, const cv::Mat& depthImg, const int& feaID, float tol = 0.1);

#endif  // ORB_TEST_INCLUDE_LIVO_LIDAR_DEPTH_H_
