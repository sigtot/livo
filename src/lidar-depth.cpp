#include "lidar-depth.h"

#include <Eigen/Core>

#include "global_params.h"

boost::optional<cv::Point2f> bearingToPixel(const Eigen::Vector3d& vec, const Eigen::Matrix3d& K)
{
  // Project
  if (vec(2) <= 0)
    return boost::none;
  const Eigen::Vector2d vec2d = Eigen::Vector2d(vec(0) / vec(2), vec(1) / vec(2));

  boost::optional<cv::Point2f> c = cv::Point2f();
  // Shift origin and scale
  c->x = static_cast<float>(K(0, 0) * vec2d(0) + K(0, 2));
  c->y = static_cast<float>(K(1, 1) * vec2d(1) + K(1, 2));
  return c;
}

tf::Transform getLidar2CameraTF()
{
  tf::Quaternion body_p_lidar_quat(GlobalParams::BodyPLidarQuat()[0], GlobalParams::BodyPLidarQuat()[1],
                                   GlobalParams::BodyPLidarQuat()[2], GlobalParams::BodyPLidarQuat()[3]);  // [x y z w]
  tf::Vector3 body_p_lidar_vec(GlobalParams::BodyPLidarVec()[0], GlobalParams::BodyPLidarVec()[1],
                               GlobalParams::BodyPLidarVec()[2]);
  tf::Transform lidar_to_body(body_p_lidar_quat, body_p_lidar_vec);

  tf::Quaternion body_p_cam_quat(GlobalParams::BodyPCamQuat()[0], GlobalParams::BodyPCamQuat()[1],
                                 GlobalParams::BodyPCamQuat()[2], GlobalParams::BodyPCamQuat()[3]);  // [x y z w]
  tf::Vector3 body_p_cam_vec(GlobalParams::BodyPCamVec()[0], GlobalParams::BodyPCamVec()[1],
                             GlobalParams::BodyPCamVec()[2]);
  tf::Transform body_to_cam(body_p_cam_quat.inverse(), -body_p_cam_vec);  // Inverse to get body -> cam tf

  return body_to_cam * lidar_to_body;
}

void projectPCLtoImgFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const tf::Transform& lidar2cameraTF,
                          cv::Mat& dImg)
{
  // If point cloud is empty return an empty Mat
  if (cloud->points.empty())
  {
    dImg = cv::Mat();
    return;
  }
  // Loop through LiDAR points
  for (const auto& pt : cloud->points)
  {
    // Check if point is good
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      continue;
    // Ignore zero points
    if (pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0)
      continue;
    // Transform point from LiDAR frame to Camera frame and remove points behind camera
    tf::Vector3 pt_cam = lidar2cameraTF * tf::Vector3(pt.x, pt.y, pt.z);
    // Check if point is behind camera (ideally zero but 10cm to avoid noisy points)
    if (pt_cam.getZ() < 0.1)
      continue;

    // Project into Image frame
    Eigen::Matrix3d K;
    K.setZero();
    K(0, 0) = GlobalParams::CamFx();
    K(0, 2) = GlobalParams::CamU0();
    K(1, 1) = GlobalParams::CamFy();
    K(1, 2) = GlobalParams::CamV0();
    K(2, 2) = 1.;
    auto pxf = bearingToPixel(Eigen::Vector3d(pt_cam.getX(), pt_cam.getY(), pt_cam.getZ()), K);
    if (!pxf)
      continue;
    cv::Point2i px(cvRound(pxf->x), cvRound(pxf->y));
    // Assign Depth
    if (px.x > 0 && px.y > 0 && px.x < dImg.cols && px.y < dImg.rows)
    {
      // Calculate range w.r.t Camera Frame
      auto range = static_cast<float>(
          std::sqrt(pt_cam.getX() * pt_cam.getX() + pt_cam.getY() * pt_cam.getY() + pt_cam.getZ() * pt_cam.getZ()));
      // If not assigned i.e. 0 - assign depth
      if (dImg.at<float>(px.y, px.x) <= 0.0001 || range < dImg.at<uchar>(px.y, px.x))
      {
        dImg.at<float>(px.y, px.x) = range;
      }
    }
  }
}

boost::optional<float> getDirectDepthFromPatch(const cv::Mat& depthPatch, const std::vector<cv::Point2i>& ROInonZeroLoc,
                                               float tol)
{
  // Loop though non-zero points
  static const cv::Point2i ctrPt(GlobalParams::LidarDepthSearchWindowWidth() / 2,
                                 GlobalParams::LidarDepthSearchWindowHeight() / 2);
  int topCount = 0, bottomCount = 0;
  // Calculate Mean Depth
  float meanDep = 0.0;
  for (auto& roiPt : ROInonZeroLoc)
  {
    meanDep += depthPatch.at<float>(roiPt);
    // Check if point is above or below the center
    if (roiPt.y < ctrPt.y)
      ++topCount;
    else
      ++bottomCount;
  }
  meanDep /= ROInonZeroLoc.size();
  // Single Beam - No information of planarity
  if (topCount < 2 || bottomCount < 2)
    return boost::none;
  // Calculate Std.Dev
  float stdDev = 0.0;
  for (auto& roiPt : ROInonZeroLoc)
    stdDev += (depthPatch.at<float>(roiPt) - meanDep) * (depthPatch.at<float>(roiPt) - meanDep);
  stdDev /= (ROInonZeroLoc.size() - 1);
  stdDev = std::sqrt(stdDev);
  // Check if patch has large Std.Dev
  if (stdDev > tol)
    return boost::none;
  return meanDep;
}

boost::optional<float> getDirectDepthFromLine(const cv::Mat& depthPatch, const std::vector<cv::Point2i>& ROInonZeroLoc,
                                              float tol)
{
  // Loop though non-zero points
  static const cv::Point2i ctrPt(GlobalParams::LidarDepthSearchWindowWidth() / 2,
                                 GlobalParams::LidarDepthSearchWindowWidth() / 2);
  int leftCount = 0, rightCount = 0;
  // Calculate Mean Depth
  float meanDep = 0.0;
  for (auto& roiPt : ROInonZeroLoc)
  {
    meanDep += depthPatch.at<float>(roiPt);
    // Check if point is above or below the center
    if (roiPt.x < ctrPt.x)
      ++leftCount;
    else
      ++rightCount;
  }
  meanDep /= ROInonZeroLoc.size();
  // Single Beam - Balanced Neighborhoods
  if (leftCount < 2 || rightCount < 2)
    return boost::none;
  // Calculate Max depth difference between consective points
  float maxDepDiff = 0.0;
  for (int i = 1; i < ROInonZeroLoc.size(); ++i)
  {
    float depDiff = std::abs(depthPatch.at<float>(ROInonZeroLoc[i]) - depthPatch.at<float>(ROInonZeroLoc[i - 1]));
    if (depDiff > maxDepDiff)
      maxDepDiff = depDiff;
  }
  // Check if line has large depth differences
  if (maxDepDiff > tol)
    return boost::none;
  return meanDep;
}

boost::optional<float> getFeatureDirectDepth(const cv::Point2i& ptf, const cv::Mat& depthImg, float tol)
{
  // Define look-up window and center point
  static const int hWinHalf = GlobalParams::LidarDepthSearchWindowWidth() / 2;
  static const int vWinHalf = GlobalParams::LidarDepthSearchWindowHeight() / 2;
  // Check if look-up ROI is within image
  cv::Point2i pt(cvRound(ptf.x), cvRound(ptf.y));
  int uMin = pt.x - hWinHalf;
  int vMin = pt.y - vWinHalf;
  // Check if ROI boundaries are valid
  if (uMin < 0 || vMin < 0)
  {
    return boost::none;
  }
  if ((pt.x + hWinHalf) > depthImg.cols - 1 || (pt.y + vWinHalf) > depthImg.rows - 1)
  {
    return boost::none;
  }
  // Count Non-Zero depth points in the ROI
  cv::Rect ROI(uMin, vMin, GlobalParams::LidarDepthSearchWindowWidth(), GlobalParams::LidarDepthSearchWindowWidth());
  cv::Mat depthPatch(depthImg, ROI);
  cv::Mat countPatch;
  cv::normalize(depthPatch, countPatch, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  std::vector<cv::Point2i> ROInonZeroLoc;
  cv::findNonZero(countPatch, ROInonZeroLoc);
  // If not enough non-zero depth points neighbor present
  if (ROInonZeroLoc.size() < GlobalParams::LidarDepthMinNonZeroNeighbors())
  {
    return boost::none;
  }
  // TODO: we should also check if the variance among the non-zero depth points within the ROI
  // Get Depth
  boost::optional<float> depth = 0;
  if (GlobalParams::LidarDepthCalcMode() == 0)
    depth = getDirectDepthFromPatch(depthPatch, ROInonZeroLoc, tol);
  else if (GlobalParams::LidarDepthCalcMode() == 1)
    depth = getDirectDepthFromLine(depthPatch, ROInonZeroLoc, tol);
  else
  {
    ROS_WARN_STREAM("=== ROVIO - INVALID DEPTH CALCULATION MODE SET IT SHOUD BE EITHER 0(PATCH) or 1(LINE)===");
    return boost::none;
  }
  if (depth && *depth > GlobalParams::LidarDepthMaxAllowedFeatureDistance())
  {
    return boost::none;
  }
  // DEBUG
  //            << ", MODE: " << (lidarDepthCalcMode_ == 0 ? "PATCH" : "LINE")
  //            << ", NonZero:" << ROInonZeroLoc.size()
  //            << ", depth: " << depth << std::endl;
  // DEBUG
  return depth;
}
