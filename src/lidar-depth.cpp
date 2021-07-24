#include "lidar-depth.h"

#include <Eigen/Core>

#include "global_params.h"

boost::optional<cv::Point2f> bearingToPixel(const Eigen::Vector3d& vec, const Eigen::Matrix3d& K)
{
  // Project
  if (vec(2) <= 0)
  {
    return boost::none;
  }
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
  tf::Transform lidar_to_body(body_p_lidar_quat, body_p_lidar_vec);  // body_p_lidar

  tf::Quaternion body_p_cam_quat(GlobalParams::BodyPCamQuat()[0], GlobalParams::BodyPCamQuat()[1],
                                 GlobalParams::BodyPCamQuat()[2], GlobalParams::BodyPCamQuat()[3]);  // [x y z w]
  tf::Vector3 body_p_cam_vec(GlobalParams::BodyPCamVec()[0], GlobalParams::BodyPCamVec()[1],
                             GlobalParams::BodyPCamVec()[2]);
  tf::Transform cam_to_body(body_p_cam_quat, body_p_cam_vec);  // body_p_cam
  auto body_to_cam = cam_to_body.inverse();                    // Inverse to get body -> cam tf // cam_p_body

  return body_to_cam * lidar_to_body;  // cam_p_body * body_p_lidar = cam_p_lidar
}

void projectPCLtoImgFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const tf::Transform& lidar2cameraTF,
                          const Eigen::Matrix3d& camera_matrix, cv::Mat& dImg)
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
    auto pxf = bearingToPixel(Eigen::Vector3d(pt_cam.getX(), pt_cam.getY(), pt_cam.getZ()), camera_matrix);
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

boost::optional<LidarDepthResult> getFeatureDirectDepth(const cv::Point2i& ptf, const cv::Mat& depthImg)
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
    // std::cout << "ROI boundaries invalid: " << uMin << ", " << vMin << std::endl;
    return boost::none;
  }
  if ((pt.x + hWinHalf) > depthImg.cols - 1 || (pt.y + vWinHalf) > depthImg.rows - 1)
  {
    // std::cout << "Out of frame?" << std::endl;
    return boost::none;
  }
  // Count Non-Zero depth points in the ROI
  cv::Rect ROI(uMin, vMin, GlobalParams::LidarDepthSearchWindowWidth(), GlobalParams::LidarDepthSearchWindowWidth());
  cv::Mat depthPatch(depthImg, ROI);
  cv::Mat countPatch;
  cv::normalize(depthPatch, countPatch, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  std::vector<cv::Point2i> ROInonZeroLoc;
  cv::findNonZero(countPatch, ROInonZeroLoc);
  bool valid = true;

  if (ROInonZeroLoc.empty())
  {
    return boost::none;
  }

  // If not enough non-zero depth points neighbor present
  if (ROInonZeroLoc.size() < GlobalParams::LidarDepthMinNonZeroNeighbors())
  {
    // std::cout << "Not enough non-zero neighbors present (have " << ROInonZeroLoc.size() << ", need "
    //           << GlobalParams::LidarDepthMinNonZeroNeighbors() << ")" << std::endl;
    valid = false;
  }
  // Get Depth

  // Loop though non-zero points
  static const cv::Point2i ctrPt(GlobalParams::LidarDepthSearchWindowWidth() / 2,
                                 GlobalParams::LidarDepthSearchWindowHeight() / 2);
  // Calculate Mean Depth
  float meanDep = 0.0;

  // Check that we have a point in every quadrant
  // ccw: top right, top left, bottom left, bottom right
  std::vector<bool> in_quadrant = { false, false, false, false };
  std::vector<float> depth_values;
  depth_values.reserve(ROInonZeroLoc.size());
  for (auto& roiPt : ROInonZeroLoc)
  {
    float depth_value = depthPatch.at<float>(roiPt);
    meanDep += depth_value;
    depth_values.push_back(depth_value);
    // Check if point is above or below the center
    auto top = roiPt.y > ctrPt.y;
    auto bottom = !top;
    auto right = roiPt.x > ctrPt.x;
    auto left = !right;
    if (top && right)
    {
      in_quadrant[0] = true;
    }
    if (top && left)
    {
      in_quadrant[1] = true;
    }
    if (bottom && left)
    {
      in_quadrant[2] = true;
    }
    if (bottom && right)
    {
      in_quadrant[3] = true;
    }
  }
  meanDep /= ROInonZeroLoc.size();
  // The check only makes sense if we need at least 4 neighboring measurements
  if (GlobalParams::LidarDepthMinNonZeroNeighbors() >= 4 &&
      !std::all_of(in_quadrant.begin(), in_quadrant.end(), [](bool x) { return x; }))
  {
    // std::cout << "Not a point in every quadrant" << std::endl;
    valid = false;
  }

  // Calculate Std.Dev
  float stdDev = 0.0;
  for (auto& depth_val : depth_values)
  {
    stdDev += (depth_val - meanDep) * (depth_val - meanDep);
  }
  stdDev /= (ROInonZeroLoc.size() - 1);
  stdDev = std::sqrt(stdDev);

  // Check if patch has large Std.Dev
  if (stdDev > GlobalParams::LidarDepthStdDevTolFactor())
  {
    // std::cout << "std too large: " << stdDev << " (max " << tol_factor * meanDep << ")" << std::endl;
    valid = false;
  }

  // Find median
  std::sort(depth_values.begin(), depth_values.end());
  float median = depth_values[depth_values.size() / 2];

  if (median > GlobalParams::LidarDepthMaxAllowedFeatureDistance())
  {
    // std::cout << "Depth is larger than max allowed feature distance";
    // std::cout << " (" << depth->depth << " > " << GlobalParams::LidarDepthMaxAllowedFeatureDistance() << ")"
    //           << std::endl;
    valid = false;
  }
  return LidarDepthResult{ .valid = valid, .depth = median, .std_dev = stdDev, .neighbors = ROInonZeroLoc.size() };
}
