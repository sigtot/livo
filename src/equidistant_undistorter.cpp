#include "equidistant_undistorter.h"

#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

EquidistantUndistorter::EquidistantUndistorter(const std::vector<double>& coeffs, const cv::Size2i& image_size,
                                               const cv::Mat& camera_matrix, int datatype)
{
  cv::Mat R = cv::Mat::eye(3, 3, datatype);
  cv::fisheye::estimateNewCameraMatrixForUndistortRectify(camera_matrix, coeffs, image_size, R, refined_camera_matrix_);
  std::cout << "Before equidistant estimate - Projection Matrix:\n" << camera_matrix << std::endl;
  std::cout << "After equidistant estimate - Projection Matrix:\n" << refined_camera_matrix_ << std::endl;

  cv::Mat temp_map_1, temp_map_2;
  cv::fisheye::initUndistortRectifyMap(camera_matrix, coeffs, R, refined_camera_matrix_, image_size, datatype,
                                       temp_map_1, temp_map_2);

  cv::convertMaps(temp_map_1, temp_map_2, rect_map_1_, rect_map_2_, CV_16SC2, true);
}

void EquidistantUndistorter::Undistort(const cv::Mat& input_image, cv::Mat& undistorted_image)
{
  cv::remap(input_image, undistorted_image, rect_map_1_, rect_map_2_, cv::INTER_NEAREST);
}

cv::Mat EquidistantUndistorter::GetRefinedCameraMatrix() const
{
  return refined_camera_matrix_;
}
