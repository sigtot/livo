#include "equidistant_undistorter.h"

#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

EquidistantUndistorter::EquidistantUndistorter(const std::vector<double>& coeffs, const cv::Size2i& image_size,
                                               const cv::Mat& camera_matrix, int datatype)
{
  // Optimal New Camera Matrices (Projection Matrix)
  cv::Mat new_camera_matrix = camera_matrix;
  cv::Mat R = cv::Mat::eye(3, 3, datatype);
  std::cout << "size: " << image_size << std::endl;
  std::cout << "Before equidistant estimate - Projection Matrix:\n" << new_camera_matrix << std::endl;
  cv::fisheye::estimateNewCameraMatrixForUndistortRectify(camera_matrix, coeffs, image_size, R, new_camera_matrix);
  std::cout << "After equidistant estimate - Projection Matrix:\n" << new_camera_matrix << std::endl;

  // Create Rectification and Undistortion Maps
  cv::Mat temp_map_1, temp_map_2;
  cv::fisheye::initUndistortRectifyMap(camera_matrix, coeffs, R, new_camera_matrix, image_size, datatype, temp_map_1,
                                       temp_map_2);

  cv::convertMaps(temp_map_1, temp_map_2, rect_map_1_, rect_map_2_, CV_16SC2, true);
}

void EquidistantUndistorter::Undistort(const cv::Mat& input_image, cv::Mat& undistorted_image)
{
  cv::remap(input_image, undistorted_image, rect_map_1_, rect_map_2_, cv::INTER_NEAREST);
}
