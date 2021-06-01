#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include "radtan_undistorter.h"

RadTanImageUndistorter::RadTanImageUndistorter(const std::vector<double>& coeffs, const cv::Size2i& image_size,
                                               const cv::Mat& camera_matrix, int datatype)
{
  refined_camera_matrix_ = cv::getOptimalNewCameraMatrix(camera_matrix, coeffs, image_size, 1);

  std::cout << "Before radtan estimate - Projection Matrix:\n" << camera_matrix << std::endl;
  std::cout << "After equidistant estimate - Projection Matrix:\n" << refined_camera_matrix_ << std::endl;

  cv::Mat temp_map_1, temp_map_2;
  cv::initUndistortRectifyMap(camera_matrix, coeffs, cv::noArray(), refined_camera_matrix_, image_size, datatype,
                              temp_map_1, temp_map_2);
  cv::convertMaps(temp_map_1, temp_map_2, rect_map_1_, rect_map_2_, CV_16SC2, true);
}

void RadTanImageUndistorter::Undistort(const cv::Mat& input_image, cv::Mat& undistorted_image)
{
  cv::remap(input_image, undistorted_image, rect_map_1_, rect_map_2_, cv::INTER_NEAREST);
}

cv::Mat RadTanImageUndistorter::GetRefinedCameraMatrix() const
{
  return refined_camera_matrix_;
}
