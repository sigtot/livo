#include <opencv2/imgproc.hpp>
#include "image_undistorter.h"

void RadTanImageUndistorter::Undistort(const cv::Mat& input_image, cv::Mat& undistorted_image)
{
  cv::remap(input_image, undistorted_image, rect_map_1_, rect_map_2_, cv::INTER_NEAREST);
}

RadTanImageUndistorter::RadTanImageUndistorter(const std::vector<double>& coeffs, const cv::Size2i& image_size,
                                               const cv::Mat& camera_matrix, int datatype)
{
  cv::Mat temp_map_1, temp_map_2;
  cv::initUndistortRectifyMap(camera_matrix, coeffs, cv::noArray(), camera_matrix, image_size, datatype, temp_map_1,
                              temp_map_2);
  cv::convertMaps(temp_map_1, temp_map_2, rect_map_1_, rect_map_2_, CV_16SC2, true);
}
