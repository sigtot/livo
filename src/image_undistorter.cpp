#include <iostream>
#include "image_undistorter.h"

void ImageUndistorter::UndistortRadTan(cv::Mat& img, const std::vector<double>& radtan_coeffs)
{
  std::cout << "Undistorting image with rad tan model..." << std::endl;
}

void ImageUndistorter::UndistortEquidistant(cv::Mat& img, const std::vector<double>& equidistant_coeffs)
{
  std::cout << "Warn: Equidistant image undistortion not implemented " << std::endl;
  // TODO implement
}
