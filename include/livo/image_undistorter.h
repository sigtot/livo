#ifndef ORB_TEST_SRC_IMAGE_UNDISTORTER_H_
#define ORB_TEST_SRC_IMAGE_UNDISTORTER_H_

#include <opencv2/core/core.hpp>

class ImageUndistorter
{
public:
  static void UndistortRadTan(cv::Mat& img, const std::vector<double>& radtan_coeffs);

  static void UndistortEquidistant(cv::Mat& img, const std::vector<double>& equidistant_coeffs);
};

#endif  // ORB_TEST_SRC_IMAGE_UNDISTORTER_H_
