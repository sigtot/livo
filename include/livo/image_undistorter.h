#ifndef ORB_TEST_SRC_IMAGE_UNDISTORTER_H_
#define ORB_TEST_SRC_IMAGE_UNDISTORTER_H_

#include <opencv2/core/core.hpp>

class ImageUndistorter
{
public:
  virtual void Undistort(const cv::Mat& input_image, cv::Mat& undistorted_image) = 0;
};

#endif  // ORB_TEST_SRC_IMAGE_UNDISTORTER_H_
