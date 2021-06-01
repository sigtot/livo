#ifndef ORB_TEST_SRC_IMAGE_UNDISTORTER_H_
#define ORB_TEST_SRC_IMAGE_UNDISTORTER_H_

#include "refined_camera_matrix_provider.h"

#include <opencv2/core/core.hpp>

class ImageUndistorter : public RefinedCameraMatrixProvider
{
public:
  virtual void Undistort(const cv::Mat& input_image, cv::Mat& undistorted_image) = 0;
};

#endif  // ORB_TEST_SRC_IMAGE_UNDISTORTER_H_
