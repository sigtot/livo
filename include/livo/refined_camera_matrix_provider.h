#ifndef ORB_TEST_INCLUDE_LIVO_REFINED_CAMERA_MATRIX_PROVIDER_H_
#define ORB_TEST_INCLUDE_LIVO_REFINED_CAMERA_MATRIX_PROVIDER_H_

#include <opencv2/core/core.hpp>

class RefinedCameraMatrixProvider
{
public:
  virtual cv::Mat GetRefinedCameraMatrix() const = 0;
};

#endif  // ORB_TEST_INCLUDE_LIVO_REFINED_CAMERA_MATRIX_PROVIDER_H_
