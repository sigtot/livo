#ifndef ORB_TEST_SRC_RADTAN_UNDISTORTER_H_
#define ORB_TEST_SRC_RADTAN_UNDISTORTER_H_

#include "image_undistorter.h"

#include <opencv2/core/core.hpp>

class RadTanImageUndistorter : public ImageUndistorter
{
private:
  cv::Mat rect_map_1_;
  cv::Mat rect_map_2_;

public:
  RadTanImageUndistorter(const std::vector<double>& coeffs, const cv::Size2i& image_size, const cv::Mat& camera_matrix,
                         int datatype = CV_32FC1);

  void Undistort(const cv::Mat& input_image, cv::Mat& undistorted_image) override;
};

#endif  // ORB_TEST_SRC_RADTAN_UNDISTORTER_H_
