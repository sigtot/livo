#ifndef ORB_TEST_INCLUDE_LIVO_ESSENTIAL_MATRIX_DECOMPOSITION_RESULT_H_
#define ORB_TEST_INCLUDE_LIVO_ESSENTIAL_MATRIX_DECOMPOSITION_RESULT_H_

#include <opencv2/core/core.hpp>
#include <utility>

struct EssentialMatrixDecompositionResult
{
  cv::Mat E;
  cv::Mat R;
  EssentialMatrixDecompositionResult(cv::Mat E, cv::Mat  R, std::vector<double>  t)
    : E(std::move(E)), R(std::move(R)), t(std::move(t))
  {
  }
  std::vector<double> t;  // Scale is not observable, only direction.
};

#endif  // ORB_TEST_INCLUDE_LIVO_ESSENTIAL_MATRIX_DECOMPOSITION_RESULT_H_
