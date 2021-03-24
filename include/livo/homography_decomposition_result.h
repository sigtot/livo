#include <utility>

#ifndef ORB_TEST_INCLUDE_LIVO_HOMOGRAPHY_DECOMPOSITION_RESULT_H_
#define ORB_TEST_INCLUDE_LIVO_HOMOGRAPHY_DECOMPOSITION_RESULT_H_

struct HomographyDecompositionResult
{
  cv::Mat H;
  std::vector<cv::Mat> Rs;
  std::vector<cv::Mat> ts;
  std::vector<cv::Mat> normals;
  boost::optional<int> selected_index;
  HomographyDecompositionResult(cv::Mat H, std::vector<cv::Mat> Rs, std::vector<cv::Mat> ts,
                                std::vector<cv::Mat> normals)
    : H(std::move(H)), Rs(std::move(Rs)), ts(std::move(ts)), normals(std::move(normals))
  {
  }

  boost::optional<cv::Mat> GetRotation()
  {
    return selected_index ? boost::optional<cv::Mat>(Rs[*selected_index]) : boost::none;
  }

  boost::optional<std::vector<double>> GetTranslation()
  {
    if (selected_index)
    {
      return std::vector<double>{ ts[*selected_index].at<double>(0, 0), ts[*selected_index].at<double>(1, 0),
                                  ts[*selected_index].at<double>(2, 0) };
    }
    return boost::none;
  }

  boost::optional<cv::Mat> GetNormal()
  {
    return selected_index ? boost::optional<cv::Mat>(normals[*selected_index]) : boost::none;
  }
};

#endif  // ORB_TEST_INCLUDE_LIVO_HOMOGRAPHY_DECOMPOSITION_RESULT_H_
