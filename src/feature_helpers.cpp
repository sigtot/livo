#include "feature_helpers.h"

bool CompareFeatureSharedPtrsByWhetherTheyHaveDepth(const std::shared_ptr<Feature>& a,
                                                    const std::shared_ptr<Feature>& b)
{
  return a->depth && !b->depth;
}

void SortFeaturesByDepthInPlace(std::vector<std::shared_ptr<Feature>>& features)
{
  std::sort(features.begin(), features.end(), CompareFeatureSharedPtrsByWhetherTheyHaveDepth);
}

double ComputePointParallax(const cv::Point2f& point1, const cv::Point2f& point2, const cv::Mat& R12, const cv::Mat& K,
                            const cv::Mat& K_inv)
{
  auto H_rot_only = R12.t();  // Quite spooky that we need to invert this: Need to take a better look at this

  cv::Mat point1_mat = (cv::Mat_<double>(3, 1) << point1.x, point1.y, 1.);
  cv::Mat point2_mat = (cv::Mat_<double>(3, 1) << point2.x, point2.y, 1.);
  cv::Mat point2_comp = K * H_rot_only * K_inv * point1_mat;

  point2_comp /= point2_comp.at<double>(2, 0);  // Normalize homogeneous coordinate
  cv::Mat point2_delta = point2_mat - point2_comp;
  double dist = std::sqrt(point2_delta.dot(point2_delta));  // This works because last coordinate is zero

  return dist;
}

void MakeFeatureCountPerCellTable(int img_w, int img_h, int cell_count_x, int cell_count_y,
                                  const std::map<int, std::weak_ptr<Feature>>& features, cv::Mat_<int>& feature_counts)
{
  int cell_w = img_w / cell_count_x;
  int cell_h = img_h / cell_count_y;

  // After extraction, we want a max of n features per cell, including both new and old features
  // Populate max_feature_counts table with initial max counts
  feature_counts = cv::Mat::zeros(cell_count_y, cell_count_x, CV_8UC1);

  // Increment the cells that already have features
  for (const auto& feature_weak : features)
  {
    auto feature = feature_weak.second.lock();
    if (!feature)
    {
      continue;
    }
    int feat_cell_x = static_cast<int>(feature->pt.x) / cell_w;
    int feat_cell_y = static_cast<int>(feature->pt.y) / cell_h;
    feature_counts.at<int>(feat_cell_y, feat_cell_x)++;
  }
}
