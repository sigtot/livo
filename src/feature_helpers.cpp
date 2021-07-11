#include "feature_helpers.h"
#include <algorithm>

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

cv::Point2i GetCellIndex(const cv::Point2f& pt, int cell_w, int cell_h)
{
  int feat_cell_x = static_cast<int>(pt.x) / cell_w;
  int feat_cell_y = static_cast<int>(pt.y) / cell_h;
  return cv::Point2i(feat_cell_x, feat_cell_y);
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
    auto cell_idx = GetCellIndex(feature->pt, cell_w, cell_h);
    feature_counts.at<int>(cell_idx.y, cell_idx.x)++;
  }
}

/**
 * Compute maximum difference in a track's lidar depth measurements.
 * @param track
 * @return max difference in depth or -1 in case the track has no depth.
 */
double ComputeMaxTrackDepthDifference(const std::shared_ptr<Track>& track)
{
  double min_depth = 999.;
  double max_depth = 0.;
  bool have_depth = false;  // We need to check that we don't have a track without any depth
  for (const auto& feature : track->features)
  {
    if (feature->depth)
    {
      max_depth = std::max(max_depth, feature->depth->depth);
      min_depth = std::min(min_depth, feature->depth->depth);
      have_depth = true;
    }
  }
  return have_depth ? max_depth - min_depth : -1.;
}

double ComputeMaxTrackDepthChange(const std::shared_ptr<Track>& track)
{
  double max_change = -1.;
  double prev_depth = -1.;
  for (const auto& feature : track->features)
  {
    if (feature->depth)
    {
      if (prev_depth > 0)
      {
        max_change = std::max(max_change, std::abs(prev_depth - feature->depth->depth));
      }
      prev_depth = feature->depth->depth;
    }
  }
  return max_change;
}

bool IsStationary(const std::vector<cv::Point2f>& prev_points, const std::vector<cv::Point2f>& new_points,
                  double thresh)
{
  double total_dist = 0;
  for (size_t i = 0; i < new_points.size(); ++i)
  {
    auto d_vec = (prev_points[i] - new_points[i]);
    double d = std::sqrt(d_vec.dot(d_vec));
    total_dist += d;
  }
  double average_dist = total_dist / static_cast<double>(new_points.size());
  return average_dist < thresh;
}
