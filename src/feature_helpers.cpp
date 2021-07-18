#include "feature_helpers.h"
#include "Initializer.h"

#include <algorithm>
#include <opencv2/calib3d.hpp>
#include <essential_matrix_decomposition_result.h>
#include <homography_decomposition_result.h>

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
                            const cv::Mat& K_inv, cv::Point2f& proj_point)
{
  auto H_rot_only = R12.t();  // Quite spooky that we need to invert this: Need to take a better look at this

  cv::Mat point1_mat = (cv::Mat_<double>(3, 1) << point1.x, point1.y, 1.);
  cv::Mat point2_mat = (cv::Mat_<double>(3, 1) << point2.x, point2.y, 1.);
  cv::Mat point2_comp = K * H_rot_only * K_inv * point1_mat;

  point2_comp /= point2_comp.at<double>(2, 0);  // Normalize homogeneous coordinate
  cv::Mat point2_delta = point2_mat - point2_comp;
  double dist = std::sqrt(point2_delta.dot(point2_delta));  // This works because last coordinate is zero
  proj_point = cv::Point2f(point2_comp.at<double>(0, 0), point2_comp.at<double>(1, 0));

  return dist;
}

bool ComputeParallaxesAndInliers(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2,
                                 const cv::Mat& K, std::vector<double>& parallaxes,
                                 std::vector<cv::Point2f>& parallax_points, std::vector<uchar>& inlier_mask)
{
  auto F = cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 3., 0.99, inlier_mask);
  assert(inlier_mask.size() == points1.size());
  auto H = cv::findHomography(points1, points2, cv::RANSAC, 3);

  std::vector<bool> F_check_inliers;
  double S_F = ORB_SLAM::CheckFundamental(F, F_check_inliers, points1, points2);

  std::vector<bool> H_check_inliers;
  double S_H = ORB_SLAM::CheckHomography(H, H.inv(), H_check_inliers, points1, points2);

  double R_H = S_H / (S_H + S_F);

  // Update inlier_mask with inliers from F check
  for (int i = 0; i < inlier_mask.size(); ++i)
  {
    inlier_mask[i] = inlier_mask[i] && F_check_inliers[i];
  }

  // Essential matrix and recovered R and t represent transformation from cam 2 to cam 1 in cam 2 frame
  // As such, we compute it in reverse order, so that we obtain the transformation from frame1 to frame2 in frame1
  // frame
  auto E = cv::findEssentialMat(points2, points1, K, cv::RANSAC);
  cv::Mat R_E;
  std::vector<double> t_E;
  cv::recoverPose(E, points2, points1, K, R_E, t_E);

  // We compute the homography also if a fundamental matrix is a better fit, to compare normals with later frames
  std::vector<cv::Mat> Rs, ts, normals;
  cv::decomposeHomographyMat(H, K, Rs, ts, normals);
  std::vector<cv::Mat> valid_Rs, valid_ts, valid_normals;
  int least_invalids_idx = 0;
  int least_invalids = 999999;
  for (int i = 0; i < Rs.size(); ++i)
  {
    int invalids = NumPointsBehindCamera(points2, normals[i], K.inv());
    if (invalids == 0)
    {
      valid_Rs.push_back(Rs[i]);
      valid_ts.push_back(ts[i]);
      valid_normals.push_back(normals[i]);
      least_invalids = invalids;
      least_invalids_idx = i;
    }
    else if (invalids < least_invalids)
    {
      least_invalids = invalids;
      least_invalids_idx = i;
    }
  }
  if (valid_Rs.empty())  //
  {
    std::cout << "WARN: May be using R and t decomposed from a degenerate homography!" << std::endl;
    std::cout << "Some (" << least_invalids << "/" << points2.size() << ") points were projected to be behind camera "
              << std::endl;
    valid_Rs.push_back(Rs[least_invalids_idx]);
    valid_ts.push_back(ts[least_invalids_idx]);
    valid_normals.push_back(normals[least_invalids_idx]);
  }
  assert(valid_Rs.size() <= 2);
  HomographyDecompositionResult homography_decomp_result(H, valid_Rs, valid_ts, valid_normals);

  /*
  // Find best homography
  int best_idx = 0;
  int best_reference_idx = 0;
  double largest_dot_product = -1.;
  for (int i = 0; i < homography_decomp_result.normals.size(); ++i)
  {
    for (int j = 0; j < prev_homography_decomp_result.normals.size(); ++j)
    {
      cv::Mat dot_product_mat = homography_decomp_result.normals[i].t() * prev_homography_decomp_result.normals[j];
      double dot_product = dot_product_mat.at<double>(0);
      if (abs(dot_product) > abs(largest_dot_product))
      {
        largest_dot_product = dot_product;
        best_idx = i;
        best_reference_idx = j;
      }
    }
  }
  homography_decomp_result.selected_index = best_idx;
   */

  if (R_H > 0.45)
  {
    // Essential mat is no good. Let's exit here.
    return false;
  }
  cv::Mat R12 = R_E;

  std::cout << "Computing parallax with R = \n" << R12 << std::endl;

  std::vector<double> uncompensated_parallaxes;
  std::vector<cv::Point2f> uncompensated_parallaxes_points;
  ComputePointParallaxes(points1, points2, R12, K, parallaxes, parallax_points);
  ComputePointParallaxes(points1, points2, cv::Mat::eye(3, 3, CV_64F), K, uncompensated_parallaxes,
                         uncompensated_parallaxes_points);

  for (int i = 0; i < parallaxes.size(); ++i)
  {
    // Simple check to avoid bad rotation estimates (e.g. due to very low motion):
    // Rotation compensation should only correct the parallax by a reasonable amount
    auto correction_amount = std::abs(uncompensated_parallaxes[i] - parallaxes[i]);
    auto max_correct = 15;
    if (correction_amount > max_correct)
    {
      std::cout << "WARN: parallax rotation compensation more than allowed: (" << correction_amount << " > "
                << max_correct << std::endl;
    }
  }

  int inlier_count = 0;
  int outlier_count = 0;
  for (const auto& inlier : inlier_mask)
  {
    if (inlier)
    {
      ++inlier_count;
    }
    else
    {
      ++outlier_count;
    }
  }
  std::cout << inlier_count << "/" << outlier_count << " inliers/outliers" << std::endl;
  return true;
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

int NumPointsBehindCamera(const std::vector<cv::Point2f>& points, const cv::Mat& n, const cv::Mat& K_inv)
{
  int num_invalid = 0;
  for (const auto& point : points)
  {
    cv::Mat p = (cv::Mat_<double>(3, 1) << point.x, point.y, 1.);
    cv::Mat m = K_inv * p;
    cv::Mat res = m.t() * n;
    if (res.at<double>(0) <= 0)
    {
      num_invalid++;
    }
  }
  return num_invalid;
}

void ComputePointParallaxes(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2,
                           const cv::Mat& R12, const cv::Mat& K,
                           std::vector<double>& parallaxes, std::vector<cv::Point2f>& parallax_points)
{
  parallaxes.resize(points1.size());
  parallax_points.resize(points1.size());
  cv::Mat K_inv = K.inv();
  for (int i = 0; i < points1.size(); ++i)
  {
    cv::Point2f proj_point;
    auto dist = ComputePointParallax(points1[i], points2[i], R12, K, K_inv, proj_point);
    parallaxes[i] = dist;
    parallax_points[i] = proj_point;
  }
}
