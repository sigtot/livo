#include "feature_helpers.h"
#include "Initializer.h"
#include "debug_value_publisher.h"
#include "lidar-depth.h"

#include <algorithm>
#include <opencv2/calib3d.hpp>
#include <essential_matrix_decomposition_result.h>
#include <homography_decomposition_result.h>
#include <chrono>
#include <thread>
#include <future>

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
  // cv::Mat point1_comp = K * R12 * K_inv * point2_mat; // Fix spooky inversion! (though it is equivalent)

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
  auto time_before_parallax = std::chrono::system_clock::now();
  if (points1.size() < 8)
  {
    return false;
  }
  auto H_future = std::async(std::launch::async,
                             [&points1, &points2]() { return cv::findHomography(points1, points2, cv::RANSAC, 3); });
  auto F = cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 3., 0.99, inlier_mask);
  auto H = H_future.get();

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

  std::cout << "R_H= " << R_H << "for parallax" << std::endl;
  std::cout << "(S_H= " << S_H << ")" << std::endl;
  std::cout << "(S_F= " << S_F << ")" << std::endl;
  if (R_H > 0.45)
  {
    // Fundamental mat is no good. Let's exit here.
    return false;
  }

  auto E = K.t() * F * K;
  cv::Mat R_E;
  std::vector<double> t_E;
  cv::recoverPose(E, points1, points2, K, R_E, t_E);

  auto time_after_parallax = std::chrono::system_clock::now();
  auto millis_parallax =
      std::chrono::duration_cast<std::chrono::milliseconds>(time_after_parallax - time_before_parallax);
  DebugValuePublisher::PublishParallaxDuration(static_cast<double>(millis_parallax.count()));

  // Essential matrix and recovered R and t represent transformation from cam 2 to cam 1 in cam 2 frame
  // As such, we invert R, so that we obtain the transformation from frame1 to frame2 in frame1
  cv::Mat R12 = R_E.t();

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
  std::cout << "Done computing parallaxes: Took " << static_cast<int>(millis_parallax.count()) << "ms" << std::endl;
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
                            const cv::Mat& R12, const cv::Mat& K, std::vector<double>& parallaxes,
                            std::vector<cv::Point2f>& parallax_points)
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

boost::optional<LidarDepthResult> MaybeGetDepth(const cv::Point2f& pt,
                                                const boost::optional<std::shared_ptr<LidarFrame>>& lidar_frame)
{
  if (lidar_frame)
  {
    return getFeatureDirectDepth(pt, (*lidar_frame)->depth_image);
  }
  return boost::none;
}

boost::optional<LidarDepthResult> CheckDepthResult(const boost::optional<LidarDepthResult>& depth)
{
  if (depth && depth->valid)
  {
    return depth;
  }
  // We force the depth to a boost::none if it is invalid to avoid accidentally using it
  return boost::none;
}

bool PointWasSubPixRefined(const cv::Point2f& point, double thresh)
{
  return std::abs(point.x - std::round(point.x)) > thresh || std::abs(point.y - std::round(point.y)) > thresh;
}

void NonMaxSuppressFeatures(std::vector<std::shared_ptr<Feature>>& features, double squared_dist_thresh, int min_j)
{
  for (int i = 0; i < features.size(); ++i)
  {
    for (int j = static_cast<int>(features.size()) - 1; j > i && j > min_j; --j)
    {
      auto d_vec = (features[i]->pt - features[j]->pt);
      double d2 = d_vec.dot(d_vec);
      if (d2 < squared_dist_thresh)
      {
        features.erase(features.begin() + j);
      }
    }
  }
}

bool IsCloseToImageEdge(const cv::Point2f& point, int width, int height, double padding_percentage)
{
  double padding_x = width * padding_percentage;
  double padding_y = height * padding_percentage;
  return !point.inside(cv::Rect2f(padding_x, padding_y, width - 2 * padding_x, height - 2 * padding_y));
}
