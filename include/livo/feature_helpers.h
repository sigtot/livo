#ifndef ORB_TEST_INCLUDE_LIVO_FEATURE_HELPERS_H_
#define ORB_TEST_INCLUDE_LIVO_FEATURE_HELPERS_H_

#include "feature.h"
#include "track.h"

#include <vector>
#include <map>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

void SortFeaturesByDepthInPlace(std::vector<std::shared_ptr<Feature>>& features);

double ComputePointParallax(const cv::Point2f& point1, const cv::Point2f& point2, const cv::Mat& R12, const cv::Mat& K,
                            const cv::Mat& K_inv);

/**
 * Compute a the cell x and y cell indices for a gridded image with given cell width and height.
 *
 * @param pt
 * @param cell_w
 * @param cell_h
 * @return a cv::Point2i with x and y as the x and y indices of the image cells
 */
cv::Point2i GetCellIndex(const cv::Point2f& pt, int cell_w, int cell_h);

void MakeFeatureCountPerCellTable(int img_w, int img_h, int cell_count_x, int cell_count_y,
                                  const std::map<int, std::weak_ptr<Feature>>& features, cv::Mat_<int>& feature_counts);

double ComputeMaxTrackDepthDifference(const std::shared_ptr<Track>& track);

bool IsStationary(const std::vector<cv::Point2f>& prev_points, const std::vector<cv::Point2f>& new_points,
                  double thresh);

#endif  // ORB_TEST_INCLUDE_LIVO_FEATURE_HELPERS_H_
