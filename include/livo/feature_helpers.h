#ifndef ORB_TEST_INCLUDE_LIVO_FEATURE_HELPERS_H_
#define ORB_TEST_INCLUDE_LIVO_FEATURE_HELPERS_H_

#include "feature.h"

#include <vector>
#include <map>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

void SortFeaturesByDepthInPlace(std::vector<std::shared_ptr<Feature>>& features);

double ComputePointParallax(const cv::Point2f& point1, const cv::Point2f& point2, const cv::Mat& R12, const cv::Mat& K,
                            const cv::Mat& K_inv);

#endif  // ORB_TEST_INCLUDE_LIVO_FEATURE_HELPERS_H_
