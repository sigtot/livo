/**
 * This code is original part of ORB-SLAM, but has been slightly modified for this use case.
 *
 * Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
 *
 * ORB-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ORB_TEST_INCLUDE_ORB_SLAM_INITIALIZER_H_
#define ORB_TEST_INCLUDE_ORB_SLAM_INITIALIZER_H_

#include <opencv2/opencv.hpp>
#include <vector>

namespace ORB_SLAM
{
double CheckFundamental(const cv::Mat& F21, std::vector<bool>& vbMatchesInliers,
                        const std::vector<cv::Point2f>& referencePoints, const std::vector<cv::Point2f>& currentPoints,
                        double sigma = 1.0);

double CheckHomography(const cv::Mat& H21, const cv::Mat& H12, std::vector<bool>& vbMatchesInliers,
                       const std::vector<cv::Point2f>& referencePoints, const std::vector<cv::Point2f>& currentPoints,
                       double sigma = 1.0);
}  // namespace ORB_SLAM

#endif  // ORB_TEST_INCLUDE_ORB_SLAM_INITIALIZER_H_
