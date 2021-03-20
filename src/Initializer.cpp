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

#include "Initializer.h"

namespace ORB_SLAM
{
double CheckFundamental(const cv::Mat& F21, std::vector<bool>& vbMatchesInliers,
                        const std::vector<cv::Point2f>& referencePoints, const std::vector<cv::Point2f>& currentPoints,
                        double sigma)
{
  assert(referencePoints.size() == currentPoints.size());
  const int N = referencePoints.size();

  const double f11 = F21.at<double>(0, 0);
  const double f12 = F21.at<double>(0, 1);
  const double f13 = F21.at<double>(0, 2);
  const double f21 = F21.at<double>(1, 0);
  const double f22 = F21.at<double>(1, 1);
  const double f23 = F21.at<double>(1, 2);
  const double f31 = F21.at<double>(2, 0);
  const double f32 = F21.at<double>(2, 1);
  const double f33 = F21.at<double>(2, 2);
  std::cout << f11 << ", " << f12 << ", " << f13 << std::endl;
  std::cout << f21 << ", " << f22 << ", " << f23 << std::endl;
  std::cout << f31 << ", " << f32 << ", " << f33 << std::endl;

  vbMatchesInliers.resize(N);

  double score = 0;

  const double th = 3.841;
  const double thScore = 5.991;

  const double invSigmaSquare = 1.0 / (sigma * sigma);

  for (int i = 0; i < N; i++)
  {
    bool bIn = true;

    const cv::Point2f& pt1 = referencePoints[i];
    const cv::Point2f& pt2 = currentPoints[i];

    const double u1 = pt1.x;
    const double v1 = pt1.y;
    const double u2 = pt2.x;
    const double v2 = pt2.y;

    // Reprojection error in second image
    // l2=F21x1=(a2,b2,c2)

    const double a2 = f11 * u1 + f12 * v1 + f13;
    const double b2 = f21 * u1 + f22 * v1 + f23;
    const double c2 = f31 * u1 + f32 * v1 + f33;

    const double num2 = a2 * u2 + b2 * v2 + c2;

    const double squareDist1 = num2 * num2 / (a2 * a2 + b2 * b2);

    const double chiSquare1 = squareDist1 * invSigmaSquare;

    if (chiSquare1 > th)
      bIn = false;
    else
      score += thScore - chiSquare1;

    // Reprojection error in second image
    // l1 =x2tF21=(a1,b1,c1)

    const double a1 = f11 * u2 + f21 * v2 + f31;
    const double b1 = f12 * u2 + f22 * v2 + f32;
    const double c1 = f13 * u2 + f23 * v2 + f33;

    const double num1 = a1 * u1 + b1 * v1 + c1;

    const double squareDist2 = num1 * num1 / (a1 * a1 + b1 * b1);

    const double chiSquare2 = squareDist2 * invSigmaSquare;

    if (chiSquare2 > th)
      bIn = false;
    else
      score += thScore - chiSquare2;

    if (bIn)
      vbMatchesInliers[i] = true;
    else
      vbMatchesInliers[i] = false;
  }

  return score;
}

double CheckHomography(const cv::Mat& H21, const cv::Mat& H12, std::vector<bool>& vbMatchesInliers,
                       const std::vector<cv::Point2f>& referencePoints, const std::vector<cv::Point2f>& currentPoints,
                       double sigma)
{
  assert(referencePoints.size() == currentPoints.size());
  const int N = referencePoints.size();

  const double h11 = H21.at<double>(0, 0);
  const double h12 = H21.at<double>(0, 1);
  const double h13 = H21.at<double>(0, 2);
  const double h21 = H21.at<double>(1, 0);
  const double h22 = H21.at<double>(1, 1);
  const double h23 = H21.at<double>(1, 2);
  const double h31 = H21.at<double>(2, 0);
  const double h32 = H21.at<double>(2, 1);
  const double h33 = H21.at<double>(2, 2);

  const double h11inv = H12.at<double>(0, 0);
  const double h12inv = H12.at<double>(0, 1);
  const double h13inv = H12.at<double>(0, 2);
  const double h21inv = H12.at<double>(1, 0);
  const double h22inv = H12.at<double>(1, 1);
  const double h23inv = H12.at<double>(1, 2);
  const double h31inv = H12.at<double>(2, 0);
  const double h32inv = H12.at<double>(2, 1);
  const double h33inv = H12.at<double>(2, 2);

  vbMatchesInliers.resize(N);

  double score = 0;

  const double th = 5.991;

  const double invSigmaSquare = 1.0 / (sigma * sigma);

  for (int i = 0; i < N; i++)
  {
    bool bIn = true;

    const cv::Point2f& kp1 = referencePoints[i];
    const cv::Point2f& kp2 = currentPoints[i];

    const double u1 = kp1.x;
    const double v1 = kp1.y;
    const double u2 = kp2.x;
    const double v2 = kp2.y;

    // Reprojection error in first image
    // x2in1 = H12*x2

    const double w2in1inv = 1.0 / (h31inv * u2 + h32inv * v2 + h33inv);
    const double u2in1 = (h11inv * u2 + h12inv * v2 + h13inv) * w2in1inv;
    const double v2in1 = (h21inv * u2 + h22inv * v2 + h23inv) * w2in1inv;

    const double squareDist1 = (u1 - u2in1) * (u1 - u2in1) + (v1 - v2in1) * (v1 - v2in1);

    const double chiSquare1 = squareDist1 * invSigmaSquare;

    if (chiSquare1 > th)
      bIn = false;
    else
      score += th - chiSquare1;

    // Reprojection error in second image
    // x1in2 = H21*x1

    const double w1in2inv = 1.0 / (h31 * u1 + h32 * v1 + h33);
    const double u1in2 = (h11 * u1 + h12 * v1 + h13) * w1in2inv;
    const double v1in2 = (h21 * u1 + h22 * v1 + h23) * w1in2inv;

    const double squareDist2 = (u2 - u1in2) * (u2 - u1in2) + (v2 - v1in2) * (v2 - v1in2);

    const double chiSquare2 = squareDist2 * invSigmaSquare;

    if (chiSquare2 > th)
      bIn = false;
    else
      score += th - chiSquare2;

    if (bIn)
      vbMatchesInliers[i] = true;
    else
      vbMatchesInliers[i] = false;
  }

  return score;
}
}  // namespace ORB_SLAM