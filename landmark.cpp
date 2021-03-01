#include "landmark.h"

cv::Mat Landmark::GetNewestDescriptor() {
  return keypoint_observations.back()->descriptor;
}
cv::KeyPoint Landmark::GetNewestKeyPoint() {
  return keypoint_observations.back()->keypoint;
}
