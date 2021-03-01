#include "frame.h"
std::vector<cv::KeyPoint> Frame::getKeyPoints() const {
  std::vector<cv::KeyPoint>
      keypoints;  // TODO reserve space up front to avoid resizes
  transform(keypoint_observations.begin(), keypoint_observations.end(),
            back_inserter(keypoints),
            [](const std::shared_ptr<KeyPointObservation>& o) -> cv::KeyPoint {
              return o->keypoint;
            });
  return keypoints;
}
cv::Mat Frame::getDescriptors() const {
  cv::Mat descriptors;
  for (const auto& obs : keypoint_observations) {
    descriptors.push_back(obs->descriptor);
  }
  return descriptors;
}
