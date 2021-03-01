#include "landmark.h"
#include "frame.h"

cv::Mat Landmark::GetNewestDescriptor() {
  return keypoint_observations.back()->descriptor;
}
cv::KeyPoint Landmark::GetNewestKeyPoint() {
  return keypoint_observations.back()->keypoint;
}
int Landmark::GetLastObservationFrameId() {
  auto last_frame = keypoint_observations.back()->frame.lock();
  if (last_frame) {
    return last_frame->id;
  } else {
    return -1; // No frame
  }
}
