#include "landmark.h"
#include "frame.h"

cv::Mat Landmark::GetNewestDescriptor()
{
  return keypoint_observations.back()->descriptor;
}
cv::KeyPoint Landmark::GetNewestKeyPoint()
{
  return keypoint_observations.back()->keypoint;
}
int Landmark::GetLastObservationFrameId()
{
  return keypoint_observations.back()->frame->id;
}
