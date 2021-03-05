#include "frame.h"

std::vector<std::shared_ptr<KeyPointObservation>> Frame::GetUnmatchedObservations()
{
  std::vector<std::shared_ptr<KeyPointObservation>> unmatched_observations;
  for (auto& obs : keypoint_observations)
  {
    if (!obs->landmark.lock())
    {
      unmatched_observations.push_back(obs);
    }
  }
  return unmatched_observations;
}
