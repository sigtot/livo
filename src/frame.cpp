#include "frame.h"
#include "feature.h"

std::vector<FeatureMatch> Frame::GetFeatureMatches(const std::shared_ptr<Frame>& target)
{
  std::vector<FeatureMatch> matches;
  for (const auto& track_id_feature_pair : target->features)
  {
    auto track_id = track_id_feature_pair.first;
    auto feature = track_id_feature_pair.second;
    if (this->features.count(track_id))
    {
      matches.emplace_back(this->features[track_id].lock(), feature.lock());
    }
  }
  return matches;
}

bool Frame::HasDepth()
{
  for (const auto & feature_pair : features)
  {
    auto feature = feature_pair.second.lock();
    if (feature && feature->depth)
    {
      return true;
    }
  }
  return false;
}
