#include "track.h"

Track::Track(std::shared_ptr<Feature> feature) : features(std::deque<std::shared_ptr<Feature>>{ std::move(feature) })
{
  static int counter = 0;
  id = counter++;
}

double Track::InlierRatio() const
{
  return static_cast<double>(inlier_count) / static_cast<double>(inlier_count + outlier_count);
}

bool Track::HasDepth() const
{
  return std::find_if(features.begin(), features.end(), [](const std::shared_ptr<Feature>& feature) {
           return feature->depth.is_initialized();
         }) != features.end();
}

size_t Track::DepthFeatureCount() const
{
  return std::count_if(features.begin(), features.end(),
                       [](const std::shared_ptr<Feature>& feature) { return feature->depth.is_initialized(); });
}

void Track::AddFeature(std::shared_ptr<Feature> feature)
{
  int max_track_length = 100;
  if (features.size() >= max_track_length)
  {
    features.pop_front();
  }
  features.push_back(std::move(feature));
}
