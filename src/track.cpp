#include "track.h"

Track::Track(std::vector<std::shared_ptr<Feature>> features) : features(std::move(features))
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
  return std::find_if(features.begin(), features.end(), [](const std::shared_ptr<Feature>& feature) -> bool {
           return feature->depth.is_initialized();
         }) != features.end();
}
