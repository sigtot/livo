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

boost::optional<LidarDepthResult> Track::LastDepth() const
{
  auto feat = std::find_if(features.rbegin(), features.rend(), [](const std::shared_ptr<Feature>& feature) {
    return feature->depth.is_initialized();
  });
  if (feat != features.rend())
  {
    return (*feat)->depth;
  }
  return boost::none;
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

double Track::IntegratedMedianFilteredParallaxes()
{
  double sum = 0;
  for (int i = 1; i < static_cast<int>(parallaxes.size()) - 1; ++i)
  {
    std::vector<double> sorted {parallaxes[i-1], parallaxes[i], parallaxes[i+1]};
    std::sort(sorted.begin(), sorted.end());
    sum += sorted[1];
  }
  return sum;
}

double Track::MedianParallax() const
{
  if (parallaxes.size() < 6)
  {
    return 0;
  }
  std::vector<double> parallaxes_copy(parallaxes);
  std::sort(parallaxes_copy.begin(), parallaxes_copy.end());
  return parallaxes_copy[static_cast<int>(parallaxes_copy.size()) / 2];
}

double Track::ParallaxNaive() const
{
  std::vector<double> naive_parallaxes;
  int ival = 5;
  if (features.size() < ival + 1)
  {
    return 0;
  }
  for (int i = ival; i < features.size(); i += ival)
  {
    std::cout << "calc par for " << i << " of " << features.size() << std::endl;
    auto d_vec = features[i]->pt - features[i - ival]->pt;
    auto d = cv::norm(d_vec);
    naive_parallaxes.push_back(d);
  }
  std::sort(naive_parallaxes.begin(), naive_parallaxes.end());
  return naive_parallaxes[static_cast<int>(naive_parallaxes.size()) / 2];
}
