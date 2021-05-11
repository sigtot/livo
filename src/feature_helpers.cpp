#include "feature_helpers.h"

bool CompareFeatureSharedPtrsByWhetherTheyHaveDepth(const std::shared_ptr<Feature>& a,
                                                    const std::shared_ptr<Feature>& b)
{
  return a->depth && !b->depth;
}

bool CompareFeatureWeakPtrsByWhetherTheyHaveDepth(const std::weak_ptr<Feature>& a, const std::weak_ptr<Feature>& b)
{
  auto feat_a = a.lock();
  auto feat_b = b.lock();
  return (feat_a && feat_a->depth) && !(feat_b && feat_b->depth);
}

std::vector<std::pair<int, std::weak_ptr<Feature>>>
SortFeatureMapByDepth(const std::map<int, std::weak_ptr<Feature>>& features)
{
  std::vector<std::pair<int, std::weak_ptr<Feature>>> sorted_features(features.begin(), features.end());

  std::sort(
      sorted_features.begin(), sorted_features.end(),
      [](const std::pair<int, std::weak_ptr<Feature>>& a, const std::pair<int, std::weak_ptr<Feature>>& b) -> bool {
        return CompareFeatureWeakPtrsByWhetherTheyHaveDepth(a.second, b.second);
      });
  return sorted_features;
}

void SortFeaturesByDepthInPlace(std::vector<std::shared_ptr<Feature>>& features)
{
  std::sort(features.begin(), features.end(), CompareFeatureSharedPtrsByWhetherTheyHaveDepth);
}
