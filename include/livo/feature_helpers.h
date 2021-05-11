#ifndef ORB_TEST_INCLUDE_LIVO_FEATURE_HELPERS_H_
#define ORB_TEST_INCLUDE_LIVO_FEATURE_HELPERS_H_

#include "feature.h"

#include <vector>
#include <map>
#include <memory>

std::vector<std::pair<int, std::weak_ptr<Feature>>>
SortFeatureMapByDepth(const std::map<int, std::weak_ptr<Feature>>& features);

void SortFeaturesByDepthInPlace(std::vector<std::shared_ptr<Feature>>& features);

#endif  // ORB_TEST_INCLUDE_LIVO_FEATURE_HELPERS_H_
