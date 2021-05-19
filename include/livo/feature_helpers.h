#ifndef ORB_TEST_INCLUDE_LIVO_FEATURE_HELPERS_H_
#define ORB_TEST_INCLUDE_LIVO_FEATURE_HELPERS_H_

#include "feature.h"

#include <vector>
#include <map>
#include <memory>

void SortFeaturesByDepthInPlace(std::vector<std::shared_ptr<Feature>>& features);

#endif  // ORB_TEST_INCLUDE_LIVO_FEATURE_HELPERS_H_
