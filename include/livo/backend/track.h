#ifndef ORB_TEST_INCLUDE_LIVO_BACKEND_TRACK_H_
#define ORB_TEST_INCLUDE_LIVO_BACKEND_TRACK_H_

#include "backend/feature.h"

#include <vector>

namespace backend
{
struct Track
{
  int id;
  double max_parallax;
  size_t depth_feature_count;
  std::vector<backend::Feature> features;
};
}  // namespace backend

#endif  // ORB_TEST_INCLUDE_LIVO_BACKEND_TRACK_H_
