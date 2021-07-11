#ifndef ORB_TEST_SRC_TRACK_H_
#define ORB_TEST_SRC_TRACK_H_

#include "feature.h"

#include <utility>
#include <vector>

struct Track
{
  std::vector<std::shared_ptr<Feature>> features;
  int id;

  int inlier_count = 0;
  int outlier_count = 0;

  double max_parallax = 0.;

  explicit Track(std::vector<std::shared_ptr<Feature>> features);

  double InlierRatio() const;

  bool HasDepth() const;

  size_t DepthFeatureCount() const;
};

#endif  // ORB_TEST_SRC_TRACK_H_
