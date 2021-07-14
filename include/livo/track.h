#ifndef ORB_TEST_SRC_TRACK_H_
#define ORB_TEST_SRC_TRACK_H_

#include "feature.h"

#include <utility>
#include <deque>

struct Track
{
  std::deque<std::shared_ptr<Feature>> features;
  int id;

  int inlier_count = 0;
  int outlier_count = 0;

  double max_parallax = 0.;

  explicit Track(std::shared_ptr<Feature> feature);

  double InlierRatio() const;

  bool HasDepth() const;

  size_t DepthFeatureCount() const;

  void AddFeature(std::shared_ptr<Feature> feature);
};

#endif  // ORB_TEST_SRC_TRACK_H_
