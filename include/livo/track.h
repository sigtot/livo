#ifndef ORB_TEST_SRC_TRACK_H_
#define ORB_TEST_SRC_TRACK_H_

#include "feature.h"

#include <utility>
#include <vector>

struct Track
{
  std::vector<std::shared_ptr<Feature>> features;
  std::vector<std::shared_ptr<Feature>> key_features;
  int id;

  int inlier_count = 0;
  int outlier_count = 0;

  explicit Track(std::vector<std::shared_ptr<Feature>> features) : features(std::move(features))
  {
    static int counter = 0;
    id = counter++;
  }

  double InlierRatio() const
  {
    return static_cast<double>(inlier_count) / static_cast<double>(inlier_count + outlier_count);
  }
};

#endif  // ORB_TEST_SRC_TRACK_H_
