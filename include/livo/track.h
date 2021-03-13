#ifndef ORB_TEST_SRC_TRACK_H_
#define ORB_TEST_SRC_TRACK_H_

#include "feature.h"

#include <utility>
#include <vector>

struct Track
{
  std::vector<Feature> features;
  int id;

  explicit Track(std::vector<Feature> features) : features(std::move(features))
  {
    static int counter = 0;
    id = counter++;
  }
};

#endif  // ORB_TEST_SRC_TRACK_H_
