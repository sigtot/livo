#ifndef ORB_TEST_SRC_SMOOTHER_H_
#define ORB_TEST_SRC_SMOOTHER_H_

#include "landmark.h"
#include <memory>

class Smoother {
 public:
  static void SmoothBatch(
      const std::vector<std::shared_ptr<Frame>>& frames,
      const std::vector<std::shared_ptr<Landmark>>& landmarks);
};

#endif
