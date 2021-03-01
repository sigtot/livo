#ifndef ORB_TEST_SRC_SMOOTHER_H_
#define ORB_TEST_SRC_SMOOTHER_H_

#include "frame.h"
#include <memory>

class Smoother {
 public:
  static void SmoothBatch(const std::vector<std::shared_ptr<Frame>>& frames);
};

#endif
