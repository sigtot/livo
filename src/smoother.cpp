#include "smoother.h"
#include <iostream>

void Smoother::SmoothBatch(const std::vector<std::shared_ptr<Frame>>& frames) {
  std::cout << "Let's process those" << frames.size() << " frames" << std::endl;
}
