#ifndef ORB_TEST_INCLUDE_LIVO_POSE3_STAMPED_H_
#define ORB_TEST_INCLUDE_LIVO_POSE3_STAMPED_H_

#include "pose3.h"

struct Pose3Stamped {
  Pose3 pose;
  double stamp;
};

#endif  // ORB_TEST_INCLUDE_LIVO_POSE3_STAMPED_H_
