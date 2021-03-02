#ifndef ORB_TEST_INCLUDE_LIVO_POSE3_H_
#define ORB_TEST_INCLUDE_LIVO_POSE3_H_

#include "point3.h"
#include "rot3.h"

struct Pose3 {
  Point3 point;
  Rot3 rot;
};

#endif  // ORB_TEST_INCLUDE_LIVO_POSE3_H_
