#ifndef ORB_TEST_INCLUDE_LIVO_LANDMARK_RESULT_H_
#define ORB_TEST_INCLUDE_LIVO_LANDMARK_RESULT_H_

#include "point3.h"

enum LandmarkType
{
  SmartFactorType,
  ProjectionFactorType
};

struct LandmarkResult
{
  Point3 pt;
  LandmarkType type = SmartFactorType;
  bool active;
  LandmarkResult(const Point3& pt, LandmarkType type, bool active) : pt(pt), type(type), active(active)
  {
  }
};

#endif  // ORB_TEST_INCLUDE_LIVO_LANDMARK_RESULT_H_
