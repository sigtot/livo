#ifndef ORB_TEST_INCLUDE_LIVO_LANDMARK_RESULT_GTSAM_H_
#define ORB_TEST_INCLUDE_LIVO_LANDMARK_RESULT_GTSAM_H_

#include "landmark_result.h"
#include "gtsam_conversions.h"
#include <gtsam/geometry/Point3.h>

#include <utility>

struct LandmarkResultGtsam
{
  gtsam::Point3 pt;
  LandmarkType type = SmartFactorType;
  LandmarkResultGtsam(gtsam::Point3  pt, LandmarkType type) : pt(std::move(pt)), type(type)
  {
  }
  LandmarkResult ToLandmarkResult() const
  {
    return { ToPoint(pt), type };
  }
};

#endif  // ORB_TEST_INCLUDE_LIVO_LANDMARK_RESULT_GTSAM_H_
