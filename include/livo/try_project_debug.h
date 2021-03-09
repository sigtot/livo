#ifndef ORB_TEST_INCLUDE_LIVO_TRY_PROJECT_DEBUG_H_
#define ORB_TEST_INCLUDE_LIVO_TRY_PROJECT_DEBUG_H_

#include "landmark.h"
#include "ros/ros.h"

#include <memory>
#include <vector>

void TryProjectDebug(const std::vector<std::shared_ptr<Landmark>>& landmarks, double depth, double timestamp,
                     ros::Publisher& landmark_publisher);

#endif  // ORB_TEST_INCLUDE_LIVO_TRY_PROJECT_DEBUG_H_
