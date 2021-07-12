#ifndef ORB_TEST_SRC_FULL_TRAJECTORY_MANAGER_H_
#define ORB_TEST_SRC_FULL_TRAJECTORY_MANAGER_H_

#include "pose3_stamped.h"

#include <map>
#include <vector>

class FullTrajectoryManager
{
private:
  std::map<int, Pose3Stamped> trajectory_;

public:
  /**
   * Update existing or add new poses to the full trajectory.
   *
   * @param poses Map of poses to be added/updated.
   * All existing poses in the full trajectory that are specified in this map will be replaced.
   */
  void UpdatePoses(const std::map<int, Pose3Stamped>& poses);

  std::map<int, Pose3Stamped> GetTrajectoryAsMap() const;

  std::vector<Pose3Stamped> GetTrajectoryAsVector() const;
};

#endif  // ORB_TEST_SRC_FULL_TRAJECTORY_MANAGER_H_
