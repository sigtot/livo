#ifndef ORB_TEST_SRC_FULL_TRAJECTORY_MANAGER_H_
#define ORB_TEST_SRC_FULL_TRAJECTORY_MANAGER_H_

#include "pose3_stamped.h"
#include "frame_metadata.h"

#include <map>
#include <vector>
#include <string>

class FullTrajectoryManager
{
private:
public:
  explicit FullTrajectoryManager(std::string export_filename);

private:
  std::map<int, Pose3Stamped> trajectory_;
  std::map<int, FrameMetadata> metadata_;
  std::string export_filename_;

  // Bookkeeping
  int last_pose_id_written_ = -1;
  int oldest_pose_updated_ = -1;

public:

  void AddMetadataForFrame(int frame_id, const FrameMetadata& metadata);

  /**
   * Update existing or add new poses to the full trajectory.
   *
   * @param poses Map of poses to be added/updated.
   * All existing poses in the full trajectory that are specified in this map will be replaced.
   */
  void UpdatePoses(const std::map<int, Pose3Stamped>& poses);

  std::map<int, Pose3Stamped> GetTrajectoryAsMap() const;

  std::vector<Pose3Stamped> GetTrajectoryAsVector() const;

  void WriteToFile();
};

#endif  // ORB_TEST_SRC_FULL_TRAJECTORY_MANAGER_H_
