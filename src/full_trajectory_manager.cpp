#include "full_trajectory_manager.h"

void FullTrajectoryManager::UpdatePoses(const std::map<int, Pose3Stamped>& poses)
{
  for (const auto& pose_pair : poses)
  {
    trajectory_[pose_pair.first] = pose_pair.second;
  }
}

std::map<int, Pose3Stamped> FullTrajectoryManager::GetTrajectoryAsMap() const
{
  return trajectory_;
}

std::vector<Pose3Stamped> FullTrajectoryManager::GetTrajectoryAsVector() const
{
  std::vector<Pose3Stamped> trajectory_vec;
  for (const auto& pose_pair : trajectory_)
  {
    trajectory_vec.push_back(pose_pair.second);
  }
  return trajectory_vec;
}
