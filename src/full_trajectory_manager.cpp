#include "full_trajectory_manager.h"

#include <utility>
#include "file_helpers.h"

FullTrajectoryManager::FullTrajectoryManager(std::string export_filename) : export_filename_(std::move(export_filename))
{
}

void FullTrajectoryManager::UpdatePoses(const std::map<int, Pose3Stamped>& poses)
{
  if (poses.empty())
  {
    return; // Nothing to do
  }
  for (const auto& pose_pair : poses)
  {
    trajectory_[pose_pair.first] = pose_pair.second;
  }
  oldest_pose_updated_ = poses.begin()->first;
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

void FullTrajectoryManager::WriteToFile()
{
  std::vector<Pose3Stamped> trajectory_vec;
  int last_id_written = last_pose_id_written_;
  auto it = trajectory_.upper_bound(last_pose_id_written_);
  while (it != trajectory_.end() && it->first != oldest_pose_updated_)
  {
    trajectory_vec.push_back(it->second);
    last_id_written = it->first;
    ++it;
  }
  file_helpers::AppendPosesToFileTUM(trajectory_vec, export_filename_);
  last_pose_id_written_ = last_id_written;
}
