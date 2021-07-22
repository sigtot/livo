#include "full_trajectory_manager.h"
#include "file_helpers.h"

#include <utility>
#include <boost/optional.hpp>

FullTrajectoryManager::FullTrajectoryManager(std::string export_filename) : export_filename_(std::move(export_filename))
{
}

void FullTrajectoryManager::AddMetadataForFrame(int frame_id, const FrameMetadata& metadata)
{
  metadata_[frame_id] = metadata;
}

void FullTrajectoryManager::UpdatePoses(const std::map<int, Pose3Stamped>& poses)
{
  if (poses.empty())
  {
    return;  // Nothing to do
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
  std::vector<boost::optional<FrameMetadata>> metadata_vec;
  int last_id_written = last_pose_id_written_;
  auto it = trajectory_.upper_bound(last_pose_id_written_);
  while (it != trajectory_.end() && it->first != oldest_pose_updated_)
  {
    auto metadata = metadata_.find(it->first);
    metadata_vec.push_back(metadata == metadata_.end() ? boost::none :
                                                         boost::optional<FrameMetadata>(metadata->second));
    trajectory_vec.push_back(it->second);
    last_id_written = it->first;
    ++it;
  }
  file_helpers::AppendPosesToFileTUM(trajectory_vec, metadata_vec, export_filename_);
  last_pose_id_written_ = last_id_written;
}
