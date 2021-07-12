#include "file_helpers.h"

#include <fstream>
#include <iomanip>

namespace file_helpers
{
void AppendPosesToFileTUM(const std::vector<Pose3Stamped>& poses, const std::string& filename)
{
  if (poses.empty())
  {
    return;  // Nothing to do
  }
  std::ofstream file;

  file.open(filename, std::ios_base::app);
  auto delimiter = ' ';
  for (const auto& pose : poses)
  {
    file << std::setprecision(17) << pose.stamp << delimiter;

    file << std::setprecision(8);
    file << pose.pose.point.x << delimiter;
    file << pose.pose.point.y << delimiter;
    file << pose.pose.point.z << delimiter;

    file << pose.pose.rot.x << delimiter;
    file << pose.pose.rot.y << delimiter;
    file << pose.pose.rot.z << delimiter;
    file << pose.pose.rot.w << std::endl;
  }
}
}  // namespace file_helpers
