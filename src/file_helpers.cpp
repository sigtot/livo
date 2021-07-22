#include "file_helpers.h"

#include <fstream>
#include <iomanip>

namespace file_helpers
{
void AppendPosesToFileTUM(const std::vector<Pose3Stamped>& poses, const std::vector<boost::optional<FrameMetadata>>& frame_metadata,
                          const std::string& filename)
{
  if (poses.empty())
  {
    return;  // Nothing to do
  }
  std::ofstream file;

  file.open(filename, std::ios_base::app);
  auto delimiter = ' ';
  for (int i = 0; i < poses.size(); ++i)
  {
    file << std::setprecision(17) << poses[i].stamp << delimiter;

    file << std::setprecision(8);
    file << poses[i].pose.point.x << delimiter;
    file << poses[i].pose.point.y << delimiter;
    file << poses[i].pose.point.z << delimiter;

    file << poses[i].pose.rot.x << delimiter;
    file << poses[i].pose.rot.y << delimiter;
    file << poses[i].pose.rot.z << delimiter;
    file << poses[i].pose.rot.w << delimiter;

    if (frame_metadata[i])
    {
      file << frame_metadata[i]->n_landmarks << delimiter;
      file << frame_metadata[i]->loam_degenerate << std::endl;
    }
  }
}
}  // namespace file_helpers
