#ifndef ORB_TEST_INCLUDE_LIVO_FILE_HELPERS_H_
#define ORB_TEST_INCLUDE_LIVO_FILE_HELPERS_H_

#include "pose3_stamped.h"
#include "frame_metadata.h"

#include <vector>
#include <string>
#include <boost/optional.hpp>

namespace file_helpers
{
/**
 * Append poses in TUM format (timestamp tx ty tz qx qy qz qw) to file.
 *
 * @param poses
 * @param filename
 */
void AppendPosesToFileTUM(const std::vector<Pose3Stamped>& poses,
                          const std::vector<boost::optional<FrameMetadata>>& frame_metadata,
                          const std::string& filename);
}  // namespace file_helpers

#endif  // ORB_TEST_INCLUDE_LIVO_FILE_HELPERS_H_
