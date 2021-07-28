#ifndef ORB_TEST_INCLUDE_LIVO_BACKEND_FRONTEND_RESULT_H_
#define ORB_TEST_INCLUDE_LIVO_BACKEND_FRONTEND_RESULT_H_

#include "backend/track.h"

namespace backend
{
struct FrontendResult
{
  int frame_id;
  double timestamp;
  bool stationary;
  bool is_keyframe;
  bool has_depth;
  /**
   * Active tracks in the system at this frame.
   * All tracks were observed in this frame, and the last feature in every track will be that observation.
   */
  std::vector<backend::Track> mature_tracks;
  int n_ransac_outliers;
};
}  // namespace backend

#endif  // ORB_TEST_INCLUDE_LIVO_BACKEND_FRONTEND_RESULT_H_
