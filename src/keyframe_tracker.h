#ifndef ORB_TEST_SRC_KEYFRAME_TRACKER_H_
#define ORB_TEST_SRC_KEYFRAME_TRACKER_H_

#include <vector>
#include "keyframe_transform.h"
#include "track.h"
#include "frame.h"

class KeyframeTracker
{
private:
  std::vector<KeyframeTransform> keyframe_transforms_;
  static void GetPoints(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                 const std::vector<std::shared_ptr<Track>>& tracks, std::vector<cv::Point2f>& points1,
                 std::vector<cv::Point2f>& points2);
  static KeyframeTransform MakeKeyframeTransform(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2,
                                    const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2);

public:
  void AddFrame(const std::shared_ptr<Frame>& frame2, const std::vector<std::shared_ptr<Track>>& tracks);
  KeyframeTracker(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                  const std::vector<std::shared_ptr<Track>>& tracks);
  const std::vector<KeyframeTransform>& GetKeyframeTransforms() const;
  bool GoodForInitialization();
};

#endif  // ORB_TEST_SRC_KEYFRAME_TRACKER_H_
