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
                        std::vector<cv::Point2f>& points2, bool init = false);
  static void GetPointsSafe(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                            const std::vector<std::shared_ptr<Track>>& tracks, std::vector<cv::Point2f>& points1,
                            std::vector<cv::Point2f>& points2);
  static void OnlyValidTracks(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                              const std::vector<std::shared_ptr<Track>>& tracks,
                              std::vector<std::shared_ptr<Track>>& valid_tracks);
  static KeyframeTransform MakeKeyframeTransform(const std::vector<cv::Point2f>& points1,
                                                 const std::vector<cv::Point2f>& points2,
                                                 const std::shared_ptr<Frame>& frame1,
                                                 const std::shared_ptr<Frame>& frame2, std::vector<uchar>& inlier_mask);
  static void UpdateTrackInlierOutlierCounts(const std::vector<std::shared_ptr<Track>>& tracks,
                                             const std::vector<uchar>& inlier_mask);

public:
  void AddFrame(const std::shared_ptr<Frame>& frame2, const std::vector<std::shared_ptr<Track>>& tracks);
  KeyframeTracker(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                  const std::shared_ptr<Frame>& frame3, const std::vector<std::shared_ptr<Track>>& tracks);
  std::vector<KeyframeTransform> GetGoodKeyframeTransforms() const;
  bool GoodForInitialization();
  static bool SafeToAddFrame(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                      const std::vector<std::shared_ptr<Track>>& tracks);
  static bool SafeToInitialize(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                               const std::shared_ptr<Frame>& frame3, const std::vector<std::shared_ptr<Track>>& tracks);
};

#endif  // ORB_TEST_SRC_KEYFRAME_TRACKER_H_
