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
  static KeyframeTransform TryMakeKeyframeTransform(const std::shared_ptr<Frame>& frame1,
                                                    const std::shared_ptr<Frame>& frame2,
                                                    const std::vector<std::shared_ptr<Track>>& tracks,
                                                    std::vector<uchar>& inlier_mask, bool init = false);
  static void UpdateTrackInlierOutlierCounts(const std::vector<std::shared_ptr<Track>>& tracks,
                                             const std::vector<uchar>& inlier_mask);
  void TryAddFrame(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                   const std::vector<std::shared_ptr<Track>>& tracks, bool init = false);
  // Check if plane with normal n is in from of the camera
  static int NumPointsBehindCamera(const std::vector<cv::Point2f>& points, const cv::Mat& n, const cv::Mat& K_inv);
  static void ChooseBestHomographyDecomposition(KeyframeTransform& transform, KeyframeTransform& reference_transform);
  static int ComputePointParallaxes(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2,
                                    const cv::Mat& R12, const cv::Mat& K, double min_parallax,
                                    std::vector<double>& parallaxes);

public:
  void TryAddFrameSafe(const std::shared_ptr<Frame>& frame2, const std::vector<std::shared_ptr<Track>>& tracks);
  KeyframeTracker(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                  const std::vector<std::shared_ptr<Track>>& tracks);
  std::vector<KeyframeTransform> GetKeyframeTransforms() const;
  std::vector<KeyframeTransform> GetGoodKeyframeTransforms() const;
  bool GoodForInitialization() const;
  static bool SafeToComputeTransforms(const std::shared_ptr<Frame>& frame1, const std::shared_ptr<Frame>& frame2,
                                      const std::vector<std::shared_ptr<Track>>& tracks);
  int GetMostRecentBadTransformIdx() const;
  int GetNumberOfGoodTransforms() const;
};

#endif  // ORB_TEST_SRC_KEYFRAME_TRACKER_H_
