#ifndef ORB_TEST_FEATUREEXTRACTOR_H
#define ORB_TEST_FEATUREEXTRACTOR_H

#include "frame.h"
#include "track.h"
#include "keyframe_transform.h"
#include "keyframe_tracker.h"
#include "lidar_frame_manager.h"
#include "image_undistorter.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <memory>
#include <mutex>
#include <deque>

using namespace std;
using namespace cv;

class FeatureExtractor
{
private:
  ros::Publisher tracks_pub_;
  std::deque<shared_ptr<Frame>> frames;
  int frame_count_ = 0;  // Number of frames we have added. The current number of frames in memory may be lower.
  const LidarFrameManager& lidar_frame_manager_;

  std::vector<std::shared_ptr<Track>> active_tracks_;

  shared_ptr<KeyframeTracker> keyframe_tracker_ = nullptr;
  std::shared_ptr<ImageUndistorter> image_undistorter_;

  std::mutex& mu_;

  void FindGoodFeaturesToTrackGridded(const cv::Mat& img, vector<cv::Point2f>& corners, int cell_count_x,
                                      int cell_count_y, int max_features_per_cell, double quality_level,
                                      double min_distance);

  void ExtractNewCornersInUnderpopulatedGridCells(const Mat& img, vector<cv::Point2f>& corners, int cell_count_x,
                                                  int cell_count_y, int min_features_per_cell,
                                                  int max_features_per_cell, double quality_level, double min_distance);

  void NonMaxSuppressTracks(double squared_dist_thresh);

  void RANSACRemoveOutlierTracks(int n_frames);

  /**
   *
   * Discard tracks labeled by the KLT tracker as bad.
   * Discards both tracks in active_tracks_ and the corresponding features in prev_points and new_points.
   *
   * All inputs should be ordered 1-1 according to active_tracks_. The modified prev_points and new_points vectors are
   * are modified in accordance with active_tracks_ so ordering is still valid after applying the method.
   *
   * @param status map of uchars that denote whether a track is bad and should be removed or not
   * @param prev_points
   * @param new_points
   */
  void KLTDiscardBadTracks(const std::vector<uchar>& status, std::vector<cv::Point2f>& prev_points,
                           std::vector<cv::Point2f>& new_points);

  /**
   * Initialize new features and add to tracks for existing tracks obtained with KLT.
   *
   * @param new_points vector of points ordered 1-1 according to active_tracks_
   * @param new_frame the frame in which the new features were observed
   * @param lidar_frame lidar frame to optionally add depth to features if it is avilable
   */
  void KLTInitNewFeatures(const std::vector<cv::Point2f>& new_points, std::shared_ptr<Frame>& new_frame,
                          const boost::optional<std::shared_ptr<LidarFrame>>& lidar_frame);

  /**
   * Initialize new features without adding them to tracks.
   *
   * @param corners
   * @param new_frame the frame in which the new features were observed
   * @param features output features
   * @param lidar_frame lidar frame to optionally add depth to features if it is available
   */
  static void InitNewExtractedFeatures(const std::vector<cv::Point2f>& corners, std::shared_ptr<Frame>& new_frame,
                                       std::vector<std::shared_ptr<Feature>>& features,
                                       const boost::optional<std::shared_ptr<LidarFrame>>& lidar_frame);

  void RemoveRejectedTracks();

  void PublishLandmarksImage(const std::shared_ptr<Frame>& frame, const cv::Mat& img,
                             const boost::optional<std::shared_ptr<LidarFrame>>& lidar_frame) const;

  void KeepOnlyNTracks(size_t n);

  static bool PointWasSubPixRefined(const Point2f& point, double thresh = 0.0001);

  static bool IsCloseToImageEdge(const Point2f& point, int width, int height, double padding_percentage);

  void UndistortImage(const cv::Mat& input_image, cv::Mat& undistorted_image) const;

public:
  explicit FeatureExtractor(const ros::Publisher& tracks_pub, const LidarFrameManager& lidar_frame_manager,
                            std::shared_ptr<ImageUndistorter> image_undistorter, std::mutex& mu);

  shared_ptr<Frame> lkCallback(const sensor_msgs::Image::ConstPtr& msg);

  bool ReadyForInitialization() const;
  vector<KeyframeTransform> GetKeyframeTransforms() const;
  vector<shared_ptr<Track>> GetHighParallaxTracks();
  vector<shared_ptr<Track>> GetActiveHighParallaxTracks();
  KeyframeTransform GetNewestKeyframeTransform() const;
  boost::optional<std::pair<std::shared_ptr<Frame>, std::shared_ptr<Frame>>>
  GetFramesForIMUAttitudeInitialization(int stationary_frame_id);
};

#endif  // ORB_TEST_FEATUREEXTRACTOR_H
