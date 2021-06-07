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

using namespace std;
using namespace cv;

class FeatureExtractor
{
private:
  ros::Publisher tracks_pub_;
  vector<shared_ptr<Frame>> frames;
  int frame_count_ = 0;
  const LidarFrameManager& lidar_frame_manager_;

  std::vector<std::shared_ptr<Track>> active_tracks_;
  std::vector<std::shared_ptr<Track>> old_tracks_;

  shared_ptr<KeyframeTracker> keyframe_tracker_ = nullptr;
  std::shared_ptr<ImageUndistorter> image_undistorter_;

  void FindGoodFeaturesToTrackGridded(const cv::Mat& img, vector<cv::Point2f>& corners, int cell_count_x,
                                      int cell_count_y, int max_features_per_cell, double quality_level,
                                      double min_distance);

  void DetectNewFeaturesInUnderpopulatedGridCells(const Mat& img, vector<cv::Point2f>& corners, int cell_count_x,
                                                  int cell_count_y, int min_features_per_cell,
                                                  int max_features_per_cell, double quality_level, double min_distance);

  void NonMaxSuppressTracks(double squared_dist_thresh);

  void RANSACRemoveOutlierTracks(int n_frames);

  void RemoveRejectedTracks();

  void PublishLandmarksImage(const std::shared_ptr<Frame>& frame, const cv::Mat& img,
                             const boost::optional<std::shared_ptr<LidarFrame>>& lidar_frame) const;

  void KeepOnlyNTracks(size_t n);

  static bool PointWasSubPixRefined(const Point2f& point, double thresh = 0.0001);

  static bool IsCloseToImageEdge(const Point2f& point, int width, int height, double padding_percentage);

  void UndistortImage(const cv::Mat& input_image, cv::Mat& undistorted_image) const;

public:
  explicit FeatureExtractor(const ros::Publisher& tracks_pub, const LidarFrameManager& lidar_frame_manager,
                            std::shared_ptr<ImageUndistorter> image_undistorter);

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
