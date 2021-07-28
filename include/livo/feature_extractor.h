#ifndef ORB_TEST_FEATUREEXTRACTOR_H
#define ORB_TEST_FEATUREEXTRACTOR_H

#include "frame.h"
#include "track.h"
#include "keyframe_transform.h"
#include "keyframe_tracker.h"
#include "lidar_frame_manager.h"
#include "image_undistorter.h"
#include "new_smoother.h"

#include "backend/frontend_result.h"
#include "backend/feature.h"
#include "backend/track.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <memory>
#include <mutex>
#include <deque>

// TODO remove
using namespace std;
using namespace cv;

class FeatureExtractor
{
private:
  ros::Publisher tracks_pub_;
  ros::Publisher high_delta_tracks_pub_;
  std::deque<shared_ptr<Frame>> frames;
  int frame_count_ = 0;  // Number of frames we have added. The current number of frames in memory may be lower.
  const LidarFrameManager& lidar_frame_manager_;
  NewSmoother& smoother_;

  std::vector<std::shared_ptr<Track>> active_tracks_;
  std::deque<std::shared_ptr<Track>> removed_tracks_;

  shared_ptr<KeyframeTracker> keyframe_tracker_ = nullptr;
  std::shared_ptr<ImageUndistorter> image_undistorter_;

  void ExtractNewCornersInUnderpopulatedGridCells(const Mat& img, vector<cv::Point2f>& corners, int cell_count_x,
                                                  int cell_count_y, int min_features_per_cell,
                                                  int max_initial_features_per_cell, double quality_level,
                                                  double min_distance);

  void RemoveBadDepthTracks();

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

  void ExtractFeatures(const cv::Mat& img, std::shared_ptr<Frame> new_frame,
                                            const boost::optional<std::shared_ptr<LidarFrame>>& lidar_frame);

  void PublishLandmarksImage(const std::shared_ptr<Frame>& frame, const cv::Mat& img,
                             const boost::optional<std::shared_ptr<LidarFrame>>& lidar_frame) const;

  void UndistortImage(const cv::Mat& input_image, cv::Mat& undistorted_image) const;
  void PreProcessImage(cv::Mat& image) const;

  std::vector<backend::Track> GetMatureTracksForBackend() const;
  bool TrackIsMature(const std::shared_ptr<Track>& track) const;

  void RejectOutliersByLandmarkProjections(int frame_id, double timestamp);

  void GetPrevPoints(std::vector<cv::Point2f>& prev_points) const;

  /**
   * @brief Compute and update active tracks with parallaxes and increment their inlier/outlier counts
   * according to the result.
   *
   * The computation may fail if the scene does not have enough parallax.
   * In that case, we neither update inlier/outlier counts nor parallaxes.
   *
   * Additionally, if there are not more than ransac_interval frames processed, no computation is done.
   *
   * @param ransac_interval the interval between the frames to perform RANSAC on (default 5)
   */
  void RANSACComputeParallaxesForTracks(int ransac_interval = 5);

  /**
   * @brief Remove tracks close to the image edges.
   *
   * @param width image width
   * @param height image height
   */
  void RemoveTracksCloseToEdge(int width, int height);

  /**
   * @brief Remove tracks with inlier ratio larger than inlier_ratio
   *
   * The inlier ratio is here defined as inlier_count / (inlier_count + outlier_count)
   *
   * @param inlier_ratio
   * @return the number of outlier tracks removed
   */
  int RemoveTracksByInlierRatio(double inlier_ratio);

public:
  explicit FeatureExtractor(const ros::Publisher& tracks_pub, const ros::Publisher& high_delta_tracks_pub,
                            const LidarFrameManager& lidar_frame_manager,
                            std::shared_ptr<ImageUndistorter> image_undistorter, NewSmoother& smoother);

  backend::FrontendResult lkCallback(const sensor_msgs::Image::ConstPtr& msg);
  void PublishSingleTrackImage(const backend::Track& track);

  boost::optional<std::pair<std::shared_ptr<Frame>, std::shared_ptr<Frame>>>
  GetFramesForIMUAttitudeInitialization(int stationary_frame_id);
};

#endif  // ORB_TEST_FEATUREEXTRACTOR_H
