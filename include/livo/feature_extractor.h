#ifndef ORB_TEST_FEATUREEXTRACTOR_H
#define ORB_TEST_FEATUREEXTRACTOR_H

#include "landmark.h"
#include "frame.h"
#include "ORBextractor.h"
#include "landmark_match.h"
#include "track.h"
#include "keyframe_transform.h"
#include "keyframe_tracker.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <memory>

using namespace std;
using namespace cv;

struct NotEnoughFramesException : public std::exception
{
  const char* what() const noexcept override
  {
    return "Cannot return first two frames when frame count is less than 2";
  }
};

class FeatureExtractor
{
private:
  ros::Publisher matches_pub_;
  ros::Publisher tracks_pub_;
  vector<shared_ptr<Frame>> frames;
  std::map<int, shared_ptr<Landmark>> landmarks;
  int landmark_count_ = 0;
  int frame_count_ = 0;
  int lag;
  const bool debug = true;

  std::vector<std::shared_ptr<Track>> active_tracks_;
  std::vector<std::shared_ptr<Track>> old_tracks_;

  ORB_SLAM::ORBextractor orb_extractor;

  shared_ptr<KeyframeTracker> keyframe_tracker_ = nullptr;

  static void getMatches(const Mat& query_descriptors, const vector<KeyPoint>& query_keypoints,
                         const cv::Mat& train_descriptors, const vector<KeyPoint>& train_keypoints,
                         vector<DMatch>& matches, vector<uchar>& outlier_mask);

  void CullLandmark(int landmark_id);

  static void FindGoodFeaturesToTrackGridded(const cv::Mat& img, vector<cv::Point2f>& corners, int cell_count_x,
                                             int cell_count_y, int max_features_per_cell, double quality_level,
                                             double min_distance);

  void GetLandmarkMatches(vector<shared_ptr<KeyPointObservation>> new_observations,
                          std::vector<std::shared_ptr<KeyPointObservation>>& remaining_observations,
                          vector<LandmarkMatch>& landmark_matches);

  void NonMaxSuppressTracks(double squared_dist_thresh);

  static bool PointWasSubPixRefined(const Point2f& point, double thresh = 0.0001);

  static bool IsCloseToImageEdge(const Point2f& point, int width, int height, double padding_percentage);

  static void UndistortImage(const cv::Mat& input_image, cv::Mat& undistorted_image);

public:
  explicit FeatureExtractor(const ros::Publisher& matches_pub, const ros::Publisher& tracks_pub, int lag);

  shared_ptr<Frame> lkCallback(const sensor_msgs::Image::ConstPtr& msg);

  void CullLandmarks(int frame_window, double min_obs_percentage);

  void PublishLandmarkTracksImage();

  pair<shared_ptr<Frame>, shared_ptr<Frame>> getFirstTwoFrames();

  int GetLandmarkCount();

  int GetFrameCount();

  vector<shared_ptr<Frame>> GetFrames();

  map<int, shared_ptr<Landmark>> GetLandmarks();

  std::vector<shared_ptr<Track>> GetActiveTracks();
  std::vector<shared_ptr<Track>> GetOldTracks();

  vector<KeyframeTransform> GetValidKeyframeTransforms() const;
  bool ReadyForInitialization() const;
  bool CanPerformStationaryIMUInitialization() const;
  vector<KeyframeTransform> GetKeyframeTransforms() const;
  vector<shared_ptr<Track>> GetHighParallaxTracks();
  vector<shared_ptr<Track>> GetActiveHighParallaxTracks();
  KeyframeTransform GetNewestKeyframeTransform() const;
  boost::optional<std::pair<std::shared_ptr<Frame>, std::shared_ptr<Frame>>>
  GetFramesForIMUAttitudeInitialization(int stationary_frame_id);
};

#endif  // ORB_TEST_FEATUREEXTRACTOR_H
