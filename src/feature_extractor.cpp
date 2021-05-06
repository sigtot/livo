#include "feature_extractor.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>

#include "global_params.h"
#include "Initializer.h"
#include "radtan_undistorter.h"
#include "lidar-depth.h"

#include <algorithm>
#include <utility>
#include <memory>

FeatureExtractor::FeatureExtractor(const ros::Publisher& tracks_pub, const LidarFrameManager& lidar_frame_manager)
  : tracks_pub_(tracks_pub), lidar_frame_manager_(lidar_frame_manager)
{
}

shared_ptr<Frame> FeatureExtractor::lkCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  auto cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);  // TODO perf maybe toCvShare?
  Mat img_resized;
  resize(cvPtr->image, img_resized, Size(), GlobalParams::ResizeFactor(), GlobalParams::ResizeFactor(), INTER_LINEAR);

  cv::Mat img_undistorted;
  UndistortImage(img_resized, img_undistorted);

  shared_ptr<Frame> new_frame = make_shared<Frame>();
  new_frame->image = img_undistorted;
  new_frame->id = frame_count_++;
  new_frame->timestamp = msg->header.stamp.toSec();
  new_frame->stationary = true;  // Assume stationary at first

  if (!frames.empty())
  {
    // Obtain prev image and points
    auto prev_img = frames.back()->image;
    vector<cv::Point2f> prev_points;
    for (const auto& track : active_tracks_)
    {
      prev_points.push_back(track->features.back()->pt);
    }

    // Use optical flow to calculate new point predictions
    vector<cv::Point2f> new_points;
    vector<uchar> status;
    vector<float> err;
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
    cv::calcOpticalFlowPyrLK(prev_img, img_undistorted, prev_points, new_points, status, err, Size(15, 15), 2,
                             criteria);

    // Discard bad points
    for (int i = static_cast<int>(prev_points.size()) - 1; i >= 0;
         --i)  // iterate backwards to not mess up vector when erasing
    {
      // Select good points
      if (status[i] != 1)
      {
        // TODO perf erase-remove?
        old_tracks_.push_back(std::move(active_tracks_[i]));
        active_tracks_.erase(active_tracks_.begin() + i);
        prev_points.erase(prev_points.begin() + i);
        new_points.erase(new_points.begin() + i);
      }
    }

    auto lidar_frame = lidar_frame_manager_.At(new_frame->timestamp);
    // Discard RANSAC outliers and add the rest as new features
    vector<uchar> inlier_mask;
    if (new_points.size() >= 8)
    {
      auto F = findFundamentalMat(prev_points, new_points, CV_FM_RANSAC, 3., 0.99, inlier_mask);
      for (int i = static_cast<int>(prev_points.size()) - 1; i >= 0;
           --i)  // iterate backwards to not mess up vector when erasing
      {
        if (inlier_mask[i])
        {
          auto new_feature = std::make_shared<Feature>(new_frame, new_points[i], active_tracks_[i]);
          if (lidar_frame)
          {
            new_feature->depth = getFeatureDirectDepth(new_feature->pt, (*lidar_frame)->depth_image);
          }
          new_frame->features[active_tracks_[i]->id] = new_feature;
          active_tracks_[i]->features.push_back(std::move(new_feature));
        }
        else
        {
          // TODO perf erase-remove?
          old_tracks_.push_back(std::move(active_tracks_[i]));
          active_tracks_.erase(active_tracks_.begin() + i);
          prev_points.erase(prev_points.begin() + i);
          new_points.erase(new_points.begin() + i);
        }
      }
    }

    double total_dist = 0;
    for (size_t i = 0; i < new_points.size(); ++i)
    {
      auto d_vec = (prev_points[i] - new_points[i]);
      double d = std::sqrt(d_vec.dot(d_vec));
      total_dist += d;
    }
    double average_dist = total_dist / new_points.size();

    if (average_dist > GlobalParams::StationaryThresh())
    {
      new_frame->stationary = false;
      frames.back()->stationary = false;  // When movement is registered between two frames, both are non-stationary
    }

    // Truncate the tracks because we're still stationary and the tracks contain no information
    if (new_frame->stationary)
    {
      /*
      for (auto& track : active_tracks_)
      {
        track->features = std::vector<std::shared_ptr<Feature>>{ track->features.back() };
      }
       */
      old_tracks_.clear();
    }

    // PUBLISH LANDMARK IMAGE. TODO: Move to separate fn
    cv_bridge::CvImage tracks_out_img;
    tracks_out_img.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    tracks_out_img.header.stamp = ros::Time(frames.back()->timestamp);
    tracks_out_img.header.seq = frames.back()->id;
    cvtColor(img_undistorted, tracks_out_img.image, CV_GRAY2RGB);

    for (const auto& track : active_tracks_)
    {
      double intensity = std::min(255., 255 * track->max_parallax / GlobalParams::MinParallaxForKeyframe());
      auto color = new_frame->stationary ? cv::Scalar(255, 0, 0) : cv::Scalar(0, intensity, 0);
      for (int i = static_cast<int>(track->features.size()) - 1; i >= 1 && track->features.size() - i < 15; --i)
      {
        cv::line(tracks_out_img.image, track->features[i - 1]->pt, track->features[i]->pt, color, 1);
      }
      cv::circle(tracks_out_img.image, track->features.back()->pt, 5, color, 1);
    }

    tracks_pub_.publish(tracks_out_img.toImageMsg());
    std::cout << "track count: " << active_tracks_.size() << " active, " << old_tracks_.size() << " old." << std::endl;
    // END PUBLISH LANDMARK IMAGE
  }

  if (active_tracks_.size() < GlobalParams::TrackCountLowerThresh())
  {
    vector<Point2f> corners;
    FindGoodFeaturesToTrackGridded(img_undistorted, corners, 9, 7, GlobalParams::MaxFeaturesPerCell(), 0.3, 7);
    for (const auto& corner : corners)
    {
      auto new_track = std::make_shared<Track>(
          std::vector<std::shared_ptr<Feature>>{ std::make_shared<Feature>(new_frame, corner) });
      new_frame->features[new_track->id] = new_track->features.back();
      new_track->features.back()->track = new_track;
      active_tracks_.push_back(std::move(new_track));
    }
    NonMaxSuppressTracks(GlobalParams::TrackNMSSquaredDistThresh());
  }

  for (int i = static_cast<int>(active_tracks_.size()) - 1; i >= 0; --i)
  {
    if (IsCloseToImageEdge(active_tracks_[i]->features.back()->pt, img_undistorted.cols, img_undistorted.rows,
                           GlobalParams::ImageEdgePaddingPercent()))
    {
      // TODO perf erase-remove?
      if (active_tracks_[i]->features.size() >= 3)
      {
        old_tracks_.push_back(std::move(active_tracks_[i]));
      }
      active_tracks_.erase(active_tracks_.begin() + i);
    }
  }

  frames.push_back(new_frame);

  if (keyframe_tracker_)
  {
    keyframe_tracker_->TryAddFrameSafe(new_frame, active_tracks_);
  }
  else
  {
    keyframe_tracker_ = std::make_shared<KeyframeTracker>(new_frame);
  }

  for (int i = static_cast<int>(active_tracks_.size()) - 1; i >= 0; --i)
  {
    if (active_tracks_[i]->InlierRatio() < GlobalParams::MinKeyframeFeatureInlierRatio())
    {
      active_tracks_.erase(active_tracks_.begin() + i);
    }
  }

  return new_frame;
}

void FeatureExtractor::FindGoodFeaturesToTrackGridded(const Mat& img, vector<cv::Point2f>& corners, int cell_count_x,
                                                      int cell_count_y, int max_features_per_cell, double quality_level,
                                                      double min_distance)
{
  int cell_w = img.cols / cell_count_x;
  int cell_h = img.rows / cell_count_y;
  for (int cell_x = 0; cell_x < cell_count_x; ++cell_x)
  {
    for (int cell_y = 0; cell_y < cell_count_y; ++cell_y)
    {
      cv::Rect mask(cell_x * cell_w, cell_y * cell_h, cell_w, cell_h);
      cv::Mat roi = img(mask);
      vector<cv::Point2f> corners_in_roi;
      // Try to extract 3 times the max number, as we will remove some we do not consider strong enough
      goodFeaturesToTrack(roi, corners_in_roi, 3 * max_features_per_cell, quality_level, min_distance);

      Size winSize = Size(5, 5);
      Size zeroZone = Size(-1, -1);
      TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001);
      if (corners_in_roi.empty())
      {
        continue;  // Nothing further to do
      }
      cv::cornerSubPix(roi, corners_in_roi, winSize, zeroZone, criteria);

      vector<cv::Point2f> best_corners;
      for (int i = 0; i < std::min(static_cast<int>(corners_in_roi.size()), max_features_per_cell); ++i)
      {
        if (PointWasSubPixRefined(corners_in_roi[i]))
        {
          best_corners.push_back(corners_in_roi[i] + cv::Point2f(cell_x * cell_w, cell_y * cell_h));
        }
      }
      corners.insert(corners.begin(), best_corners.begin(), best_corners.end());
    }
  }
}

bool FeatureExtractor::PointWasSubPixRefined(const cv::Point2f& point, double thresh)
{
  return std::abs(point.x - std::round(point.x)) > thresh || std::abs(point.y - std::round(point.y)) > thresh;
}

void FeatureExtractor::NonMaxSuppressTracks(double squared_dist_thresh)
{
  for (int i = 0; i < active_tracks_.size(); ++i)
  {
    for (int j = static_cast<int>(active_tracks_.size()) - 1; j > i; --j)
    {
      auto d_vec = (active_tracks_[i]->features.back()->pt - active_tracks_[j]->features.back()->pt);
      double d2 = d_vec.dot(d_vec);
      if (d2 < squared_dist_thresh)
      {
        active_tracks_.erase(active_tracks_.begin() + j);

        // TODO: Maybe, if track is of certain min length, add it to old_tracks_ instead of deleting
      }
    }
  }
}

bool FeatureExtractor::IsCloseToImageEdge(const Point2f& point, int width, int height, double padding_percentage)
{
  double padding_x = width * padding_percentage;
  double padding_y = height * padding_percentage;
  return !point.inside(cv::Rect2f(padding_x, padding_y, width - 2 * padding_x, height - 2 * padding_y));
}

std::vector<shared_ptr<Track>> FeatureExtractor::GetActiveHighParallaxTracks()
{
  std::vector<shared_ptr<Track>> tracks;
  for (auto& track : active_tracks_)
  {
    if (track->max_parallax >= GlobalParams::MinParallaxForSmoothing())
    {
      tracks.push_back(track);
    }
  }
  return tracks;
}

std::vector<shared_ptr<Track>> FeatureExtractor::GetHighParallaxTracks()
{
  std::vector<shared_ptr<Track>> tracks;
  for (auto& track : old_tracks_)
  {
    if (track->max_parallax >= GlobalParams::MinParallaxForSmoothing())
    {
      tracks.push_back(track);
    }
  }
  for (auto& track : active_tracks_)
  {
    if (track->max_parallax >= GlobalParams::MinParallaxForSmoothing())
    {
      tracks.push_back(track);
    }
  }
  return tracks;
}

std::vector<KeyframeTransform> FeatureExtractor::GetKeyframeTransforms() const
{
  return keyframe_tracker_ ? keyframe_tracker_->GetKeyframeTransforms() : std::vector<KeyframeTransform>{};
}

bool FeatureExtractor::ReadyForInitialization() const
{
  return keyframe_tracker_ && keyframe_tracker_->GoodForInitialization();
}

KeyframeTransform FeatureExtractor::GetNewestKeyframeTransform() const
{
  return keyframe_tracker_->GetNewestKeyframeTransform();
}

boost::optional<std::pair<std::shared_ptr<Frame>, std::shared_ptr<Frame>>>
FeatureExtractor::GetFramesForIMUAttitudeInitialization(int stationary_frame_id)
{
  auto target_it = frames.begin();
  for (; target_it != frames.end() && (*target_it)->id != stationary_frame_id; ++target_it)
    ;

  assert(target_it->get()->id == stationary_frame_id);  // Don't give a non-existent id

  std::pair<std::shared_ptr<Frame>, std::shared_ptr<Frame>> result;

  // Move backward until first non-stationary frame
  auto backward_it = target_it;
  for (; backward_it != frames.begin() && (*(backward_it - 1))->stationary; --backward_it)
    ;
  result.first = *backward_it;

  // Move forward until first non-stationary frame
  auto forward_it = target_it;
  for (; forward_it + 1 != frames.end() && (*(forward_it + 1))->stationary; ++forward_it)
    ;
  result.second = *forward_it;

  if (result.first->stationary && result.second->stationary)
  {
    return boost::make_optional(result);
  }
  else
  {
    return boost::none;
  }
}

std::unique_ptr<ImageUndistorter> makeUndistorter(const cv::Size2i& size)
{
  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << GlobalParams::CamFx(), 0., GlobalParams::CamU0(), 0.,
                           GlobalParams::CamFy(), GlobalParams::CamV0(), 0., 0., 1.);
  if (GlobalParams::DistortionModel() == "radtan")
  {
    return std::unique_ptr<RadTanImageUndistorter>(
        new RadTanImageUndistorter(GlobalParams::DistortionCoefficients(), size, camera_matrix, CV_32FC1));
  }
  else if (GlobalParams::DistortionModel() == "equidistant")
  {
    std::cout << "Equidistant undistortion not yet implemented" << std::endl;
    exit(1);
  }
  else
  {
    std::cout << "Given unsupported distortion model " << GlobalParams::DistortionModel() << ". Typo?" << std::endl;
    exit(1);
  }
}

void FeatureExtractor::UndistortImage(const cv::Mat& input_image, cv::Mat& undistorted_image)
{
  // Construct on first use
  static unique_ptr<ImageUndistorter> image_undistorter = makeUndistorter(cv::Size(input_image.cols, input_image.rows));
  image_undistorter->Undistort(input_image, undistorted_image);
}
