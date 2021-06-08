#include "feature_extractor.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>

#include "global_params.h"
#include "Initializer.h"
#include "radtan_undistorter.h"
#include "equidistant_undistorter.h"
#include "lidar-depth.h"
#include "feature_helpers.h"

#include <algorithm>
#include <utility>
#include <memory>

FeatureExtractor::FeatureExtractor(const ros::Publisher& tracks_pub, const LidarFrameManager& lidar_frame_manager,
                                   std::shared_ptr<ImageUndistorter> image_undistorter, std::mutex& mu)
  : tracks_pub_(tracks_pub), lidar_frame_manager_(lidar_frame_manager), image_undistorter_(std::move(image_undistorter)),
  mu_(mu)
{
}

shared_ptr<Frame> FeatureExtractor::lkCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  RemoveRejectedTracks();

  auto encoding =
      GlobalParams::ColorImage() ? sensor_msgs::image_encodings::TYPE_8UC3 : sensor_msgs::image_encodings::TYPE_8UC1;

  auto cvPtr = cv_bridge::toCvCopy(msg, encoding);  // TODO perf maybe toCvShare?

  Mat img_bw;
  if (GlobalParams::ColorImage())
  {
    cvtColor(cvPtr->image, img_bw, CV_BGR2GRAY);
  }
  else
  {
    img_bw = cvPtr->image;
  }

  Mat img_resized;
  resize(img_bw, img_resized, Size(), GlobalParams::ResizeFactor(), GlobalParams::ResizeFactor(), INTER_LINEAR);

  cv::Mat img_undistorted;
  UndistortImage(img_resized, img_undistorted);

  shared_ptr<Frame> new_frame = make_shared<Frame>();
  new_frame->image = img_undistorted;
  new_frame->id = frame_count_++;
  new_frame->timestamp = msg->header.stamp.toSec();
  new_frame->stationary = true;  // Assume stationary at first

  auto lidar_frame = lidar_frame_manager_.At(new_frame->timestamp);
  // 5 -> 17ms
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

    {
      std::lock_guard<std::mutex> lk(mu_);
      // Discard bad points
      for (int i = static_cast<int>(prev_points.size()) - 1; i >= 0;
           --i)  // iterate backwards to not mess up vector when erasing
      {
        // Select good points
        if (status[i] != 1)
        {
          active_tracks_.erase(active_tracks_.begin() + i);
          prev_points.erase(prev_points.begin() + i);
          new_points.erase(new_points.begin() + i);
        }
      }
    }
    {
      std::lock_guard<std::mutex> lk(mu_);
      // Initialize tracks for the new features. We will remove outliers later.
      for (int i = 0; i < new_points.size(); ++i)
      {
        auto new_feature = std::make_shared<Feature>(new_frame, new_points[i], active_tracks_[i]);
        if (lidar_frame)
        {
          new_feature->depth = getFeatureDirectDepth(new_feature->pt, (*lidar_frame)->depth_image);
        }
        new_frame->features[active_tracks_[i]->id] = new_feature;
        active_tracks_[i]->features.push_back(std::move(new_feature));
      }
    }

    // Discard RANSAC outliers
    std::cout << "Discarding RANSAC outliers" << std::endl;
    RANSACRemoveOutlierTracks(1);
    RANSACRemoveOutlierTracks(GlobalParams::SecondRANSACNFrames());

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
      std::lock_guard<std::mutex> lk(mu_);
      new_frame->stationary = false;
      frames.back()->stationary = false;  // When movement is registered between two frames, both are non-stationary
    }

    PublishLandmarksImage(new_frame, img_undistorted, lidar_frame);
  }

  // 30 -> 40ms when finding new features
  if (GlobalParams::CountFeaturesPerCell())
  {
    vector<Point2f> corners;
    // 30 -> 40ms
    DetectNewFeaturesInUnderpopulatedGridCells(img_undistorted, corners, GlobalParams::GridCellsX(),
                                               GlobalParams::GridCellsY(), GlobalParams::MinFeaturesPerCell(),
                                               GlobalParams::MaxFeaturesPerCell(), 0.3, 7);
    std::vector<std::shared_ptr<Feature>> features;
    for (const auto& corner : corners)
    {
      auto new_feature = std::make_shared<Feature>(new_frame, corner);
      if (lidar_frame)
      {
        new_feature->depth = getFeatureDirectDepth(new_feature->pt, (*lidar_frame)->depth_image);
      }
      features.push_back(new_feature);
    }
    SortFeaturesByDepthInPlace(features);
    {
      std::lock_guard<std::mutex> lk(mu_);
      for (const auto& feature : features)
      {
        auto new_track = std::make_shared<Track>(std::vector<std::shared_ptr<Feature>>{ feature });
        new_frame->features[new_track->id] = feature;
        feature->track = new_track;
        active_tracks_.push_back(std::move(new_track));
      }
      NonMaxSuppressTracks(GlobalParams::TrackNMSSquaredDistThresh());
    }
  }
  else if (active_tracks_.size() < GlobalParams::TrackCountLowerThresh())
  {
    vector<Point2f> corners;
    // 30 -> 40ms
    FindGoodFeaturesToTrackGridded(img_undistorted, corners, GlobalParams::GridCellsX(), GlobalParams::GridCellsY(),
                                   GlobalParams::MaxFeaturesPerCell(), 0.3, 7);
    std::vector<std::shared_ptr<Feature>> features;
    for (const auto& corner : corners)
    {
      auto new_feature = std::make_shared<Feature>(new_frame, corner);
      if (lidar_frame)
      {
        new_feature->depth = getFeatureDirectDepth(new_feature->pt, (*lidar_frame)->depth_image);
      }
      features.push_back(new_feature);
    }
    SortFeaturesByDepthInPlace(features);
    for (const auto& feature : features)
    {
      auto new_track = std::make_shared<Track>(std::vector<std::shared_ptr<Feature>>{ feature });
      new_frame->features[new_track->id] = feature;
      feature->track = new_track;
      active_tracks_.push_back(std::move(new_track));
    }
    NonMaxSuppressTracks(GlobalParams::TrackNMSSquaredDistThresh());
    KeepOnlyNTracks(GlobalParams::MaxFeatures());
  }
  {
    std::lock_guard<std::mutex> lk(mu_);
    for (int i = static_cast<int>(active_tracks_.size()) - 1; i >= 0; --i)
    {
      if (IsCloseToImageEdge(active_tracks_[i]->features.back()->pt, img_undistorted.cols, img_undistorted.rows,
                             GlobalParams::ImageEdgePaddingPercent()))
      {
        active_tracks_.erase(active_tracks_.begin() + i);
      }
    }
  }

  {
    std::lock_guard<std::mutex> lk(mu_);
    frames.push_back(new_frame);
  }

  if (GlobalParams::UseParallaxKeyframes())
  {
    if (keyframe_tracker_)
    {
      keyframe_tracker_->TryAddFrameSafe(new_frame, active_tracks_);
    }
    else
    {
      keyframe_tracker_ = std::make_shared<KeyframeTracker>(new_frame);
    }
  }
  else if (new_frame->id % GlobalParams::TemporalKeyframeInterval() == 0)
  {
    std::lock_guard<std::mutex> lk(mu_);
    new_frame->is_keyframe = true;
  }

  {
    std::lock_guard<std::mutex> lk(mu_);
    for (int i = static_cast<int>(active_tracks_.size()) - 1; i >= 0; --i)
    {
      if (active_tracks_[i]->InlierRatio() < GlobalParams::MinKeyframeFeatureInlierRatio())
      {
        active_tracks_.erase(active_tracks_.begin() + i);
      }
    }
  }

  return new_frame;
}

void FeatureExtractor::DetectNewFeaturesInUnderpopulatedGridCells(const Mat& img, vector<cv::Point2f>& corners,
                                                                  int cell_count_x, int cell_count_y,
                                                                  int min_features_per_cell, int max_features_per_cell,
                                                                  double quality_level, double min_distance)
{
  int cell_w = img.cols / cell_count_x;
  int cell_h = img.rows / cell_count_y;

  Mat_<int> feature_counts;
  MakeFeatureCountPerCellTable(img.cols, img.rows, cell_count_x, cell_count_y,
                               frames.empty() ? std::map<int, std::weak_ptr<Feature>>{} : frames.back()->features,
                               feature_counts);

  int cells_repopulated = 0;
  for (int cell_x = 0; cell_x < cell_count_x; ++cell_x)
  {
    for (int cell_y = 0; cell_y < cell_count_y; ++cell_y)
    {
      if (feature_counts.at<int>(cell_y, cell_x) >= min_features_per_cell)
      {
        // If we already have enough features in the cell, there's no need to extract more.
        continue;
      }
      auto max_features_to_extract = max_features_per_cell - feature_counts.at<int>(cell_y, cell_x);

      cv::Rect mask(cell_x * cell_w, cell_y * cell_h, cell_w, cell_h);
      cv::Mat roi = img(mask);
      vector<cv::Point2f> corners_in_roi;
      // Try to extract 3 times the max number, as we will remove some we do not consider strong enough
      goodFeaturesToTrack(roi, corners_in_roi, 3 * max_features_to_extract, quality_level, min_distance);

      if (corners_in_roi.empty())
      {
        continue;  // Nothing further to do
      }

      Size winSize = Size(5, 5);
      Size zeroZone = Size(-1, -1);
      TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001);
      cv::cornerSubPix(roi, corners_in_roi, winSize, zeroZone, criteria);

      for (int i = 0; i < std::min(static_cast<int>(corners_in_roi.size()), max_features_to_extract); ++i)
      {
        if (PointWasSubPixRefined(corners_in_roi[i]))
        {
          corners.push_back(corners_in_roi[i] + cv::Point2f(cell_x * cell_w, cell_y * cell_h));
        }
      }
      cells_repopulated++;
    }
  }
  std::cout << "Detected new features in " << cells_repopulated << " cells" << std::endl;
}

void FeatureExtractor::FindGoodFeaturesToTrackGridded(const Mat& img, vector<cv::Point2f>& corners, int cell_count_x,
                                                      int cell_count_y, int max_features_per_cell, double quality_level,
                                                      double min_distance)
{
  int cell_w = img.cols / cell_count_x;
  int cell_h = img.rows / cell_count_y;

  Mat_<int> feature_counts;
  MakeFeatureCountPerCellTable(img.cols, img.rows, cell_count_x, cell_count_y,
                               frames.empty() ? std::map<int, std::weak_ptr<Feature>>{} : frames.back()->features,
                               feature_counts);

  std::vector<std::vector<cv::Point2f>> best_corners_vectors;  // One vector for each cell
  size_t most_corners_in_cell = 0;
  for (int cell_x = 0; cell_x < cell_count_x; ++cell_x)
  {
    for (int cell_y = 0; cell_y < cell_count_y; ++cell_y)
    {
      auto max_features_in_this_cell = max_features_per_cell - feature_counts.at<int>(cell_y, cell_x);
      if (max_features_in_this_cell <= 0)
      {
        continue;
      }

      cv::Rect mask(cell_x * cell_w, cell_y * cell_h, cell_w, cell_h);
      cv::Mat roi = img(mask);
      vector<cv::Point2f> corners_in_roi;
      // Try to extract 3 times the max number, as we will remove some we do not consider strong enough
      goodFeaturesToTrack(roi, corners_in_roi, 3 * max_features_in_this_cell, quality_level, min_distance);

      Size winSize = Size(5, 5);
      Size zeroZone = Size(-1, -1);
      TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001);
      if (corners_in_roi.empty())
      {
        continue;  // Nothing further to do
      }
      cv::cornerSubPix(roi, corners_in_roi, winSize, zeroZone, criteria);

      vector<cv::Point2f> best_corners;
      for (int i = 0; i < std::min(static_cast<int>(corners_in_roi.size()), max_features_in_this_cell); ++i)
      {
        if (PointWasSubPixRefined(corners_in_roi[i]))
        {
          best_corners.push_back(corners_in_roi[i] + cv::Point2f(cell_x * cell_w, cell_y * cell_h));
        }
      }
      most_corners_in_cell = std::max(most_corners_in_cell, best_corners.size());
      best_corners_vectors.push_back(best_corners);
    }
  }
  for (size_t i = 0; i < most_corners_in_cell; ++i)
  {
    for (auto& best_corners_vector : best_corners_vectors)
    {
      if (i >= best_corners_vector.size())
      {
        continue;
      }
      corners.push_back(best_corners_vector[i]);
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
        for (const auto& feature : active_tracks_[j]->features)
        {
          feature->frame->features.erase(active_tracks_[j]->id);
        }
        active_tracks_.erase(active_tracks_.begin() + j);
      }
    }
  }
}

void FeatureExtractor::RemoveRejectedTracks()
{
  std::lock_guard<std::mutex> lk(mu_);
  for (int i = static_cast<int>(active_tracks_.size()) - 1; i > 0; --i)
  {
    if (active_tracks_[i]->rejected)
    {
      std::cout << "Removing rejected track " << i << std::endl;
      active_tracks_.erase(active_tracks_.begin() + i);
    }
  }
}

void FeatureExtractor::KeepOnlyNTracks(size_t n)
{
  assert(n > 0);
  if (n >= active_tracks_.size())
  {
    return;
  }
  for (size_t i = n - 1; i < active_tracks_.size(); ++i)
  {
    for (const auto& feature : active_tracks_[i]->features)
    {
      feature->frame->features.erase(active_tracks_[i]->id);
    }
  }
  active_tracks_.resize(n);
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

void FeatureExtractor::UndistortImage(const cv::Mat& input_image, cv::Mat& undistorted_image) const
{
  image_undistorter_->Undistort(input_image, undistorted_image);
}

void FeatureExtractor::PublishLandmarksImage(const std::shared_ptr<Frame>& frame, const cv::Mat& img,
                                             const boost::optional<std::shared_ptr<LidarFrame>>& lidar_frame) const
{
  cv_bridge::CvImage tracks_out_img;
  tracks_out_img.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
  tracks_out_img.header.stamp = ros::Time(frames.back()->timestamp);
  tracks_out_img.header.seq = frames.back()->id;
  cvtColor(img, tracks_out_img.image, CV_GRAY2RGB);

  if (GlobalParams::DrawLidarLines())
  {
    if (lidar_frame)
    {
      cv::Mat depth_img_mask;
      (*lidar_frame)->depth_image.convertTo(depth_img_mask, CV_8U);

      double min_depth, max_depth;
      cv::minMaxLoc((*lidar_frame)->depth_image, &min_depth, &max_depth, nullptr, nullptr, depth_img_mask);

      auto alpha = 255. / (max_depth - min_depth);
      auto beta = -255. * min_depth / (max_depth - min_depth);
      cv::Mat depth_img_8UC1;
      (*lidar_frame)->depth_image.convertTo(depth_img_8UC1, CV_8UC1, alpha, beta);

      cv::Mat depth_image_jet;
      cv::applyColorMap(depth_img_8UC1, depth_image_jet, cv::COLORMAP_RAINBOW);

      depth_image_jet.copyTo(tracks_out_img.image, depth_img_mask);
    }
    else
    {
      return;
    }
  }

  for (const auto& track : active_tracks_)
  {
    double intensity = std::min(255., 255 * track->max_parallax / GlobalParams::MinParallaxForSmoothing());
    // double intensity = 255;
    auto non_stationary_color = track->HasDepth() ? cv::Scalar(intensity, 0, 0) : cv::Scalar(0, intensity, 0);
    // auto non_stationary_color = cv::Scalar(0, intensity, 0);
    auto color = frame->stationary ? cv::Scalar(255, 255, 0) : non_stationary_color;
    for (int i = static_cast<int>(track->features.size()) - 1; i >= 1 && track->features.size() - i < 15; --i)
    {
      cv::line(tracks_out_img.image, track->features[i - 1]->pt, track->features[i]->pt, color, 1);
    }
    cv::circle(tracks_out_img.image, track->features.back()->pt, 5, color, 1);
    cv::putText(tracks_out_img.image, std::to_string(track->id), track->features.back()->pt + cv::Point2f(7., 7.),
                cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 200));
    if (track->HasDepth())
    {
      auto depth = (*std::find_if(track->features.rbegin(), track->features.rend(),
                                  [](const std::shared_ptr<Feature>& feature) -> bool {
                                    return feature->depth.is_initialized();
                                  }))
                       ->depth;
      std::stringstream stream;
      stream << std::fixed << std::setprecision(2) << depth->depth;
      std::string depth_str = stream.str();
      cv::putText(tracks_out_img.image, depth_str + "m", track->features.back()->pt + cv::Point2f(7., 20.),
                  cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 200));
    }
  }

  tracks_pub_.publish(tracks_out_img.toImageMsg());
  std::cout << "track count: " << active_tracks_.size() << std::endl;
}

void FeatureExtractor::RANSACRemoveOutlierTracks(int n_frames)
{
  std::lock_guard<std::mutex> lk(mu_); // Lock for the whole method so that indexes are not changed
  std::vector<int> track_indices;
  std::vector<cv::Point2f> points_1;
  std::vector<cv::Point2f> points_2;
  for (int i = 0; i < active_tracks_.size(); ++i)
  {
    int track_len = static_cast<int>(active_tracks_[i]->features.size());
    if (track_len <= n_frames)
    {
      // We need features from the n_frames-th previous frame, but the track length is not long enough for it
      continue;
    }

    track_indices.push_back(i);
    points_1.push_back(active_tracks_[i]->features.back()->pt);
    points_2.push_back(active_tracks_[i]->features[track_len - n_frames - 1]->pt);
  }

  if (track_indices.size() < 8)
  {
    return;
  }

  std::vector<uchar> inliers;

  auto F = findFundamentalMat(points_1, points_2, CV_FM_RANSAC, 3., 0.99, inliers);

  // iterate backwards to not mess up vector when erasing
  for (int i = static_cast<int>(track_indices.size()) - 1; i >= 0; --i)
  {
    if (!inliers[i])
    {
      std::cout << "Removing outlier track " << active_tracks_[track_indices[i]]->id << std::endl;
      active_tracks_.erase(active_tracks_.begin() + track_indices[i]);
    }
  }
}
