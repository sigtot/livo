#include "feature_extractor.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>

#include "global_params.h"
#include "Initializer.h"
#include "radtan_undistorter.h"
#include "equidistant_undistorter.h"
#include "lidar-depth.h"
#include "feature_helpers.h"
#include "debug_value_publisher.h"

#include <algorithm>
#include <utility>
#include <memory>

FeatureExtractor::FeatureExtractor(const ros::Publisher& tracks_pub, const LidarFrameManager& lidar_frame_manager,
                                   std::shared_ptr<ImageUndistorter> image_undistorter, std::mutex& mu,
                                   const NewSmoother& smoother)
  : tracks_pub_(tracks_pub)
  , lidar_frame_manager_(lidar_frame_manager)
  , image_undistorter_(std::move(image_undistorter))
  , mu_(mu)
  , smoother_(smoother)
{
}

shared_ptr<Frame> FeatureExtractor::lkCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  auto encoding =
      GlobalParams::ColorImage() ? sensor_msgs::image_encodings::TYPE_8UC3 : sensor_msgs::image_encodings::TYPE_8UC1;

  auto cvPtr = cv_bridge::toCvShare(msg, encoding);

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
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), GlobalParams::KLTMaxIterations(),
                                         GlobalParams::KLTConvergenceEpsilon());

    auto time_before_klt = std::chrono::system_clock::now();
    cv::calcOpticalFlowPyrLK(prev_img, img_undistorted, prev_points, new_points, status, err,
                             Size(GlobalParams::KLTWinSize(), GlobalParams::KLTWinSize()), GlobalParams::KLTPyramids(),
                             criteria);

    auto time_after_klt = std::chrono::system_clock::now();
    auto micros_klt = std::chrono::duration_cast<std::chrono::microseconds>(time_after_klt - time_before_klt);
    double millis_klt = static_cast<double>(micros_klt.count()) / 1000.;
    DebugValuePublisher::PublishKLTDuration(millis_klt);

    KLTDiscardBadTracks(status, prev_points, new_points);

    // Initialize features. We will remove outliers after.
    KLTInitNewFeatures(new_points, new_frame, lidar_frame);

    // Discard RANSAC outliers
    std::cout << "Discarding RANSAC outliers" << std::endl;
    RANSACRemoveOutlierTracks(1);
    RANSACRemoveOutlierTracks(GlobalParams::SecondRANSACNFrames() / 2);
    RANSACRemoveOutlierTracks(GlobalParams::SecondRANSACNFrames());

    std::cout << "Discarding tracks with too high change between consecutive features" << std::endl;
    RemoveBadDepthTracks();

    if (!IsStationary(prev_points, new_points, GlobalParams::StationaryThresh()))
    {
      std::lock_guard<std::mutex> lk(mu_);
      new_frame->stationary = false;
      frames.back()->stationary = false;  // When movement is registered between two frames, both are non-stationary
    }
  }

  // 30 -> 40ms when finding new features

  auto time_before_extraction = std::chrono::system_clock::now();
  if (GlobalParams::CountFeaturesPerCell())
  {
    DoFeatureExtractionPerCellPopulation(img_undistorted, new_frame, lidar_frame);
  }
  else if (active_tracks_.size() < GlobalParams::TrackCountLowerThresh())
  {
    DoFeatureExtractionByTotalCount(img_undistorted, new_frame, lidar_frame);
  }

  auto time_after_extraction = std::chrono::system_clock::now();
  auto micros_extraction =
      std::chrono::duration_cast<std::chrono::microseconds>(time_after_extraction - time_before_extraction);
  double millis_extraction = static_cast<double>(micros_extraction.count()) / 1000.;
  DebugValuePublisher::PublishFeatureExtractionDuration(millis_extraction);

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

  if (!GlobalParams::UseParallaxKeyframes() && new_frame->id % GlobalParams::TemporalKeyframeInterval() == 0)
  {
    new_frame->is_keyframe = true;
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

  if (frames.front()->timestamp < new_frame->timestamp - GlobalParams::SmootherLag() - 10.)
  {
    frames.front()->image.release();
    //std::cout << "frame use count" << frames.front().use_count() << std::endl;
    frames.pop_front();
  }

  UpdateTrackParallaxes();

  auto time_before_publish = std::chrono::system_clock::now();
  PublishLandmarksImage(new_frame, img_undistorted, lidar_frame);
  auto time_after_publish = std::chrono::system_clock::now();
  auto micros_publish = std::chrono::duration_cast<std::chrono::microseconds>(time_after_publish - time_before_publish);
  double millis_publish = static_cast<double>(micros_publish.count()) / 1000.;
  DebugValuePublisher::PublishImagePublishDuration(millis_publish);

  return new_frame;
}

void FeatureExtractor::ExtractNewCornersInUnderpopulatedGridCells(const Mat& img, vector<cv::Point2f>& corners,
                                                                  int cell_count_x, int cell_count_y,
                                                                  int min_features_per_cell,
                                                                  int max_initial_features_per_cell,
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

      cv::Rect mask(cell_x * cell_w, cell_y * cell_h, cell_w, cell_h);
      cv::Mat roi = img(mask);
      vector<cv::Point2f> corners_in_roi;
      goodFeaturesToTrack(roi, corners_in_roi, max_initial_features_per_cell, quality_level, min_distance);

      if (corners_in_roi.empty())
      {
        continue;  // Nothing further to do
      }

      // Do min eigenval check
      cv::Mat ev;
      cv::cornerMinEigenVal(roi, ev, 3);
      std::vector<bool> min_eigval_ok;
      min_eigval_ok.reserve(corners_in_roi.size());
      for (const auto& corner : corners_in_roi)
      {
        int x = static_cast<int>(corner.x);
        int y = static_cast<int>(corner.y);
        float f_lambda = ev.at<float>(y, x);
        auto d_lambda = static_cast<double>(f_lambda);
        min_eigval_ok.push_back(d_lambda > GlobalParams::FeatureExtractionMinEigenValue());
      }

      // Subpix refinement
      Size winSize = Size(5, 5);
      Size zeroZone = Size(-1, -1);
      TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001);
      cv::cornerSubPix(roi, corners_in_roi, winSize, zeroZone, criteria);

      for (int i = 0; i < corners_in_roi.size(); ++i)
      {
        if (PointWasSubPixRefined(corners_in_roi[i]) && min_eigval_ok[i])
        {
          corners.push_back(corners_in_roi[i] + cv::Point2f(cell_x * cell_w, cell_y * cell_h));
        }
      }
      cells_repopulated++;
    }
  }
  std::cout << "Detected new features in " << cells_repopulated << " cells" << std::endl;
  DebugValuePublisher::PublishNCellsRepopulated(cells_repopulated);
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

void FeatureExtractor::NonMaxSuppressFeatures(std::vector<std::shared_ptr<Feature>>& features,
                                              double squared_dist_thresh, int min_j)
{
  for (int i = 0; i < features.size(); ++i)
  {
    for (int j = static_cast<int>(features.size()) - 1; j > i && j > min_j; --j)
    {
      auto d_vec = (features[i]->pt - features[j]->pt);
      double d2 = d_vec.dot(d_vec);
      if (d2 < squared_dist_thresh)
      {
        features.erase(features.begin() + j);
      }
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
    else
    {
      // Draw parallax string
      std::stringstream stream1;
      stream1 << std::fixed << std::setprecision(2) << track->max_parallax << std::endl;
      std::string parallax_str = stream1.str();

      cv::putText(tracks_out_img.image, parallax_str, track->features.back()->pt + cv::Point2f(7., 20.),
                  cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 200));

      // Draw track length string
      std::stringstream stream2;
      stream2 << track->features.size() << std::endl;
      std::string len_str = stream2.str();

      cv::putText(tracks_out_img.image, len_str, track->features.back()->pt + cv::Point2f(7., 40.),
                  cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 200));
    }
  }

  tracks_pub_.publish(tracks_out_img.toImageMsg());
  std::cout << "track count: " << active_tracks_.size() << std::endl;
}

void FeatureExtractor::RemoveBadDepthTracks()
{
  std::lock_guard<std::mutex> lk(mu_);  // Lock for the whole method so that indexes are not changed
  // iterate backwards to not mess up vector when erasing
  for (int i = static_cast<int>(active_tracks_.size()) - 1; i >= 0; --i)
  {
    auto max_change = ComputeMaxTrackDepthChange(active_tracks_[i]);
    if (max_change > GlobalParams::MaxDepthDifferenceBeforeRemoval())
    {
      std::cout << "Removing track " << active_tracks_[i]->id << " because max depth change is " << max_change << " > "
                << GlobalParams::MaxDepthDifferenceBeforeRemoval() << std::endl;
      active_tracks_.erase(active_tracks_.begin() + i);
    }
  }
}

void FeatureExtractor::RANSACRemoveOutlierTracks(int n_frames)
{
  std::lock_guard<std::mutex> lk(mu_);  // Lock for the whole method so that indexes are not changed
  std::vector<int> track_indices;
  std::vector<cv::Point2f> points_1;
  std::vector<cv::Point2f> points_2;

  // Check that all frames in RANSAC window are non-stationary
  if (frames.size() < n_frames + 1)
  {
    return;
  }
  int last_frame_idx = static_cast<int>(frames.size()) - 1;
  for (int i = last_frame_idx; i >= 0 && i > last_frame_idx - n_frames; --i)
  {
    if (frames[i]->stationary)
    {
      std::cout << "Skipping RANSAC with n_frames " << n_frames << " because of stationary frames." << std::endl;
      return;
    }
  }

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

void FeatureExtractor::KLTDiscardBadTracks(const std::vector<uchar>& status, std::vector<cv::Point2f>& prev_points,
                                           std::vector<cv::Point2f>& new_points)

{
  std::lock_guard<std::mutex> lk(mu_);
  // iterate backwards to not mess up vector when erasing
  for (int i = static_cast<int>(prev_points.size()) - 1; i >= 0; --i)
  {
    if (status[i] != 1)
    {
      active_tracks_.erase(active_tracks_.begin() + i);
      prev_points.erase(prev_points.begin() + i);
      new_points.erase(new_points.begin() + i);
    }
  }
}

void FeatureExtractor::KLTInitNewFeatures(const std::vector<cv::Point2f>& new_points, std::shared_ptr<Frame>& new_frame,
                                          const boost::optional<std::shared_ptr<LidarFrame>>& lidar_frame)
{
  std::lock_guard<std::mutex> lk(mu_);
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

void FeatureExtractor::InitNewExtractedFeatures(const std::vector<cv::Point2f>& corners,
                                                std::shared_ptr<Frame>& new_frame,
                                                std::vector<std::shared_ptr<Feature>>& features,
                                                const boost::optional<std::shared_ptr<LidarFrame>>& lidar_frame)
{
  for (const auto& corner : corners)
  {
    auto new_feature = std::make_shared<Feature>(new_frame, corner);
    if (lidar_frame)
    {
      new_feature->depth = getFeatureDirectDepth(new_feature->pt, (*lidar_frame)->depth_image);
    }
    features.push_back(new_feature);
  }
}

void FeatureExtractor::DoFeatureExtractionPerCellPopulation(
    const cv::Mat& img, std::shared_ptr<Frame> new_frame,
    const boost::optional<std::shared_ptr<LidarFrame>>& lidar_frame)
{
  Mat_<int> feature_counts;
  MakeFeatureCountPerCellTable(img.cols, img.rows, GlobalParams::GridCellsX(), GlobalParams::GridCellsY(),
                               frames.empty() ? std::map<int, std::weak_ptr<Feature>>{} : frames.back()->features,
                               feature_counts);

  vector<Point2f> corners;
  ExtractNewCornersInUnderpopulatedGridCells(img, corners, GlobalParams::GridCellsX(), GlobalParams::GridCellsY(),
                                             GlobalParams::MinFeaturesPerCell(), 3 * GlobalParams::MaxFeaturesPerCell(),
                                             GlobalParams::FeatureExtractionQualityLevel(),
                                             GlobalParams::FeatureExtractionMinDistance());

  std::vector<std::shared_ptr<Feature>> new_features;
  InitNewExtractedFeatures(corners, new_frame, new_features, lidar_frame);

  SortFeaturesByDepthInPlace(new_features);

  std::vector<std::shared_ptr<Feature>> new_and_old_features;
  new_and_old_features.reserve(active_tracks_.size() + new_features.size());
  for (const auto& track : active_tracks_)
  {
    new_and_old_features.push_back(track->features.back());
  }

  // Put the new features behind the old ones so that they are prioritized less during NMS
  new_and_old_features.insert(new_and_old_features.end(), new_features.begin(), new_features.end());

  FeatureExtractor::NonMaxSuppressFeatures(new_and_old_features, GlobalParams::TrackNMSSquaredDistThresh(),
                                           static_cast<int>(active_tracks_.size()) - 1);

  {
    int cell_w = img.cols / GlobalParams::GridCellsX();
    int cell_h = img.rows / GlobalParams::GridCellsY();

    std::lock_guard<std::mutex> lk(mu_);
    for (int i = static_cast<int>(active_tracks_.size()); i < new_and_old_features.size(); ++i)
    {
      auto feat_cell_idx = GetCellIndex(new_and_old_features[i]->pt, cell_w, cell_h);

      if (feature_counts.at<int>(feat_cell_idx.y, feat_cell_idx.x) >= GlobalParams::MaxFeaturesPerCell())
      {
        continue;
      }
      auto new_track = std::make_shared<Track>(std::vector<std::shared_ptr<Feature>>{ new_and_old_features[i] });
      new_frame->features[new_track->id] = new_and_old_features[i];
      new_and_old_features[i]->track = new_track;
      active_tracks_.push_back(std::move(new_track));
      feature_counts.at<int>(feat_cell_idx.y, feat_cell_idx.x)++;
    }
  }
}

void FeatureExtractor::DoFeatureExtractionByTotalCount(const cv::Mat& img, std::shared_ptr<Frame> new_frame,
                                                       const boost::optional<std::shared_ptr<LidarFrame>>& lidar_frame)
{
  vector<Point2f> corners;
  // 30 -> 40ms
  FindGoodFeaturesToTrackGridded(img, corners, GlobalParams::GridCellsX(), GlobalParams::GridCellsY(),
                                 GlobalParams::MaxFeaturesPerCell(), GlobalParams::FeatureExtractionQualityLevel(),
                                 GlobalParams::FeatureExtractionMinDistance());

  std::vector<std::shared_ptr<Feature>> features;
  InitNewExtractedFeatures(corners, new_frame, features, lidar_frame);

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

void FeatureExtractor::UpdateTrackParallaxes()
{
  int count = 0;
  for (const auto& track : active_tracks_)
  {
    if (track->max_parallax < GlobalParams::MinParallaxForSmoothing())
    {
      auto parallax = smoother_.CalculateParallax(track);
      std::lock_guard<std::mutex> lk(mu_);
      track->max_parallax = std::max(parallax, track->max_parallax);
      count++;
    }
  }
  std::cout << "Calculated parallaxes for " << count << " tracks" << std::endl;
}
