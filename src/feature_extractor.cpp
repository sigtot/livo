#include "feature_extractor.h"

#include <cv_bridge/cv_bridge.h>

#include "global_params.h"
#include "Initializer.h"
#include "image_undistorter.h"
#include "lidar-depth.h"
#include "feature_helpers.h"
#include "debug_value_publisher.h"

#include <algorithm>
#include <utility>
#include <memory>

FeatureExtractor::FeatureExtractor(const ros::Publisher& tracks_pub, const ros::Publisher& high_delta_tracks_pub,
                                   const LidarFrameManager& lidar_frame_manager,
                                   std::shared_ptr<ImageUndistorter> image_undistorter, NewSmoother& smoother)
  : tracks_pub_(tracks_pub)
  , high_delta_tracks_pub_(high_delta_tracks_pub)
  , lidar_frame_manager_(lidar_frame_manager)
  , image_undistorter_(std::move(image_undistorter))
  , smoother_(smoother)
{
}

backend::FrontendResult FeatureExtractor::lkCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  std::cout << "Frontend processing new frame" << std::endl;
  cv::Mat img;
  PrepareImage(msg, img);

  shared_ptr<Frame> new_frame = make_shared<Frame>();
  new_frame->image = img;
  new_frame->id = frame_count_++;
  new_frame->timestamp = msg->header.stamp.toSec();
  new_frame->stationary = true;  // Assume stationary at first

  auto lidar_frame = lidar_frame_manager_.At(new_frame->timestamp);

  // Do processing that requires existing frames
  if (!frames.empty())
  {
    // Obtain prev image and points
    auto prev_img = frames.back()->image;
    std::vector<cv::Point2f> prev_points;
    GetPrevPoints(prev_points);

    // Use optical flow to calculate new point predictions
    std::vector<cv::Point2f> new_points;
    std::vector<uchar> status;
    KLTPredictFeatureLocations(prev_img, img, prev_points, new_points, status);
    KLTDiscardBadTracks(status, prev_points, new_points);
    KLTInitNewFeatures(new_points, new_frame, lidar_frame);

    // Compute parallaxes and increment inlier/outlier counts. Will reject outliers later.
    RANSACComputeParallaxesForTracks();

    // Reject tracks based on reprojection and depth
    RejectOutliersByLandmarkProjections(new_frame->id, new_frame->timestamp);
    RemoveBadDepthTracks();

    if (!IsStationary(prev_points, new_points, GlobalParams::StationaryThresh()))
    {
      new_frame->stationary = false;
      frames.back()->stationary = false;  // When movement is registered between two frames, both are non-stationary
    }
  }

  ExtractFeatures(img, new_frame, lidar_frame);

  RemoveTracksCloseToEdge(img.cols, img.rows);

  if (new_frame->id % GlobalParams::TemporalKeyframeInterval() == 0)
  {
    new_frame->is_keyframe = true;
  }

  frames.push_back(new_frame);

  int n_ransac_outliers = RemoveTracksByInlierRatio(GlobalParams::MinKeyframeFeatureInlierRatio());

  // Keep a constant maximum number of frames in memory
  if (frames.front()->timestamp < new_frame->timestamp - GlobalParams::SmootherLag() - 10.)
  {
    frames.pop_front();
  }

  PublishLandmarksImage(new_frame, img, lidar_frame);

  return backend::FrontendResult{ .frame_id = new_frame->id,
                                  .timestamp = new_frame->timestamp,
                                  .stationary = new_frame->stationary,
                                  .is_keyframe = new_frame->is_keyframe,
                                  .has_depth = new_frame->HasDepth(),
                                  .mature_tracks = GetMatureTracksForBackend(),
                                  .n_ransac_outliers = n_ransac_outliers };
}

void FeatureExtractor::PrepareImage(const sensor_msgs::Image::ConstPtr& img_msg, cv::Mat& img) const
{
  auto encoding =
      GlobalParams::ColorImage() ? sensor_msgs::image_encodings::TYPE_8UC3 : sensor_msgs::image_encodings::TYPE_8UC1;

  auto cvPtr = cv_bridge::toCvShare(img_msg, encoding);

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

  UndistortImage(img_resized, img);

  if (GlobalParams::PreProcess())
  {
    PreProcessImage(img);
  }
}

bool ParallaxOk(const std::shared_ptr<Track>& track)
{
  return (track->MedianParallax() / static_cast<double>(GlobalParams::MinTrackLengthForSmoothing())) *
             static_cast<double>(track->features.size()) >
         GlobalParams::MinParallaxForSmoothing();
}

bool NonDepthTrackIsOk(const std::shared_ptr<Track>& track)
{
  return track->features.size() > GlobalParams::MinTrackLengthForSmoothing() &&
         ParallaxOk(track) &&
         track->depth_hint < GlobalParams::LandmarkDistanceThreshold();
}

bool IsMatureDepthTrack(const std::shared_ptr<Track>& track)
{
  return track->HasDepth() && track->features.size() > GlobalParams::MinTrackLengthForSmoothingDepth() &&
         track->DepthFeatureCount() > GlobalParams::MinDepthMeasurementsForSmoothing() &&
         track->MedianParallax() > GlobalParams::MinParallaxForSmoothingDepth() &&
         track->LastDepth()->depth > 7 &&  // Without this, we sometimes get landmarks behind camera
         track->LastDepth()->depth < GlobalParams::MaxDepthForSmoothing();
}

bool FeatureExtractor::TrackIsMature(const std::shared_ptr<Track>& track) const
{
  // If it is already in the smoother, the track must be mature
  return smoother_.IsLandmarkTracked(track->id) || IsMatureDepthTrack(track) || NonDepthTrackIsOk(track);
}

std::vector<backend::Track> FeatureExtractor::GetMatureTracksForBackend() const
{
  std::vector<backend::Track> tracks;
  tracks.reserve(active_tracks_.size());
  auto time_before = std::chrono::system_clock::now();
  int n_added = 0;
  for (const auto& track : active_tracks_)
  {
    if (!TrackIsMature(track))
    {
      continue;
    }
    bool is_mature_depth_track = IsMatureDepthTrack(track);
    std::vector<backend::Feature> features;
    features.reserve(track->features.size());
    for (const auto& feature : track->features)
    {
      features.push_back(backend::Feature{ .track_id = track->id,
                                           .frame_id = feature->frame_id,
                                           .timestamp = feature->timestamp,
                                           .pt = feature->pt,
                                           .depth = is_mature_depth_track ? feature->depth : boost::none });
    }
    tracks.push_back(backend::Track{ .id = track->id,
                                     .max_parallax = track->max_parallax,
                                     .have_depth = is_mature_depth_track,
                                     .features = std::move(features) });
    ++n_added;
    if (n_added >= GlobalParams::MaxFeatures())
    {
      break;
    }
  }
  auto time_after = std::chrono::system_clock::now();
  auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(time_after - time_before);
  std::cout << "Getting mature tracks took " << millis.count() << "ms" << std::endl;
  return tracks;
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

void FeatureExtractor::PreProcessImage(cv::Mat& image) const
{
  cv::convertScaleAbs(image, image, GlobalParams::ContrastAlpha(), GlobalParams::ContrastBeta());
}

void FeatureExtractor::PublishLandmarksImage(const std::shared_ptr<Frame>& frame, const cv::Mat& img,
                                             const boost::optional<std::shared_ptr<LidarFrame>>& lidar_frame) const
{
  auto time_before_publish = std::chrono::system_clock::now();
  if (!GlobalParams::VisualizationEnabled())
  {
    return;
  }
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
    if (GlobalParams::DrawOnlyInSmootherLandmarks() && !smoother_.IsLandmarkTracked(track->id))
    {
      continue;
    }
    auto parallax = track->MedianParallax();
    double intensity = std::min(255., 255 * parallax / GlobalParams::MinParallaxForSmoothing());
    // double intensity = 255;
    auto non_stationary_color = track->HasDepth() ? cv::Scalar(intensity, 0, 0) : cv::Scalar(0, intensity, 0);
    // auto non_stationary_color = cv::Scalar(0, intensity, 0);
    auto color = frame->stationary ? cv::Scalar(255, 255, 0) : non_stationary_color;
    for (int i = static_cast<int>(track->features.size()) - 1; i >= 1 && track->features.size() - i < 15; --i)
    {
      cv::line(tracks_out_img.image, track->features[i - 1]->pt, track->features[i]->pt, color, 1);
    }
    cv::circle(tracks_out_img.image, track->features.back()->pt, 5, color, 1);
    //if (track->last_parallax)
    //{
    //  cv::circle(tracks_out_img.image, *track->last_parallax, 5, cv::Scalar(0, 255, 255), 1);
    //}
    if (track->last_landmark_projection)
    {
      cv::circle(tracks_out_img.image, *track->last_landmark_projection, GlobalParams::FeaturePredictionOutlierThresh(),
                 cv::Scalar(0, 255, 255), 1);
    }
    cv::putText(tracks_out_img.image, std::to_string(track->id), track->features.back()->pt + cv::Point2f(7., 7.),
                cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 0, 200));

    // Draw parallax string
    std::stringstream stream1;
    stream1 << std::fixed << std::setprecision(2) << parallax;
    std::string parallax_str = stream1.str();

    cv::putText(tracks_out_img.image, parallax_str, track->features.back()->pt + cv::Point2f(7., 20.),
                cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 0, 200));

    // Draw track length string
    std::stringstream stream2;
    stream2 << track->features.size();
    std::string len_str = stream2.str();

    cv::putText(tracks_out_img.image, len_str, track->features.back()->pt + cv::Point2f(7., 36.),
                cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 0, 200));
    if (track->HasDepth())
    {
      // Draw depth string
      auto depth = (*std::find_if(track->features.rbegin(), track->features.rend(),
                                  [](const std::shared_ptr<Feature>& feature) -> bool {
                                    return feature->depth.is_initialized();
                                  }))
                       ->depth;
      std::stringstream stream;
      stream << std::fixed << std::setprecision(2) << depth->depth;
      std::string depth_str = stream.str();
      cv::putText(tracks_out_img.image, depth_str + "m", track->features.back()->pt + cv::Point2f(7., 52.),
                  cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 0, 200));
    }
  }

  tracks_pub_.publish(tracks_out_img.toImageMsg());
  std::cout << "track count: " << active_tracks_.size() << std::endl;

  auto time_after_publish = std::chrono::system_clock::now();
  auto micros_publish = std::chrono::duration_cast<std::chrono::microseconds>(time_after_publish - time_before_publish);
  double millis_publish = static_cast<double>(micros_publish.count()) / 1000.;
  DebugValuePublisher::PublishImagePublishDuration(millis_publish);
}

void FeatureExtractor::RejectOutliersByLandmarkProjections(int frame_id, double timestamp)
{
  auto time_before = std::chrono::system_clock::now();
  std::vector<int> track_ids;
  track_ids.reserve(active_tracks_.size());
  for (const auto& track : active_tracks_)
  {
    track_ids.push_back(track->id);
  }

  std::vector<boost::optional<cv::Point2f>> projections;
  auto success = smoother_.ProjectLandmarksIntoFrame(track_ids, frame_id, timestamp, projections);
  if (!success)
  {
    return;
  }
  // iterate backwards to not mess up vector when erasing
  for (int i = static_cast<int>(projections.size()) - 1; i >= 0; --i)
  {
    active_tracks_[i]->last_landmark_projection = projections[i];
    if (projections[i])
    {
      auto d = cv::norm((*projections[i] - active_tracks_[i]->features.back()->pt));
      if (d > GlobalParams::FeaturePredictionOutlierThresh()) {
        std::cout << "Erasing track " << i << " bc reproj error is too off: " << d << std::endl;
        active_tracks_.erase(active_tracks_.begin() + i);
      }
    }
  }
  auto time_after = std::chrono::system_clock::now();
  auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(time_after - time_before);
  DebugValuePublisher::PublishReprojectionRejectionDuration(static_cast<double>(millis.count()));
}

void FeatureExtractor::RemoveBadDepthTracks()
{
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

void FeatureExtractor::KLTPredictFeatureLocations(const cv::Mat& prev_img, const cv::Mat& new_img,
                                                  const std::vector<cv::Point2f>& prev_points,
                                                  std::vector<cv::Point2f>& new_points,
                                                  std::vector<uchar>& status)
{
  std::vector<float> err;
  TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), GlobalParams::KLTMaxIterations(),
                                       GlobalParams::KLTConvergenceEpsilon());

  auto time_before_klt = std::chrono::system_clock::now();
  cv::calcOpticalFlowPyrLK(prev_img, new_img, prev_points, new_points, status, err,
                           Size(GlobalParams::KLTWinSize(), GlobalParams::KLTWinSize()), GlobalParams::KLTPyramids(),
                           criteria);

  auto time_after_klt = std::chrono::system_clock::now();
  auto micros_klt = std::chrono::duration_cast<std::chrono::microseconds>(time_after_klt - time_before_klt);
  double millis_klt = static_cast<double>(micros_klt.count()) / 1000.;
  DebugValuePublisher::PublishKLTDuration(millis_klt);
}

void FeatureExtractor::KLTDiscardBadTracks(const std::vector<uchar>& status, std::vector<cv::Point2f>& prev_points,
                                           std::vector<cv::Point2f>& new_points)

{
  // iterate backwards to not mess up active_tracks_ when erasing
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
  for (int i = 0; i < new_points.size(); ++i)
  {
    auto new_feature = std::make_shared<Feature>(new_points[i], new_frame->id, new_frame->timestamp, active_tracks_[i]);
    auto depth = MaybeGetDepth(new_feature->pt, lidar_frame);
    if (depth)
    {
      // We set the depth hint regardless of whether it is from a valid depth result.
      // Invalid depth information, while not reliable to use as a measurement, can be used to reject far-points.
      active_tracks_[i]->depth_hint = std::max(active_tracks_[i]->depth_hint, depth->depth);
    }
    new_feature->depth = CheckDepthResult(depth);
    new_frame->features[active_tracks_[i]->id] = new_feature;
    active_tracks_[i]->AddFeature(std::move(new_feature));
  }
}

void FeatureExtractor::InitNewExtractedFeatures(const std::vector<cv::Point2f>& corners,
                                                std::shared_ptr<Frame>& new_frame,
                                                std::vector<std::shared_ptr<Feature>>& features,
                                                const boost::optional<std::shared_ptr<LidarFrame>>& lidar_frame)
{
  for (const auto& corner : corners)
  {
    auto new_feature = std::make_shared<Feature>(corner, new_frame->id, new_frame->timestamp);
    new_feature->depth = CheckDepthResult(MaybeGetDepth(new_feature->pt, lidar_frame));
    features.push_back(new_feature);
  }
}

void FeatureExtractor::ExtractFeatures(
    const cv::Mat& img, std::shared_ptr<Frame> new_frame,
    const boost::optional<std::shared_ptr<LidarFrame>>& lidar_frame)
{
  auto time_before_extraction = std::chrono::system_clock::now();
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

  NonMaxSuppressFeatures(new_and_old_features, GlobalParams::TrackNMSSquaredDistThresh(),
                                           static_cast<int>(active_tracks_.size()) - 1);

  {
    int cell_w = img.cols / GlobalParams::GridCellsX();
    int cell_h = img.rows / GlobalParams::GridCellsY();

    for (int i = static_cast<int>(active_tracks_.size()); i < new_and_old_features.size(); ++i)
    {
      auto feat_cell_idx = GetCellIndex(new_and_old_features[i]->pt, cell_w, cell_h);

      if (feature_counts.at<int>(feat_cell_idx.y, feat_cell_idx.x) >= GlobalParams::MaxFeaturesPerCell())
      {
        continue;
      }
      auto new_track = std::make_shared<Track>(new_and_old_features[i]);
      new_frame->features[new_track->id] = new_and_old_features[i];
      new_and_old_features[i]->track = new_track;
      active_tracks_.push_back(std::move(new_track));
      feature_counts.at<int>(feat_cell_idx.y, feat_cell_idx.x)++;
    }
  }

  auto time_after_extraction = std::chrono::system_clock::now();
  auto micros_extraction =
      std::chrono::duration_cast<std::chrono::microseconds>(time_after_extraction - time_before_extraction);
  double millis_extraction = static_cast<double>(micros_extraction.count()) / 1000.;
  DebugValuePublisher::PublishFeatureExtractionDuration(millis_extraction);
}

void FeatureExtractor::PublishSingleTrackImage(const backend::Track& track)
{
  std::cout << "Publishing single track image" << std::endl;
  auto frame_id = track.features.back().frame_id;
  // Search backwards because last features is likely in one of the most recent frames
  auto frame = std::find_if(frames.rbegin(), frames.rend(),
                            [frame_id](const std::shared_ptr<Frame>& frame) -> bool { return frame->id == frame_id; });
  cv_bridge::CvImage out_img;
  out_img.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
  out_img.header.stamp = ros::Time(frames.back()->timestamp);
  out_img.header.seq = frames.back()->id;
  cvtColor((*frame)->image, out_img.image, CV_GRAY2RGB);

  cv::Scalar color(0, 0, 255);
  std::cout << "This track has a length of " << track.features.size() << std::endl;
  for (int i = static_cast<int>(track.features.size()) - 1; i >= 1; --i)
  {
    cv::line(out_img.image, track.features[i - 1].pt, track.features[i].pt, color, 1);
  }
  cv::circle(out_img.image, track.features.back().pt, 5, color, 1);
  high_delta_tracks_pub_.publish(out_img.toImageMsg());
}

void FeatureExtractor::RANSACComputeParallaxesForTracks(int ransac_interval)
{
  /// What interval we want between the frames we perform RANSAC on
  if (frames.size() <= ransac_interval)
  {
    // Nothing to do yet. Wait until we have enough frames.
    return;
  }

  // Obtain feature correspondences
  std::vector<cv::Point2f> old_matches;
  std::vector<cv::Point2f> new_matches;
  std::vector<int> track_indices; // Also keep track of track indices for later outlier removal
  for (int i = 0; i < active_tracks_.size(); ++i)
  {
    if (active_tracks_[i]->features.size() > ransac_interval)
    {
      auto old_feature = active_tracks_[i]->features[active_tracks_[i]->features.size() - 1 - ransac_interval];
      auto new_feature = active_tracks_[i]->features.back();
      old_matches.push_back(old_feature->pt);
      new_matches.push_back(new_feature->pt);
      track_indices.push_back(i);
    }
  }

  // Compute parallaxes
  std::vector<double> parallaxes;
  std::vector<cv::Point2f> parallax_proj_points;
  std::vector<uchar> inliers;
  bool success = ComputeParallaxesAndInliers(old_matches, new_matches, image_undistorter_->GetRefinedCameraMatrix(),
                                             parallaxes, parallax_proj_points, inliers);

  // If parallax computation failed, we do not want to use the result for either parallax or outliers
  if(!success)
  {
    return;
  }

  // Parallax computation succeeded. Update parallaxes and inlier/outlier counts
  for (int i = 0; i < parallaxes.size(); ++i)
  {
    auto track_idx = track_indices[i];
    if (inliers[i])
    {
      active_tracks_[track_idx]->inlier_count++;
    }
    else
    {
      active_tracks_[track_idx]->outlier_count++;
    }
    active_tracks_[track_idx]->parallaxes.push_back(parallaxes[i]);
    active_tracks_[track_idx]->last_parallax = parallax_proj_points[i];
  }
}

int FeatureExtractor::RemoveTracksByInlierRatio(double inlier_ratio)
{
  int outliers_removed = 0;
  for (int i = static_cast<int>(active_tracks_.size()) - 1; i >= 0; --i)
  {
    if ((active_tracks_[i]->inlier_count > 0 || active_tracks_[i]->outlier_count > 0) &&
        active_tracks_[i]->InlierRatio() < inlier_ratio)
    {
      active_tracks_.erase(active_tracks_.begin() + i);
      outliers_removed++;
    }
  }
  return outliers_removed;
}

void FeatureExtractor::RemoveTracksCloseToEdge(int width, int height)
{
  for (int i = static_cast<int>(active_tracks_.size()) - 1; i >= 0; --i)
  {
    if (IsCloseToImageEdge(active_tracks_[i]->features.back()->pt, width, height,
                           GlobalParams::ImageEdgePaddingPercent()))
    {
      active_tracks_.erase(active_tracks_.begin() + i);
    }
  }
}

void FeatureExtractor::GetPrevPoints(std::vector<cv::Point2f>& prev_points) const
{
  for (const auto& track : active_tracks_)
  {
    prev_points.push_back(track->features.back()->pt);
  }
}
