#include "feature_extractor.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>

#include "global_params.h"
#include "match_result.h"
#include "Initializer.h"

#include <algorithm>
#include <utility>

FeatureExtractor::FeatureExtractor(const ros::Publisher& matches_pub, const ros::Publisher& tracks_pub, int lag)
  : matches_pub_(matches_pub), tracks_pub_(tracks_pub), lag(lag), orb_extractor()
{
}

shared_ptr<Frame> FeatureExtractor::lkCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  auto cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);  // TODO perf maybe toCvShare?
  Mat img_resized;
  resize(cvPtr->image, img_resized, Size(), GlobalParams::ResizeFactor(), GlobalParams::ResizeFactor(), INTER_LINEAR);

  shared_ptr<Frame> new_frame = make_shared<Frame>();
  new_frame->image = img_resized;
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
    cv::calcOpticalFlowPyrLK(prev_img, img_resized, prev_points, new_points, status, err, Size(15, 15), 2, criteria);

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

    // Discard RANSAC outliers
    vector<uchar> inlier_mask;
    if (new_points.size() >= 8)
    {
      auto F = findFundamentalMat(prev_points, new_points, CV_FM_RANSAC, 3., 0.99, inlier_mask);
      for (int i = static_cast<int>(prev_points.size()) - 1; i >= 0;
           --i)  // iterate backwards to not mess up vector when erasing
      {
        if (inlier_mask[i])
        {
          active_tracks_[i]->features.push_back(std::make_shared<Feature>(new_frame, new_points[i]));
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
    cvtColor(img_resized, tracks_out_img.image, CV_GRAY2RGB);


    for (const auto& track : active_tracks_)
    {
      double intensity = std::min(255., 255 * track->max_parallax / GlobalParams::MinParallax());
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
    FindGoodFeaturesToTrackGridded(img_resized, corners, 9, 7, GlobalParams::MaxFeaturesPerCell(), 0.3, 7);
    for (const auto& corner : corners)
    {
      active_tracks_.push_back(std::make_shared<Track>(
          std::vector<std::shared_ptr<Feature>>{ std::make_shared<Feature>(new_frame, corner) }));
    }
    NonMaxSuppressTracks(GlobalParams::TrackNMSSquaredDistThresh());
  }

  for (int i = static_cast<int>(active_tracks_.size()) - 1; i >= 0; --i)
  {
    if (IsCloseToImageEdge(active_tracks_[i]->features.back()->pt, img_resized.cols, img_resized.rows,
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
    if (active_tracks_[i]->key_features.size() >= GlobalParams::MinTrackLengthForSmoothing() &&
        active_tracks_[i]->InlierRatio() < GlobalParams::MinKeyframeFeatureInlierRatio())
    {
      active_tracks_.erase(active_tracks_.begin() + i);
    }
  }

  return new_frame;
}

shared_ptr<Frame> FeatureExtractor::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  auto cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);  // Makes copy. We can also share to
  // increase performance
  Mat img_resized;
  resize(cvPtr->image, img_resized, Size(), GlobalParams::ResizeFactor(), GlobalParams::ResizeFactor(), INTER_LINEAR);

  Ptr<Feature2D> orb = ORB::create(GlobalParams::MaxFeaturesPerCell());

  vector<KeyPoint> keypoints;
  Mat descriptors;

  orb_extractor(img_resized, cv::Mat(), keypoints, descriptors);

  // Register observations in new frame
  shared_ptr<Frame> new_frame = make_shared<Frame>();
  new_frame->image = img_resized;
  new_frame->id = frame_count_++;
  new_frame->timestamp = msg->header.stamp.toSec();
  for (int i = 0; i < keypoints.size(); ++i)
  {
    shared_ptr<KeyPointObservation> observation =
        make_shared<KeyPointObservation>(keypoints[i], descriptors.row(i), new_frame);
    new_frame->keypoint_observations.push_back(move(observation));
  }

  // Perform matching and create new landmarks
  if (!frames.empty())
  {
    // Match with existing landmarks
    std::vector<LandmarkMatch> landmark_matches;
    std::vector<std::shared_ptr<KeyPointObservation>> unmatched_observations;
    GetLandmarkMatches(new_frame->keypoint_observations, unmatched_observations, landmark_matches);
    for (auto& landmark_match : landmark_matches)
    {
      landmark_match.landmark_->keypoint_observations.push_back(landmark_match.observation_);
      auto obs = landmark_match.observation_;
      auto lm = weak_ptr<Landmark>(landmark_match.landmark_);
      auto obs_value = *obs;
      landmark_match.observation_->landmark = weak_ptr<Landmark>(landmark_match.landmark_);
    }
    std::cout << unmatched_observations.size() << " remaining of " << new_frame->keypoint_observations.size()
              << " observations after matching with existing landmarks." << std::endl;

    std::vector<KeyPoint> unmatched_keypoints;
    cv::Mat unmatched_descriptors;
    for (auto& observation : unmatched_observations)
    {
      unmatched_keypoints.push_back(observation->keypoint);
      unmatched_descriptors.push_back(observation->descriptor);
    }

    auto prev_frame_unmatched_observations = frames.back()->GetUnmatchedObservations();
    std::vector<KeyPoint> prev_frame_unmatched_keypoints;
    cv::Mat prev_frame_unmatched_descriptors;
    for (auto& observation : prev_frame_unmatched_observations)
    {
      prev_frame_unmatched_keypoints.push_back(observation->keypoint);
      prev_frame_unmatched_descriptors.push_back(observation->descriptor);
    }
    MatchResult match_result;
    getMatches(unmatched_descriptors, unmatched_keypoints, prev_frame_unmatched_descriptors,
               prev_frame_unmatched_keypoints, match_result.matches, match_result.inliers);

    // Remove dupes, outliers and bad matches
    std::map<int, cv::DMatch> best_matches_by_train_idx;
    for (int i = 0; i < unmatched_observations.size(); ++i)
    {
      auto match = match_result.matches[i];
      // train and query idx can be -1 for some reason?
      if (match_result.inliers[i] && match.queryIdx >= 0 && match.queryIdx < unmatched_observations.size() &&
          match.trainIdx >= 0 && match.trainIdx < prev_frame_unmatched_observations.size() &&
          match.distance < GlobalParams::MatchMaxDistance())
      {
        auto dupe_match = best_matches_by_train_idx.find(match.trainIdx);
        if (dupe_match != best_matches_by_train_idx.end())
        {
          if (match.distance < dupe_match->second.distance)
          {
            best_matches_by_train_idx[match.trainIdx] = match;
          }  // else discarded ...
        }
        else
        {
          best_matches_by_train_idx[match.trainIdx] = match;
        }
      }
    }

    // Init new landmarks
    for (auto& match_by_train_idx : best_matches_by_train_idx)
    {
      auto match = match_by_train_idx.second;

      // Init new landmark
      shared_ptr<Landmark> new_landmark = make_shared<Landmark>(Landmark());
      new_landmark->id = landmark_count_++;

      // Add weak_ptr to previous frame
      new_landmark->keypoint_observations.push_back(prev_frame_unmatched_observations[match.trainIdx]);
      prev_frame_unmatched_observations[match.trainIdx]->landmark = std::weak_ptr<Landmark>(new_landmark);

      // Also add to the new_landmarks vector
      frames.back()->new_landmarks.push_back(std::weak_ptr<Landmark>(new_landmark));

      // Add weak_ptr to new frame
      new_landmark->keypoint_observations.push_back(unmatched_observations[match.queryIdx]);
      unmatched_observations[match.queryIdx]->landmark = std::weak_ptr<Landmark>(new_landmark);

      // Move shared_ptr to landmarks map
      landmarks[new_landmark->id] = move(new_landmark);
    }
  }

  // Persist new frame
  frames.push_back(new_frame);

  return new_frame;
}

void FeatureExtractor::getMatches(const Mat& query_descriptors, const vector<KeyPoint>& query_keypoints,
                                  const cv::Mat& train_descriptors, const vector<KeyPoint>& train_keypoints,
                                  vector<DMatch>& matches, vector<uchar>& outlier_mask)
{
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);
  matcher->match(query_descriptors, train_descriptors, matches);

  vector<Point> src_points;
  vector<Point> dst_points;
  for (auto match : matches)
  {
    src_points.push_back(query_keypoints[match.queryIdx].pt);
    dst_points.push_back(train_keypoints[match.trainIdx].pt);
  }

  // Homography is much better than fundamental matrix for whatever reason.
  // Could be due to low parallax
  // findHomography(src_points, dst_points, CV_RANSAC, 3, outlier_mask);
  findFundamentalMat(src_points, dst_points, CV_FM_RANSAC, 3., 0.99, outlier_mask);
}

pair<shared_ptr<Frame>, shared_ptr<Frame>> FeatureExtractor::getFirstTwoFrames()
{
  if (frame_count_ >= 2)
  {
    return pair<shared_ptr<Frame>, shared_ptr<Frame>>(frames[0], frames[1]);
  }
  else
  {
    throw NotEnoughFramesException();
  }
}

void FeatureExtractor::PublishLandmarkTracksImage()
{
  cv_bridge::CvImage tracks_out_img;
  tracks_out_img.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
  tracks_out_img.header.stamp = ros::Time(frames.back()->timestamp);
  tracks_out_img.header.seq = frames.back()->id;
  cvtColor(frames.back()->image, tracks_out_img.image, CV_GRAY2RGB);
  for (const auto& landmark_pair : landmarks)
  {
    auto landmark = landmark_pair.second;
    if (landmark->keypoint_observations.size() > 1 &&
        landmark->keypoint_observations.back()->frame->id > frame_count_ - 5)
    {
      int obsCount = static_cast<int>(landmark->keypoint_observations.size());
      for (int k = 1; k < obsCount; ++k)
      {
        line(tracks_out_img.image, landmark->keypoint_observations[k]->keypoint.pt,
             landmark->keypoint_observations[k - 1]->keypoint.pt, Scalar(255, 0, 0), 1);
      }
      Point point = landmark->keypoint_observations.back()->keypoint.pt;
      if (landmark->keypoint_observations.front()->frame->id < frame_count_ - GlobalParams::LandmarkCullingFrameCount())
      {
        // Landmark has passed the culling criteria so we draw it in green
        circle(tracks_out_img.image, point, 5, Scalar(0, 255, 0), 2);
      }
      else
      {
        // Landmark might not have passed the culling criteria: draw in red
        circle(tracks_out_img.image, point, 3, Scalar(0, 0, 255), 1);
      }
      /*
      putText(tracks_out_img.image,
              to_string(landmark->id),  // text
              point + Point(5, 5), FONT_HERSHEY_DUPLEX, 0.3,
              CV_RGB(255, 0, 0),  // font color
              1);
              */
    }
  }
  tracks_pub_.publish(tracks_out_img.toImageMsg());
}

void FeatureExtractor::GetLandmarkMatches(vector<shared_ptr<KeyPointObservation>> new_observations,
                                          std::vector<std::shared_ptr<KeyPointObservation>>& remaining_observations,
                                          vector<LandmarkMatch>& landmark_matches)
{
  std::map<int, vector<shared_ptr<Landmark>>> landmarks_table;
  for (auto& landmark_pair : landmarks)
  {
    auto landmark = landmark_pair.second;
    if (landmark->GetLastObservationFrameId() > frame_count_ - GlobalParams::LandmarkCullingFrameCount())
    {
      landmarks_table[landmark->GetLastObservationFrameId()].push_back(landmark);
    }
  }

  for (auto& x : landmarks_table)
  {
    int frame_id = x.first;  // TODO unused. maybe use a vector instead of a map
    auto landmarks_in_frame = x.second;
    if (landmarks_in_frame.size() < 8)
    {
      continue;
    }
    std::vector<KeyPoint> landmark_keypoints;
    cv::Mat landmark_descriptors;
    for (auto& landmark : landmarks_in_frame)
    {
      landmark_keypoints.push_back(landmark->GetNewestKeyPoint());
      landmark_descriptors.push_back(landmark->GetNewestDescriptor());
    }

    std::vector<KeyPoint> new_keypoints;
    cv::Mat new_descriptors;
    for (auto& observation : new_observations)
    {
      new_keypoints.push_back(observation->keypoint);
      new_descriptors.push_back(observation->descriptor);
    }

    MatchResult match_result;
    getMatches(landmark_descriptors, landmark_keypoints, new_descriptors, new_keypoints, match_result.matches,
               match_result.inliers);

    // Remove dupes, outliers and bad matches
    std::map<int, cv::DMatch> best_matches_by_train_idx;
    for (int i = 0; i < landmarks_in_frame.size(); ++i)
    {
      auto match = match_result.matches[i];
      // train and query idx can be -1 for some reason?
      // and larger than new_observations.size somehow? this was maybe due to a dereference bug. TODO: investigate
      if (match_result.inliers[i] && match.queryIdx >= 0 && match.queryIdx < new_observations.size() &&
          match.trainIdx >= 0 && match.trainIdx < new_observations.size() &&
          match.distance < GlobalParams::MatchMaxDistance())
      {
        auto dupe_match = best_matches_by_train_idx.find(match.trainIdx);
        if (dupe_match != best_matches_by_train_idx.end())
        {
          if (match.distance < dupe_match->second.distance)
          {
            best_matches_by_train_idx[match.trainIdx] = match;
          }  // else discarded ...
        }
        else
        {
          best_matches_by_train_idx[match.trainIdx] = match;
        }
      }
    }

    std::map<int, bool> observations_to_remove;
    for (auto& match_by_train_idx : best_matches_by_train_idx)
    {
      auto match = match_by_train_idx.second;
      landmark_matches.push_back(
          LandmarkMatch{ landmarks_in_frame[match.queryIdx], new_observations[match.trainIdx], match.distance });
      observations_to_remove[match.trainIdx] = true;
    }

    // Remove-erase all the matched observations for the next matching iteration
    // TODO: This will likely be faster if we index directly instead of applying the remove_if predicate to all items
    {
      auto i = new_observations.size() - 1;
      new_observations.erase(std::remove_if(new_observations.begin(), new_observations.end(),
                                            [&observations_to_remove, &i](const shared_ptr<KeyPointObservation>& obs) {
                                              return observations_to_remove.count(i--);
                                            }),
                             new_observations.end());
    }
  }
  remaining_observations = new_observations;
}

int FeatureExtractor::GetLandmarkCount()
{
  return landmarks.size();
}

int FeatureExtractor::GetFrameCount()
{
  return frames.size();
}

bool cellIsInCenter(int cell, int cell_count)
{
  if (cell_count % 2 == 0)
  {
    return cell == cell_count / 2 || cell == cell_count / 2 - 1;
  }
  else
  {
    return cell == cell_count / 2;
  }
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
      /*
      if (cellIsInCenter(cell_x, cell_count_x) && cellIsInCenter(cell_y, cell_count_y))
      {
        std::cout << "skipping cell" << cell_x << "," << cell_y << " because it is in the center" << std::endl;
        continue;
      }
       */
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

vector<shared_ptr<Frame>> FeatureExtractor::GetFrames()
{
  return frames;
}

map<int, shared_ptr<Landmark>> FeatureExtractor::GetLandmarks()
{
  return landmarks;
}
void FeatureExtractor::CullLandmarks(int frame_window, double min_obs_percentage)
{
  if (frame_count_ < frame_window)
  {
    return;
  }
  auto frame = frames[frame_count_ - frame_window];
  for (auto& landmark_weak : frame->new_landmarks)
  {
    auto landmark = landmark_weak.lock();
    if (landmark)
    {
      if (landmark->keypoint_observations.size() < min_obs_percentage * frame_window)
      {
        CullLandmark(landmark->id);
      }
    }
  }
}

void FeatureExtractor::CullLandmark(int landmark_id)
{
  landmarks.erase(landmark_id);
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

std::vector<shared_ptr<Track>> FeatureExtractor::GetActiveTracks()
{
  return active_tracks_;
}

std::vector<shared_ptr<Track>> FeatureExtractor::GetOldTracks()
{
  return old_tracks_;
}

std::vector<shared_ptr<Track>> FeatureExtractor::GetActiveHighParallaxTracks()
{
  std::vector<shared_ptr<Track>> tracks;
  for (auto & track : active_tracks_)
  {
    if(track->max_parallax >= GlobalParams::MinParallax() / 2)
    {
      tracks.push_back(track);
    }
  }
  return tracks;
}

std::vector<shared_ptr<Track>> FeatureExtractor::GetHighParallaxTracks()
{
  std::vector<shared_ptr<Track>> tracks;
  for (auto & track : old_tracks_)
  {
    if(track->max_parallax >= GlobalParams::MinParallax() / 2)
    {
      tracks.push_back(track);
    }
  }
  for (auto & track : active_tracks_)
  {
    if(track->max_parallax >= GlobalParams::MinParallax() / 2)
    {
      tracks.push_back(track);
    }
  }
  return tracks;
}

std::vector<KeyframeTransform> FeatureExtractor::GetValidKeyframeTransforms() const
{
  return keyframe_tracker_ ? keyframe_tracker_->GetGoodKeyframeTransforms() : std::vector<KeyframeTransform>{};
}

std::vector<KeyframeTransform> FeatureExtractor::GetKeyframeTransforms() const
{
  return keyframe_tracker_ ? keyframe_tracker_->GetKeyframeTransforms() : std::vector<KeyframeTransform>{};
}

bool FeatureExtractor::ReadyForInitialization() const
{
  return keyframe_tracker_ && keyframe_tracker_->GoodForInitialization();
}

bool FeatureExtractor::CanPerformStationaryIMUInitialization() const
{
  bool perform_stationary_imu_update = !frames.back()->stationary;  // Last frame is non-stationary...
  for (size_t i = frames.size() - 7; perform_stationary_imu_update && i < frames.size() - 2; ++i)
  {
    perform_stationary_imu_update = frames[i]->stationary;  // ... and the preceding frames are stationary
  }
  return perform_stationary_imu_update;
}
