#include "feature_extractor.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>

#include "global_params.h"
#include "match_result.h"
#include "match_in_frame.h"

#include <algorithm>

FeatureExtractor::FeatureExtractor(const ros::Publisher& matches_pub,
                                   const ros::Publisher& tracks_pub, int lag)
    : matches_pub_(matches_pub),
      tracks_pub_(tracks_pub),
      lag(lag),
      orb_extractor() {}

shared_ptr<Frame> FeatureExtractor::imageCallback(
    const sensor_msgs::Image::ConstPtr& msg) {
  auto cvPtr = cv_bridge::toCvCopy(
      msg,
      sensor_msgs::image_encodings::
          TYPE_8UC1);  // Makes copy. We can also share to increase performance
  Mat img_resized;
  resize(cvPtr->image, img_resized, Size(), GlobalParams::ResizeFactor(),
         GlobalParams::ResizeFactor(), INTER_LINEAR);

  Ptr<Feature2D> orb = ORB::create(GlobalParams::MaxFeaturesPerCell());

  vector<KeyPoint> keypoints;
  Mat descriptors;

  vector<cv::Point2f> corners;
  orb_extractor(img_resized, cv::Mat(), keypoints, descriptors);

  for (auto& corner : corners) {
    keypoints.emplace_back(corner, 1);
  }

  orb->compute(img_resized, keypoints, descriptors);
  keypoints.insert(keypoints.end(), keypoints.begin(), keypoints.end());
  descriptors.push_back(descriptors);

  // Register observations in new frame
  shared_ptr<Frame> new_frame = make_shared<Frame>();
  new_frame->image = img_resized;
  new_frame->id = frame_count_++;
  new_frame->timestamp = msg->header.stamp.toSec();
  for (int i = 0; i < keypoints.size(); ++i) {
    shared_ptr<KeyPointObservation> observation =
        make_shared<KeyPointObservation>(keypoints[i], descriptors.row(i),
                                         new_frame);
    new_frame->keypoint_observations.push_back(move(observation));
  }

  // Perform matching and create new landmarks
  if (!frames.empty()) {
    MatchResult match_result;
    getMatches(frames.back(), descriptors, keypoints, match_result.matches,
               match_result.inliers);

    vector<MatchInFrame> good_matches;
    for (int i = 0; i < keypoints.size(); ++i) {
      if (match_result.inliers[i] && match_result.matches[i].queryIdx >= 0 &&
          match_result.matches[i].trainIdx >= 0 &&  // can be -1, idk why
          match_result.matches[i].distance < 20) {
        good_matches.push_back(MatchInFrame{.match = match_result.matches[i],
                                            .frame = frames.back()});
      }
    }

    // You cannot observe the same landmark more than once in a frame
    map<int, MatchInFrame> existing_landmark_matches;
    map<int, MatchInFrame> no_landmark_matches;
    for (auto& match : good_matches) {
      auto landmark = match.frame->keypoint_observations[match.match.trainIdx]
                          ->landmark.lock();
      if (landmark) {
        auto existing_landmark_match =
            existing_landmark_matches.find(landmark->id);
        if (existing_landmark_match != existing_landmark_matches.end()) {
          if (match.match.distance >
              existing_landmark_match->second.match.distance) {
            existing_landmark_matches[landmark->id] = match;
          }
        } else {
          existing_landmark_matches[landmark->id] = match;
        }
      } else {
        auto existing_no_landmark_match =
            no_landmark_matches.find(match.match.trainIdx);
        if (existing_no_landmark_match != no_landmark_matches.end()) {
          if (match.match.distance >
              existing_no_landmark_match->second.match.distance) {
            no_landmark_matches[match.match.trainIdx] = match;
          }
        } else {
          no_landmark_matches[match.match.trainIdx] = match;
        }
      }
    }

    for (auto& match_in_frame_it : existing_landmark_matches) {
      auto match_in_frame = match_in_frame_it.second;
      auto existing_landmark =
          match_in_frame.frame
              ->keypoint_observations[match_in_frame.match.trainIdx]
              ->landmark.lock();
      // Add observation for existing landmark
      // cout << "Add obs to existing landmark: " << "new frame " <<
      // newFrame->id << ", old frame" << matchInFrame.frame->id << " landmark
      // id " << existingLandmark->id << endl;
      new_frame->keypoint_observations[match_in_frame.match.queryIdx]
          ->landmark = weak_ptr<Landmark>(existing_landmark);
      existing_landmark->keypoint_observations.push_back(
          new_frame->keypoint_observations[match_in_frame.match.queryIdx]);
    }

    for (auto& match_in_frame_it : no_landmark_matches) {
      auto match_in_frame = match_in_frame_it.second;
      // Init new landmark
      shared_ptr<Landmark> new_landmark = make_shared<Landmark>(Landmark());
      new_landmark->id = landmark_count_++;
      new_landmark->keypoint_observations.push_back(
          match_in_frame.frame
              ->keypoint_observations[match_in_frame.match.trainIdx]);
      new_landmark->keypoint_observations.push_back(
          new_frame->keypoint_observations[match_in_frame.match.queryIdx]);
      match_in_frame.frame->keypoint_observations[match_in_frame.match.trainIdx]
          ->landmark = weak_ptr<Landmark>(new_landmark);
      new_frame->keypoint_observations[match_in_frame.match.queryIdx]
          ->landmark = weak_ptr<Landmark>(new_landmark);
      match_in_frame.frame->new_landmarks.push_back(
          weak_ptr<Landmark>(new_landmark));

      // cout << "Add new landmark: " << "new frame" << newFrame->id << ", old
      // frame" << matchInFrame.frame->id << " landmark id" << newLandmark->id
      // << endl;

      landmarks.push_back(move(new_landmark));
    }

    // cout << unmatchedIndices.size() << " observations were not matched" <<
    // endl;
  }

  // Persist new frame
  frames.push_back(new_frame);

  return new_frame;
}

void FeatureExtractor::getMatches(const shared_ptr<Frame>& frame,
                                  const Mat& descriptors,
                                  const vector<KeyPoint>& keypoints,
                                  vector<DMatch>& matches,
                                  vector<uchar>& outlier_mask) {
  Ptr<DescriptorMatcher> matcher =
      DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);
  auto prev_key_points = frame->getKeyPoints();
  auto prev_descriptors = frame->getDescriptors();

  matcher->match(descriptors, prev_descriptors, matches);

  vector<Point> src_points;
  vector<Point> dst_points;
  for (auto match : matches) {
    src_points.push_back(keypoints[match.queryIdx].pt);
    dst_points.push_back(prev_key_points[match.trainIdx].pt);
  }

  // Homography is much better than fundamental matrix for whatever reason.
  // Could be due to low parallax
  // findHomography(src_points, dst_points, CV_RANSAC, 3, outlier_mask);
  findFundamentalMat(src_points, dst_points, CV_FM_RANSAC, 3., 0.99,
                     outlier_mask);
}

pair<shared_ptr<Frame>, shared_ptr<Frame>>
FeatureExtractor::getFirstTwoFrames() {
  if (frame_count_ >= 2) {
    return pair<shared_ptr<Frame>, shared_ptr<Frame>>(frames[0], frames[1]);
  } else {
    throw NotEnoughFramesException();
  }
}

void FeatureExtractor::PublishLandmarkTracksImage() {
  cv_bridge::CvImage tracks_out_img;
  tracks_out_img.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
  tracks_out_img.header.stamp = ros::Time(frames.back()->timestamp);
  tracks_out_img.header.seq = frames.back()->id;
  cvtColor(frames.back()->image, tracks_out_img.image, CV_GRAY2RGB);
  for (const auto& landmark : landmarks) {
    if (landmark->keypoint_observations.size() > 1 &&
        landmark->keypoint_observations.back()->frame->id > frame_count_ - 5) {
      int obsCount = static_cast<int>(landmark->keypoint_observations.size());
      for (int k = 1; k < obsCount; ++k) {
        line(tracks_out_img.image,
             landmark->keypoint_observations[k]->keypoint.pt,
             landmark->keypoint_observations[k - 1]->keypoint.pt,
             Scalar(255, 0, 0), 1);
      }
      Point point = landmark->keypoint_observations.back()->keypoint.pt;
      if (landmark->keypoint_observations.front()->frame->id <
          frame_count_ - GlobalParams::LandmarkCullingFrameCount()) {
        // Landmark has passed the culling criteria so we draw it in green
        circle(tracks_out_img.image, point, 5, Scalar(0, 255, 0), 2);
      } else {
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

void FeatureExtractor::GetLandmarkMatches(const Mat& descriptors,
                                          const vector<KeyPoint>& keypoints,
                                          vector<DMatch>& matches,
                                          vector<uchar>& outlier_mask) {}

int FeatureExtractor::GetLandmarkCount() { return landmarks.size(); }

int FeatureExtractor::GetFrameCount() { return frames.size(); }

void FeatureExtractor::FindGoodFeaturesToTrackGridded(
    const Mat& img, vector<cv::Point2f>& corners, int cell_count_x,
    int cell_count_y, int max_features_per_cell, double quality_level,
    double min_distance) {
  int cell_w = img.cols / cell_count_x;
  int cell_h = img.rows / cell_count_y;
  for (int cell_x = 0; cell_x < cell_count_x; ++cell_x) {
    for (int cell_y = 0; cell_y < cell_count_y; ++cell_y) {
      cv::Rect mask(cell_x * cell_w, cell_y * cell_h, cell_w, cell_h);
      cv::Mat roi = img(mask);
      vector<cv::Point2f> corners_in_roi;
      goodFeaturesToTrack(roi, corners_in_roi, max_features_per_cell,
                          quality_level, min_distance);
      for (auto& corner : corners_in_roi) {
        corner += cv::Point2f(cell_x * cell_w, cell_y * cell_h);
      }
      corners.insert(corners.begin(), corners_in_roi.begin(),
                     corners_in_roi.end());
    }
  }
}

vector<shared_ptr<Frame>> FeatureExtractor::GetFrames() { return frames; }

vector<shared_ptr<Landmark>> FeatureExtractor::GetLandmarks() {
  return landmarks;
}
void FeatureExtractor::CullLandmarks() {
  if (frame_count_ < GlobalParams::LandmarkCullingFrameCount()) {
    return;
  }
  auto frame = frames[frame_count_ - GlobalParams::LandmarkCullingFrameCount()];
  for (auto& landmark_weak : frame->new_landmarks) {
    auto landmark = landmark_weak.lock();
    if (landmark) {
      if (landmark->keypoint_observations.size() <
          GlobalParams::LandmarkCullingObservationPercentage() *
              GlobalParams::LandmarkCullingFrameCount()) {
        CullLandmark(landmark->id);
      }
    }
  }
}

void FeatureExtractor::CullLandmark(int landmark_id) {
  auto new_end =
      std::remove_if(landmarks.begin(), landmarks.end(),
                     [landmark_id](const shared_ptr<Landmark>& landmark) {
                       return landmark->id == landmark_id;
                     });
  landmarks.erase(new_end, landmarks.end());
}
