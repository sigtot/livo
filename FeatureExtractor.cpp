#include "FeatureExtractor.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>

#include "GlobalParams.h"
#include "match_result.h"
#include "match_in_frame.h"

FeatureExtractor::FeatureExtractor(const ros::Publisher& matches_pub,
                                   const ros::Publisher& tracks_pub, int lag)
    : matches_pub_(matches_pub), tracks_pub_(tracks_pub), lag(lag) {}

const double RESIZE_FACTOR = 0.5;

const int MATCH_HORIZON = 5;

shared_ptr<Frame> FeatureExtractor::imageCallback(
    const sensor_msgs::Image::ConstPtr& msg) {
  auto cvPtr = cv_bridge::toCvCopy(
      msg,
      sensor_msgs::image_encodings::
          TYPE_8UC1);  // Makes copy. We can also share to increase performance
  Mat img_resized;
  resize(cvPtr->image, img_resized, Size(), RESIZE_FACTOR, RESIZE_FACTOR,
         INTER_LINEAR);

  Ptr<Feature2D> orb = ORB::create(GlobalParams::MaxFeatures());

  vector<KeyPoint> keypoints;
  Mat descriptors;

  vector<cv::Point2f> corners;
  goodFeaturesToTrack(img_resized, corners, GlobalParams::MaxFeatures(), 0.01,
                      7);

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
                                         weak_ptr<Frame>(new_frame));
    new_frame->keypoint_observations.push_back(move(observation));
  }

  // Perform matching and create new landmarks
  if (!frames.empty()) {
    vector<MatchResult> match_results;
    int last_frame_idx = static_cast<int>(frames.size()) - 1;
    for (int k = last_frame_idx; k > max(last_frame_idx - MATCH_HORIZON, 0);
         --k) {
      MatchResult match_result;
      getMatches(frames[k], descriptors, keypoints, match_result.matches,
                 match_result.inliers);
      match_result.frame = frames[k];
      match_results.push_back(match_result);
    }

    vector<int> unmatched_indices;
    vector<MatchInFrame> best_matches;
    for (int i = 0; i < keypoints.size(); ++i) {
      MatchInFrame best_match;
      bool have_best_match = false;
      for (auto& match_result : match_results) {
        if (match_result.inliers[i] && match_result.matches[i].queryIdx >= 0 &&
            match_result.matches[i].trainIdx >= 0 &&  // can be -1, idk why
            (!have_best_match ||
             match_result.matches[i].distance < best_match.match.distance)) {
          best_match = MatchInFrame{.match = match_result.matches[i],
                                    .frame = match_result.frame};
          have_best_match = true;
        }
      }
      if (have_best_match) {
        best_matches.push_back(best_match);
      } else {
        unmatched_indices.push_back(i);
      }
    }

    // You cannot observe the same landmark more than once in a frame
    map<int, MatchInFrame> best_existing_landmark_matches;
    map<int, MatchInFrame> best_no_landmark_matches;
    for (auto& match : best_matches) {
      auto landmark = match.frame->keypoint_observations[match.match.trainIdx]
                          ->landmark.lock();
      if (landmark && !best_existing_landmark_matches.count(landmark->id)) {
        best_existing_landmark_matches[landmark->id] = match;
      } else if (!best_no_landmark_matches.count(match.match.trainIdx)) {
        best_no_landmark_matches[match.match.trainIdx] = match;
      }
      // For now we discard unlucky matches.
      // TODO: choose the best match in case of dupes
    }

    for (auto& match_in_frame_it : best_existing_landmark_matches) {
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

    for (auto& match_in_frame_it : best_no_landmark_matches) {
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
  findHomography(src_points, dst_points, CV_RANSAC, 3, outlier_mask);
  // findFundamentalMat(srcPoints, dstPoints, CV_FM_RANSAC, 3., 0.99,
  // outlierMask);
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
    if (landmark->keypoint_observations.size() > 2 &&
        landmark->keypoint_observations.back()->frame.lock()->id >
            frame_count_ - 5) {
      int obsCount = static_cast<int>(landmark->keypoint_observations.size());
      for (int k = 1; k < obsCount; ++k) {
        line(tracks_out_img.image,
             landmark->keypoint_observations[k]->keypoint.pt,
             landmark->keypoint_observations[k - 1]->keypoint.pt,
             Scalar(0, 255, 0), 1);
      }
      Point point = landmark->keypoint_observations.back()->keypoint.pt;
      circle(tracks_out_img.image, point, 5, Scalar(0, 0, 255), 1);
      /*
      putText(tracksOutImg.image,
              to_string(landMark->id), //text
              point + Point(5, 5),
              FONT_HERSHEY_DUPLEX,
              0.3,
              CV_RGB(255, 0, 0), //font color
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
