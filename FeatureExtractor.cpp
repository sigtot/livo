#include "FeatureExtractor.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>

FeatureExtractor::FeatureExtractor(const ros::Publisher& matchesPub,
                                   const ros::Publisher& tracksPub, int lag)
    : matchesPub(matchesPub), tracksPub(tracksPub), lag(lag) {}

const int MAX_FEATURES = 200;

const double RESIZE_FACTOR = 0.5;

const int matchHorizon = 5;

shared_ptr<Frame> FeatureExtractor::imageCallback(
    const sensor_msgs::Image::ConstPtr& msg) {
  auto cvPtr = cv_bridge::toCvCopy(
      msg,
      sensor_msgs::image_encodings::
          TYPE_8UC1);  // Makes copy. We can also share to increase performance
  Mat imgResized;
  resize(cvPtr->image, imgResized, Size(), RESIZE_FACTOR, RESIZE_FACTOR,
         INTER_LINEAR);

  Ptr<Feature2D> orb = ORB::create(MAX_FEATURES);
  Ptr<DescriptorMatcher> matcher =
      DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);

  vector<KeyPoint> keyPoints;
  Mat descriptors;

  vector<cv::Point2f> corners;
  goodFeaturesToTrack(imgResized, corners, MAX_FEATURES, 0.01, 7);

  for (auto& corner : corners) {
    keyPoints.emplace_back(corner, 1);
  }

  orb->compute(imgResized, keyPoints, descriptors);
  keyPoints.insert(keyPoints.end(), keyPoints.begin(), keyPoints.end());
  descriptors.push_back(descriptors);

  // Register observations in new frame
  shared_ptr<Frame> newFrame = make_shared<Frame>();
  newFrame->image = imgResized;
  newFrame->id = frameCount++;
  newFrame->timeStamp = msg->header.stamp.toSec();
  for (int i = 0; i < keyPoints.size(); ++i) {
    shared_ptr<KeyPointObservation> observation =
        make_shared<KeyPointObservation>();
    observation->keyPoint = keyPoints[i];
    observation->descriptor =
        descriptors.row(i).clone();  // clone maybe unnecessary
    observation->frame = weak_ptr<Frame>(newFrame);
    newFrame->keyPointObservations.push_back(move(observation));
  }

  // Perform matching and create new landmarks
  if (!frames.empty()) {
    vector<MatchResult> matchResults;
    int lastFrameIdx = static_cast<int>(frames.size()) - 1;
    for (int k = lastFrameIdx; k > max(lastFrameIdx - matchHorizon, 0); --k) {
      MatchResult matchResult;
      getMatches(frames[k], descriptors, keyPoints, matchResult.matches,
                 matchResult.inliers);
      matchResult.frame = frames[k];
      matchResults.push_back(matchResult);
    }

    vector<int> unmatchedIndices;
    vector<MatchInFrame> bestMatches;
    for (int i = 0; i < keyPoints.size(); ++i) {
      MatchInFrame bestMatch;
      bool haveBestMatch = false;
      for (auto& matchResult : matchResults) {
        if (matchResult.inliers[i] && matchResult.matches[i].queryIdx >= 0 &&
            matchResult.matches[i].trainIdx >= 0 &&  // can be -1, idk why
            (!haveBestMatch ||
             matchResult.matches[i].distance < bestMatch.match.distance)) {
          bestMatch = MatchInFrame{.match = matchResult.matches[i],
                                   .frame = matchResult.frame};
          haveBestMatch = true;
        }
      }
      if (haveBestMatch) {
        bestMatches.push_back(bestMatch);
      } else {
        unmatchedIndices.push_back(i);
      }
    }

    // You cannot observe the same landmark more than once in a frame
    map<int, MatchInFrame> bestExistingLandmarkMatches;
    map<int, MatchInFrame> bestNoLandmarkMatches;
    for (auto& match : bestMatches) {
      auto landmark = match.frame->keyPointObservations[match.match.trainIdx]
                          ->landmark.lock();
      if (landmark && !bestExistingLandmarkMatches.count(landmark->id)) {
        bestExistingLandmarkMatches[landmark->id] = match;
      } else if (!bestNoLandmarkMatches.count(match.match.trainIdx)) {
        bestNoLandmarkMatches[match.match.trainIdx] = match;
      }
      // For now we discard unlucky matches. TODO: choose the best match in case
      // of dupes
    }

    for (auto& matchInFrameIt : bestExistingLandmarkMatches) {
      auto matchInFrame = matchInFrameIt.second;
      auto existingLandmark =
          matchInFrame.frame->keyPointObservations[matchInFrame.match.trainIdx]
              ->landmark.lock();
      // Add observation for existing landmark
      // cout << "Add obs to existing landmark: " << "new frame " <<
      // newFrame->id << ", old frame" << matchInFrame.frame->id << " landmark
      // id " << existingLandmark->id << endl;
      newFrame->keyPointObservations[matchInFrame.match.queryIdx]->landmark =
          weak_ptr<Landmark>(existingLandmark);
      existingLandmark->keyPointObservations.push_back(
          newFrame->keyPointObservations[matchInFrame.match.queryIdx]);
    }

    for (auto& matchInFrameIt : bestNoLandmarkMatches) {
      auto matchInFrame = matchInFrameIt.second;
      // Init new landmark
      shared_ptr<Landmark> newLandmark = make_shared<Landmark>(Landmark());
      newLandmark->id = landmarkCount++;
      newLandmark->keyPointObservations.push_back(
          matchInFrame.frame
              ->keyPointObservations[matchInFrame.match.trainIdx]);
      newLandmark->keyPointObservations.push_back(
          newFrame->keyPointObservations[matchInFrame.match.queryIdx]);
      matchInFrame.frame->keyPointObservations[matchInFrame.match.trainIdx]
          ->landmark = weak_ptr<Landmark>(newLandmark);
      newFrame->keyPointObservations[matchInFrame.match.queryIdx]->landmark =
          weak_ptr<Landmark>(newLandmark);

      // cout << "Add new landmark: " << "new frame" << newFrame->id << ", old
      // frame" << matchInFrame.frame->id << " landmark id" << newLandmark->id
      // << endl;

      landmarks.push_back(move(newLandmark));
    }

    // cout << unmatchedIndices.size() << " observations were not matched" <<
    // endl;

    // Publish image of landmark tracks
    cv_bridge::CvImage tracksOutImg(msg->header,
                                    sensor_msgs::image_encodings::TYPE_8UC3);
    cvtColor(imgResized, tracksOutImg.image, CV_GRAY2RGB);
    for (const auto& landMark : landmarks) {
      if (landMark->keyPointObservations.size() > 2 &&
          landMark->keyPointObservations.back()->frame.lock()->id >
              frameCount - 5) {
        int obsCount = static_cast<int>(landMark->keyPointObservations.size());
        for (int k = 1; k < obsCount; ++k) {
          line(tracksOutImg.image,
               landMark->keyPointObservations[k]->keyPoint.pt,
               landMark->keyPointObservations[k - 1]->keyPoint.pt,
               Scalar(0, 255, 0), 1);
        }
        Point point = landMark->keyPointObservations.back()->keyPoint.pt;
        circle(tracksOutImg.image, point, 5, Scalar(0, 0, 255), 1);
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
    tracksPub.publish(tracksOutImg.toImageMsg());
  }

  // Persist new frame
  frames.push_back(newFrame);

  return newFrame;
}

void FeatureExtractor::getMatches(const shared_ptr<Frame>& frame,
                                  const Mat& descriptors,
                                  const vector<KeyPoint>& keyPoints,
                                  vector<DMatch>& matches,
                                  vector<uchar>& outlierMask) {
  Ptr<DescriptorMatcher> matcher =
      DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);
  auto prevKeyPoints = frame->getKeyPoints();
  auto prevDescriptors = frame->getDescriptors();

  matcher->match(descriptors, prevDescriptors, matches);

  vector<Point> srcPoints;
  vector<Point> dstPoints;
  for (auto match : matches) {
    srcPoints.push_back(keyPoints[match.queryIdx].pt);
    dstPoints.push_back(prevKeyPoints[match.trainIdx].pt);
  }

  // Homography is much better than fundamental matrix for whatever reason.
  // Could be due to low parallax
  findHomography(srcPoints, dstPoints, CV_RANSAC, 3, outlierMask);
  // findFundamentalMat(srcPoints, dstPoints, CV_FM_RANSAC, 3., 0.99,
  // outlierMask);
}

pair<shared_ptr<Frame>, shared_ptr<Frame>>
FeatureExtractor::getFirstTwoFrames() {
  if (frameCount >= 2) {
    return pair<shared_ptr<Frame>, shared_ptr<Frame>>(frames[0], frames[1]);
  } else {
    throw NotEnoughFramesException();
  }
}
