#include "FeatureExtractor.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>

FeatureExtractor::FeatureExtractor(const ros::Publisher &matchesPub, const ros::Publisher &tracksPub, int lag)
        : matchesPub(matchesPub), tracksPub(tracksPub), lag(lag) {}

const int MAX_FEATURES = 500;

// The current grid implementation slows down the code a lot, so I've disabled it by setting GRID_SIZE=1
const int GRID_SIZE = 1;

const double RESIZE_FACTOR = 0.5;

const int matchHorizon = 5;

void FeatureExtractor::imageCallback(const sensor_msgs::Image::ConstPtr &msg) {
    auto cvPtr = cv_bridge::toCvCopy(msg,
                                     sensor_msgs::image_encodings::TYPE_8UC1); // Makes copy. We can also share to increase performance
    Mat imgResized;
    resize(cvPtr->image, imgResized, Size(), RESIZE_FACTOR, RESIZE_FACTOR, INTER_LINEAR);

    int imgHeight = imgResized.rows;
    int imgWidth = imgResized.cols;

    int gridBlockHeightPx = imgHeight / GRID_SIZE;
    int gridBlockWidthPx = imgWidth / GRID_SIZE;


    Ptr<Feature2D> orb = ORB::create(MAX_FEATURES);
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);

    vector<KeyPoint> keyPoints;
    Mat descriptors;

    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; j++) {
            vector<KeyPoint> newKeyPoints;
            Mat newDescriptors;

            Mat mask;
            mask = Mat::zeros(imgResized.rows, imgResized.cols, CV_8U);
            mask(Rect(i * gridBlockWidthPx, j * gridBlockHeightPx, gridBlockWidthPx, gridBlockHeightPx)) = 1;

            orb->detectAndCompute(imgResized, mask, newKeyPoints, newDescriptors);
            keyPoints.insert(keyPoints.end(), newKeyPoints.begin(), newKeyPoints.end());
            descriptors.push_back(newDescriptors);
        }
    }

    // Register observations in new frame
    shared_ptr<Frame> newFrame = make_shared<Frame>();
    newFrame->image = imgResized;
    newFrame->id = frameCount++;
    for (int i = 0; i < keyPoints.size(); ++i) {
        shared_ptr<KeyPointObservation> observation = make_shared<KeyPointObservation>();
        observation->keyPoint = keyPoints[i];
        observation->descriptor = descriptors.row(i).clone(); // clone maybe unnecessary
        observation->frame = weak_ptr<Frame>(newFrame);
        newFrame->keyPointObservations.push_back(move(observation));
    }

    // Perform matching and create new landmarks
    if (!frames.empty()) {
        vector<MatchResult> matchResults;
        int lastFrameIdx = static_cast<int>(frames.size()) - 1;
        for (int k = lastFrameIdx; k > max(lastFrameIdx - matchHorizon, 0); --k) {
            MatchResult matchResult;
            getMatches(frames[k], descriptors, keyPoints, matchResult.matches, matchResult.inliers);
            matchResult.frame = frames[k];
            matchResults.push_back(matchResult);
        }

        vector<int> unmatchedIndices;
        vector<MatchInFrame> bestMatches;
        for (int i = 0; i < keyPoints.size(); ++i) {
            MatchInFrame bestMatch;
            bool haveBestMatch = false;
            for (auto &matchResult : matchResults) {
                if (matchResult.inliers[i] &&
                    matchResult.matches[i].queryIdx > 0 && matchResult.matches[i].trainIdx > 0 && // can be -1, idk why
                    (!haveBestMatch || matchResult.matches[i].distance < bestMatch.match.distance)) {
                    bestMatch = MatchInFrame {.match = matchResult.matches[i], .frame = matchResult.frame};
                    haveBestMatch = true;
                }
            }
            if (haveBestMatch) {
                bestMatches.push_back(bestMatch);
            } else {
                unmatchedIndices.push_back(i);
            }
        }

        for (auto &matchInFrame : bestMatches) {
            auto existingLandmark = matchInFrame.frame->keyPointObservations[matchInFrame.match.trainIdx]->landmark.lock();
            if (existingLandmark) {
                // Add observation for existing landmark
                int existingMatchCount = existingLandmark->keyPointObservations.size();
                cout << "Matched to existing landmark " << existingLandmark->id << "(" << existingMatchCount
                     << " existing matches)" << endl;
                newFrame->keyPointObservations[matchInFrame.match.queryIdx]->landmark = weak_ptr<Landmark>(existingLandmark);
                existingLandmark->keyPointObservations.push_back(newFrame->keyPointObservations[matchInFrame.match.queryIdx]);

            } else {
                // Init new landmark
                shared_ptr<Landmark> newLandmark = make_shared<Landmark>(Landmark());
                newLandmark->id = landmarkCount++;
                newLandmark->keyPointObservations.push_back(matchInFrame.frame->keyPointObservations[matchInFrame.match.trainIdx]);
                newLandmark->keyPointObservations.push_back(newFrame->keyPointObservations[matchInFrame.match.queryIdx]);
                matchInFrame.frame->keyPointObservations[matchInFrame.match.trainIdx]->landmark = weak_ptr<Landmark>(newLandmark);
                newFrame->keyPointObservations[matchInFrame.match.queryIdx]->landmark = weak_ptr<Landmark>(newLandmark);

                cout << "Initialized new landmark " << newLandmark->id << endl;
                landmarks.push_back(move(newLandmark));
            }
        }

        cout << unmatchedIndices.size() << " observations were not matched" << endl;

        // Publish image of landmark tracks
        cv_bridge::CvImage tracksOutImg(msg->header, sensor_msgs::image_encodings::TYPE_8UC3);
        cvtColor(imgResized, tracksOutImg.image, CV_GRAY2RGB);
        for (const auto &landMark : landmarks) {
            if (landMark->keyPointObservations.size() > 3 &&
                landMark->keyPointObservations.back()->frame.lock()->id == frameCount - 1) {
                int obsCount = static_cast<int>(landMark->keyPointObservations.size());
                for (int k = obsCount - 1; k > max(obsCount - lag, 0); --k) {
                    line(tracksOutImg.image, landMark->keyPointObservations[k]->keyPoint.pt,
                         landMark->keyPointObservations[k - 1]->keyPoint.pt, Scalar(0, 255, 0), 2);
                }
            }
        }
        tracksPub.publish(tracksOutImg.toImageMsg());
    }

    // Persist new frame
    frames.push_back(move(newFrame));
}

void FeatureExtractor::getMatches(const shared_ptr<Frame> &frame, const Mat &descriptors, vector<KeyPoint> keyPoints,
                                  vector<DMatch> &matches, vector<uchar> &outlierMask) {
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);
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
    //findFundamentalMat(srcPoints, dstPoints, CV_FM_RANSAC, 3., 0.99, outlierMask);
}
