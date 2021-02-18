#include "FeatureExtractor.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>

FeatureExtractor::FeatureExtractor(const ros::Publisher &pub, int lag) : pub(pub), lag(lag) {}

const int MAX_FEATURES = 500;

// The current grid implementation slows down the code a lot, so I've disabled it by setting GRID_SIZE=1
const int GRID_SIZE = 1;

const double RESIZE_FACTOR = 0.5;

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
        auto prevFrame = frames.back();
        vector<DMatch> matches;
        vector<char> outlierMask;
        getMatches(frames.back(), descriptors, keyPoints, matches, outlierMask);

        for (int i = 0; i < matches.size(); ++i) {
            auto match = matches[i];
            char isInlier = outlierMask[i];
            if (isInlier) {
                auto existingLandmark = prevFrame->keyPointObservations[match.queryIdx]->landmark.lock();
                if (existingLandmark) {
                    int existingMatchCount = existingLandmark->keyPointObservations.size();
                    cout << "Matched to existing landmark " << existingLandmark->id << "(" << existingMatchCount
                         << " existing matches)" << endl;
                    newFrame->keyPointObservations[match.trainIdx]->landmark = weak_ptr<Landmark>(existingLandmark);
                    existingLandmark->keyPointObservations.push_back(newFrame->keyPointObservations[match.trainIdx]);
                } else {
                    shared_ptr<Landmark> newLandmark = make_shared<Landmark>(Landmark());
                    newLandmark->id = landmarkCount++;
                    newLandmark->keyPointObservations.push_back(prevFrame->keyPointObservations[match.queryIdx]);
                    newLandmark->keyPointObservations.push_back(newFrame->keyPointObservations[match.trainIdx]);
                    prevFrame->keyPointObservations[match.queryIdx]->landmark = weak_ptr<Landmark>(newLandmark);
                    newFrame->keyPointObservations[match.trainIdx]->landmark = weak_ptr<Landmark>(newLandmark);

                    cout << "Initialized new landmark " << newLandmark->id << endl;
                    landmarks.push_back(move(newLandmark));
                }
            } else {
                cout << "Discarding outlier match " << i << endl;
            }
        }

        auto prevKeyPoints = prevFrame->getKeyPoints();
        // TODO write a real header
        cv_bridge::CvImage outImg(msg->header, sensor_msgs::image_encodings::TYPE_8UC3);
        drawMatches(prevFrame->image, prevKeyPoints, imgResized, keyPoints, matches, outImg.image, Scalar::all(-1),
                    Scalar::all(-1), outlierMask);

        pub.publish(outImg.toImageMsg());
    }

    // Persist new frame
    frames.push_back(move(newFrame));
}

void FeatureExtractor::getMatches(const shared_ptr<Frame> &frame, const Mat &descriptors, vector<KeyPoint> keyPoints,
                                  vector<DMatch> &matches, vector<char> &outlierMask) {
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);
    auto prevKeyPoints = frame->getKeyPoints();
    auto prevDescriptors = frame->getDescriptors();

    matcher->match(prevDescriptors, descriptors, matches);

    vector<Point> srcPoints;
    vector<Point> dstPoints;
    for (auto match : matches) {
        srcPoints.push_back(prevKeyPoints[match.queryIdx].pt);
        dstPoints.push_back(keyPoints[match.trainIdx].pt);
    }

    // Use findHomography with RANSAC to create a outlier mask
    findHomography(srcPoints, dstPoints, CV_RANSAC, 3, outlierMask);
}
