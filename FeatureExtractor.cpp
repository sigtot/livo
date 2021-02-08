#include "FeatureExtractor.h"

#include <cv_bridge/cv_bridge.h>

FeatureExtractor::FeatureExtractor(const ros::Publisher &pub) : pub(pub) {}

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
    int i = 0;
    for (const auto& kp : keyPoints) {
        shared_ptr<KeyPointObservation> observation = make_shared<KeyPointObservation>();
        observation->keyPoint = kp;
        observation->descriptor = descriptors.row(i).clone(); // clone maybe unnecessary
        observation->frame = weak_ptr<Frame>(newFrame);
        newFrame->keyPointObservations.push_back(move(observation));
        i++;
    }

    // Perform matching and create new landmarks
    if (!frames.empty()) {
        auto prevFrame = frames.back();

        auto prevKeyPoints = prevFrame->getKeyPoints();
        auto prevDescriptors = prevFrame->getDescriptors();

        vector<DMatch> matches;
        matcher->match(prevDescriptors, descriptors, matches);

        for (auto match : matches) {
            // TODO if(match.distance < xxx)
            auto existingLandmark = prevFrame->keyPointObservations[match.queryIdx]->landmark.lock();
            if (existingLandmark) {
                int existingMatchCount = existingLandmark->keyPointObservations.size();
                cout << "Matched to existing landmark " << existingLandmark->id << "(" << existingMatchCount << " existing matches)" << endl;
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
        }

        // TODO write a real header
        cv_bridge::CvImage outImg(msg->header, sensor_msgs::image_encodings::TYPE_8UC3);
        drawMatches(prevFrame->image, prevKeyPoints, imgResized, keyPoints, matches, outImg.image);

        pub.publish(outImg.toImageMsg());
    }

    // Persist new frame
    frames.push_back(move(newFrame));
}
