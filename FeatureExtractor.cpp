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

    if (!frames.empty()) {
        auto prevFrame = frames.back();

        auto prevKeyPoints = prevFrame->getKeyPoints();
        auto prevDescriptors = prevFrame->getDescriptors();

        vector<DMatch> matches;
        matcher->match(prevDescriptors, descriptors, matches);

        cout << matches.size() << endl;
        for (auto match : matches) {
            cout << match.distance << ", ";
        }
        cout << endl;

        // TODO write a real header
        cv_bridge::CvImage outImg(msg->header, sensor_msgs::image_encodings::TYPE_8UC3);
        drawMatches(prevFrame->image, prevKeyPoints, imgResized, keyPoints, matches, outImg.image);

        pub.publish(outImg.toImageMsg());
    }

    shared_ptr<Frame> newFrame = make_shared<Frame>();
    newFrame->image = imgResized;
    int i = 0;
    for (const auto& kp : keyPoints) {
        shared_ptr<KeyPointObservation> observation = make_shared<KeyPointObservation>();
        observation->keyPoint = kp;
        observation->descriptor = descriptors.row(i).clone(); // clone maybe unnecessary
        observation->frame = weak_ptr<Frame>(newFrame);

        shared_ptr<Landmark> landmark = make_shared<Landmark>();
        landmark->id = i;

        observation->landmark = weak_ptr<Landmark>(landmark);
        landmark->keyPointObservations.push_back(observation); // Makes copy of the shared_ptr: shared ownership
        landmarks.push_back(move(landmark));

        newFrame->keyPointObservations.push_back(move(observation)); // Move the initial shared_ptr

        i++;
    }
    frames.push_back(move(newFrame));
}
