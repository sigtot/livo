#include "FeatureExtractor.h"

#include <cv_bridge/cv_bridge.h>

FeatureExtractor::FeatureExtractor(const ros::Publisher &pub) : pub(pub) {}

const int MAX_FEATURES = 500;

void FeatureExtractor::imageCallback(const sensor_msgs::Image::ConstPtr &msg) {
    auto cvPtr = cv_bridge::toCvCopy(msg,
                                     sensor_msgs::image_encodings::TYPE_8UC1); // Makes copy. We can also share to increase performance

    Ptr<Feature2D> orb = ORB::create(MAX_FEATURES);
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);

    vector<KeyPoint> keyPoints;
    Mat descriptors;

    orb->detectAndCompute(cvPtr->image, Mat(), keyPoints, descriptors);

    if (!frames.empty()) {
        auto prevFrame = frames.back();
        auto prevKeyPoints = prevFrame.getKeyPoints();
        auto prevDescriptors = prevFrame.getDescriptors();

        vector<DMatch> matches;
        matcher->match(prevDescriptors, descriptors, matches);

        cout << matches.size() << endl;
        for (auto match : matches) {
            cout << match.distance << ", ";
        }
        cout << endl;

        cv_bridge::CvImage outImg(msg->header, sensor_msgs::image_encodings::TYPE_8UC3);
        drawMatches(prevFrame.image, prevKeyPoints, cvPtr->image, keyPoints, matches, outImg.image);

        pub.publish(outImg.toImageMsg());
    }

    Frame newFrame;
    newFrame.image = cvPtr->image;
    int i = 0;
    for (auto &kp : keyPoints) {
        Landmark landmark; // ONLY DO THIS ON NEW LANDMARK INITIALIZATION, NOT EVERY TIME
        KeyPointObservation observation{
                .keyPoint = kp,
                .descriptor = descriptors.row(i).clone(),
                .landmark = &landmark,
                .frame = &newFrame};
        landmark.keyPointObservations.push_back(&observation);
        newFrame.keyPointObservations.push_back(observation);
        i++;
    }
    frames.push_back(newFrame);
}
