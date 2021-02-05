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

    cv_bridge::CvImage outImg(msg->header, sensor_msgs::image_encodings::TYPE_8UC3);
    drawKeypoints(cvPtr->image, keyPoints, outImg.image);

    if (!frames.empty()) {
        auto prevKeyPoints = frames.back().getKeyPoints();
        cout << "type" << descriptors.type() << endl;
        auto prevDescriptors = frames.back().getDescriptors();
        int i = 0;
        for (const auto &kp : prevKeyPoints) {
            cout << "(" << kp.pt.x << "," << kp.pt.y << ") " << endl;
            cout << prevDescriptors.row(i) << endl;
            cout << descriptors.row(i) << endl;
            i++;
        }
    }

    Frame newFrame;
    int i = 0;
    for (auto &kp : keyPoints) {
        Landmark landmark; // ONLY DO THIS ON NEW LANDMARK INITIALIZATION, NOT EVERY TIME
        cout << i << " " << descriptors.row(i).clone() << endl;
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

    pub.publish(outImg.toImageMsg());
}
