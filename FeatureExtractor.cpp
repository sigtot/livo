#include <vector>
#include "FeatureExtractor.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>


FeatureExtractor::FeatureExtractor(const ros::Publisher &pub) : pub(pub) {}

using namespace std;
using namespace cv;

const int MAX_FEATURES = 500;

void FeatureExtractor::imageCallback(const sensor_msgs::Image::ConstPtr &msg) {
    auto cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1); // Makes copy. We can also share to increase performance

    Ptr<Feature2D> orb = ORB::create(MAX_FEATURES);
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);

    vector<KeyPoint> keyPoints;
    Mat descriptors;

    orb->detectAndCompute(cvPtr->image, Mat(), keyPoints, descriptors);

    cv_bridge::CvImage outImg(msg->header, sensor_msgs::image_encodings::TYPE_8UC3);
    drawKeypoints(cvPtr->image, keyPoints, outImg.image);

    pub.publish(outImg.toImageMsg());
}
