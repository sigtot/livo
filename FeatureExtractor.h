#ifndef ORB_TEST_FEATUREEXTRACTOR_H
#define ORB_TEST_FEATUREEXTRACTOR_H

#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

struct KeyPointObservation;
struct Landmark;
struct Frame;

struct KeyPointObservation {
    KeyPoint keyPoint;
    Mat descriptor;
    weak_ptr<Landmark> landmark;
    weak_ptr<Frame> frame;
};

struct Landmark {
    vector<shared_ptr<KeyPointObservation>> keyPointObservations;
    int id;
};

struct Frame {
    vector<KeyPoint> getKeyPoints() const {
        vector<KeyPoint> keyPoints; // TODO reserve space up front to avoid resizes
        transform(keyPointObservations.begin(), keyPointObservations.end(), back_inserter(keyPoints),
                  [](const shared_ptr<KeyPointObservation> &o) -> KeyPoint { return o->keyPoint; });
        return keyPoints;
    }

    Mat getDescriptors() const {
        Mat descriptors;
        for (const auto &obs : keyPointObservations) {
            descriptors.push_back(obs->descriptor);
        }
        return descriptors;
    }

    Mat image;
    vector<shared_ptr<KeyPointObservation>> keyPointObservations;
    int id;
};

class FeatureExtractor {
private:
    ros::Publisher pub;
    vector<shared_ptr<Frame>> frames;
    vector<shared_ptr<Landmark>> landmarks;
    int landmarkCount = 0;
    int frameCount = 0;
    int lag;
public:
    explicit FeatureExtractor(const ros::Publisher &pub, int lag);

    void imageCallback(const sensor_msgs::Image::ConstPtr &msg);

    static void getMatches(const shared_ptr<Frame> &frame, const Mat &descriptors, vector<KeyPoint> keyPoints,
                           vector<DMatch> &matches, vector<char> &outlierMask);
};


#endif //ORB_TEST_FEATUREEXTRACTOR_H
