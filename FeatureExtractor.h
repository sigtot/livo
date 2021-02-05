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
    Landmark *landmark;
    Frame *frame;
};

struct Landmark {
    vector<KeyPointObservation*> keyPointObservations;
};

struct Frame {
    vector<KeyPoint> getKeyPoints() const {
        vector<KeyPoint> keyPoints; // TODO reserve space up front to avoid resizes
        transform(keyPointObservations.begin(), keyPointObservations.end(), back_inserter(keyPoints),
                  [](const KeyPointObservation& o) -> KeyPoint { return o.keyPoint; });
        return keyPoints;
    }

    Mat getDescriptors() const {
        Mat descriptors;
        for (const auto& obs : keyPointObservations) {
            descriptors.push_back(obs.descriptor);
        }
        return descriptors;
    }

    Mat image;
    vector<KeyPointObservation> keyPointObservations;
    int seq{};
};

class FeatureExtractor {
private:
    ros::Publisher pub;
    vector<Frame> frames;
public:
    explicit FeatureExtractor(const ros::Publisher &pub);

    void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
};


#endif //ORB_TEST_FEATUREEXTRACTOR_H
