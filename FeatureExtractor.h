#ifndef ORB_TEST_FEATUREEXTRACTOR_H
#define ORB_TEST_FEATUREEXTRACTOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

using namespace std;
using namespace cv;

struct KeyPointObservation;
struct Landmark;
struct Frame;

struct MatchResult {
  vector<DMatch> matches;
  vector<uchar> inliers;
  shared_ptr<Frame> frame;
};

struct MatchInFrame {
  DMatch match;
  shared_ptr<Frame> frame;
};

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

struct NotEnoughFramesException : public std::exception {
  const char* what() const noexcept override {
    return "Cannot return first two frames when frame count is less than 2";
  }
};

struct Frame {
  vector<KeyPoint> getKeyPoints() const {
    vector<KeyPoint> keyPoints;  // TODO reserve space up front to avoid resizes
    transform(keyPointObservations.begin(), keyPointObservations.end(),
              back_inserter(keyPoints),
              [](const shared_ptr<KeyPointObservation>& o) -> KeyPoint {
                return o->keyPoint;
              });
    return keyPoints;
  }

  Mat getDescriptors() const {
    Mat descriptors;
    for (const auto& obs : keyPointObservations) {
      descriptors.push_back(obs->descriptor);
    }
    return descriptors;
  }

  Mat image;
  vector<shared_ptr<KeyPointObservation>> keyPointObservations;
  int id;
  double timeStamp;
};

class FeatureExtractor {
 private:
  ros::Publisher matchesPub;
  ros::Publisher tracksPub;
  vector<shared_ptr<Frame>> frames;
  vector<shared_ptr<Landmark>> landmarks;
  int landmarkCount = 0;
  int frameCount = 0;
  int lag;
  const bool debug = true;

  static void getMatches(const shared_ptr<Frame>& frame, const Mat& descriptors,
                         vector<KeyPoint> keyPoints, vector<DMatch>& matches,
                         vector<uchar>& outlierMask);

 public:
  explicit FeatureExtractor(const ros::Publisher& matchesPub,
                            const ros::Publisher& tracksPub, int lag);

  shared_ptr<Frame> imageCallback(const sensor_msgs::Image::ConstPtr& msg);

  pair<shared_ptr<Frame>, shared_ptr<Frame>> getFirstTwoFrames();
};

#endif  // ORB_TEST_FEATUREEXTRACTOR_H
