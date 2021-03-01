#ifndef ORB_TEST_FEATUREEXTRACTOR_H
#define ORB_TEST_FEATUREEXTRACTOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include "landmark.h"
#include "frame.h"

using namespace std;
using namespace cv;

struct NotEnoughFramesException : public std::exception {
  const char* what() const noexcept override {
    return "Cannot return first two frames when frame count is less than 2";
  }
};

class FeatureExtractor {
 private:
  ros::Publisher matches_pub_;
  ros::Publisher tracks_pub_;
  vector<shared_ptr<Frame>> frames;
  vector<shared_ptr<Landmark>> landmarks;
  int landmark_count_ = 0;
  int frame_count_ = 0;
  int lag;
  const bool debug = true;

  static void getMatches(const shared_ptr<Frame>& frame, const Mat& descriptors,
                         const vector<KeyPoint>& keypoints,
                         vector<DMatch>& matches, vector<uchar>& outlier_mask);

  void GetLandmarkMatches(const Mat& descriptors,
                          const vector<KeyPoint>& keypoints,
                          vector<DMatch>& matches, vector<uchar>& outlier_mask);

 public:
  explicit FeatureExtractor(const ros::Publisher& matches_pub,
                            const ros::Publisher& tracks_pub, int lag);

  shared_ptr<Frame> imageCallback(const sensor_msgs::Image::ConstPtr& msg);

  void PublishLandmarkTracksImage();

  pair<shared_ptr<Frame>, shared_ptr<Frame>> getFirstTwoFrames();

  int GetLandmarkCount();
};

#endif  // ORB_TEST_FEATUREEXTRACTOR_H
