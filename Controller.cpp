#include "Controller.h"
#include <geometry_msgs/PoseStamped.h>

using namespace std;

Controller::Controller(FeatureExtractor& frontend,
                       ros::Publisher& posePublisher,
                       ros::Publisher& landmarkPublisher)
    : frontend(frontend),
      pose_publisher_(posePublisher),
      landmark_publisher_(landmarkPublisher) {}

void Controller::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
  shared_ptr<Frame> newFrame = frontend.imageCallback(msg);

  // Eventually, we will put backend updates here
}
