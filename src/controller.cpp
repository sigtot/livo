#include "controller.h"
#include "frame.h"
#include <geometry_msgs/PoseStamped.h>

Controller::Controller(FeatureExtractor& frontend,
                       ros::Publisher& posePublisher,
                       ros::Publisher& landmarkPublisher)
    : frontend(frontend),
      pose_publisher_(posePublisher),
      landmark_publisher_(landmarkPublisher) {}

void Controller::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
  shared_ptr<Frame> new_frame = frontend.imageCallback(msg);
  frontend.PublishLandmarkTracksImage();
  std::cout << "Landmark count: " << frontend.GetLandmarkCount() << std::endl;

  if (frontend.GetFrameCount() > 100) {
    auto frame_0 = frontend.GetFrames()[0];
    auto frame_50 = frontend.GetFrames()[50];
    std::cout << "frame " << frame_0->id << ": " << std::setprecision(20)
              << frame_0->timestamp << std::endl;
    std::cout << "frame " << frame_50->id << ": " << std::setprecision(20)
              << frame_50->timestamp << std::endl;
    Smoother::SmoothBatch(frontend.GetFrames(), frontend.GetLandmarks());
    exit(0);
  }
}
