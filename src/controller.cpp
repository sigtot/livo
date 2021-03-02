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
    std::vector<Pose3> pose_estimates;
    std::vector<Point3> landmark_estimates;
    Smoother::SmoothBatch(frontend.GetFrames(), frontend.GetLandmarks(),
                          pose_estimates, landmark_estimates);
    std::cout << "poses:" << std::endl;
    for (auto& pose : pose_estimates) {
      std::cout << "[" << pose.point.x << ", " << pose.point.y << ", "
                << pose.point.z << "]" << std::endl;
    }

    std::cout << "landmarks:" << std::endl;
    for (auto& landmark : landmark_estimates) {
      std::cout << "[" << landmark.x << ", " << landmark.y << ", " << landmark.z
                << "]" << std::endl;
    }
    exit(0);
  }
}
