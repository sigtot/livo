#include "controller.h"
#include "frame.h"
#include "ros_helpers.h"

#include <geometry_msgs/PoseStamped.h>
#include <pose3_stamped.h>

#include <global_params.h>

Controller::Controller(FeatureExtractor& frontend, Smoother& backend, ros::Publisher& path_publisher, ros::Publisher& pose_arr_publisher,
                       ros::Publisher& landmark_publisher)
  : frontend(frontend), backend(backend), path_publisher_(path_publisher), pose_arr_publisher_(pose_arr_publisher), landmark_publisher_(landmark_publisher)
{
}

void Controller::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  auto new_frame = frontend.lkCallback(msg);
  std::cout << "frame " << new_frame->id << std::endl;
  if (frontend.ReadyForInitialization())
  {
    std::cout << "R_H good. Ready for initialization!" << std::endl;
  }

  if (backend.GetStatus() != kLandmarksInitialized && frontend.ReadyForInitialization())
  {
    {
      std::vector<Pose3Stamped> pose_estimates;
      std::map<int, Point3> landmark_estimates;

      auto keyframe_transforms = frontend.GetKeyframeTransforms();
      auto frames_for_imu_init = frontend.GetFramesForIMUAttitudeInitialization(keyframe_transforms.front().frame1->id);
      if (frames_for_imu_init)
      {
        std::cout << frames_for_imu_init->first->id << " -> " << frames_for_imu_init->second->id << " imu" << std::endl;
      }
      backend.InitializeLandmarks(keyframe_transforms, frontend.GetHighParallaxTracks(), frames_for_imu_init,
                                  pose_estimates, landmark_estimates);

      PublishPoses(pose_estimates);
      PublishLandmarks(landmark_estimates, new_frame->timestamp);
    }
    {
      std::vector<Pose3Stamped> pose_estimates;
      std::map<int, Point3> landmark_estimates;

      backend.InitializeIMU(frontend.GetKeyframeTransforms(), pose_estimates, landmark_estimates);

      PublishPoses(pose_estimates);
      PublishLandmarks(landmark_estimates, new_frame->timestamp);
    }
  }
  else if (backend.GetStatus() == kLandmarksInitialized &&
           frontend.GetNewestKeyframeTransform().frame2->id > backend.GetLastFrameId())
  {
    std::vector<Pose3Stamped> pose_estimates;
    std::map<int, Point3> landmark_estimates;
    backend.AddKeyframe(frontend.GetNewestKeyframeTransform(), frontend.GetActiveHighParallaxTracks(), pose_estimates,
                        landmark_estimates);
    PublishPoses(pose_estimates);
    PublishLandmarks(landmark_estimates, new_frame->timestamp);
  }
  else if (backend.GetStatus() == kLandmarksInitialized){
    std::vector<Pose3Stamped> pose_estimates;
    std::map<int, Point3> landmark_estimates;
    backend.AddFrame(new_frame, frontend.GetActiveHighParallaxTracks(), pose_estimates, landmark_estimates);
    PublishPoses(pose_estimates);
    PublishLandmarks(landmark_estimates, new_frame->timestamp);
  }
}

void Controller::PublishPoses(const std::vector<Pose3Stamped>& poses)
{
  ros_helpers::PublishPoses(poses, path_publisher_, pose_arr_publisher_);
}

void Controller::PublishLandmarks(const std::map<int, Point3>& landmarks, double timestamp)
{
  ros_helpers::PublishLandmarks(landmarks, timestamp, landmark_publisher_);
}
