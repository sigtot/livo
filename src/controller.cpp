#include "controller.h"
#include "frame.h"
#include "ros_helpers.h"

#include <geometry_msgs/PoseStamped.h>
#include <pose3_stamped.h>

#include <global_params.h>

Controller::Controller(FeatureExtractor& frontend, Smoother& backend, IMUGroundTruthSmoother& imu_ground_truth_smoother,
                       ros::Publisher& path_publisher, ros::Publisher& pose_arr_publisher,
                       ros::Publisher& landmark_publisher)
  : frontend_(frontend)
  , backend_(backend)
  , imu_ground_truth_smoother_(imu_ground_truth_smoother)
  , path_publisher_(path_publisher)
  , pose_arr_publisher_(pose_arr_publisher)
  , landmark_publisher_(landmark_publisher)
{
}

void Controller::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  auto new_frame = frontend_.lkCallback(msg);
  std::cout << "frame " << new_frame->id << std::endl;
  if (frontend_.ReadyForInitialization())
  {
    std::cout << "R_H good. Ready for initialization!" << std::endl;
  }
  /*

  if (!imu_ground_truth_smoother_.IsInitialized() && !frontend_.GetKeyframeTransforms().empty())
  {
    auto transform = frontend_.GetNewestKeyframeTransform();
    std::vector<Pose3Stamped> pose_estimates;
    imu_ground_truth_smoother_.Initialize(transform.frame1->timestamp, transform.frame2->timestamp, pose_estimates);
    PublishPoses(pose_estimates);
  }
  else if (imu_ground_truth_smoother_.IsInitialized())
  {
    if (new_frame->id == frontend_.GetNewestKeyframeTransform().frame2->id)
    {
      std::vector<Pose3Stamped> pose_estimates;
      imu_ground_truth_smoother_.GroundTruthUpdate(new_frame->timestamp, pose_estimates);
      PublishPoses(pose_estimates);
    }
    else
    {
      std::vector<Pose3Stamped> pose_estimates;
      imu_ground_truth_smoother_.IMUPredict(new_frame->timestamp, pose_estimates);
      PublishPoses(pose_estimates);
    }
  }
   */

  if (backend_.GetStatus() != kLandmarksInitialized && frontend_.ReadyForInitialization())
  {
    {
      std::vector<Pose3Stamped> pose_estimates;
      std::map<int, Point3> landmark_estimates;

      auto keyframe_transforms = frontend_.GetKeyframeTransforms();
      auto frames_for_imu_init =
          frontend_.GetFramesForIMUAttitudeInitialization(keyframe_transforms.front().frame1->id);
      if (frames_for_imu_init)
      {
        std::cout << frames_for_imu_init->first->id << " -> " << frames_for_imu_init->second->id << " imu" << std::endl;
      }
      backend_.InitializeLandmarks(keyframe_transforms, frontend_.GetHighParallaxTracks(), frames_for_imu_init,
                                   pose_estimates, landmark_estimates);

      PublishPoses(pose_estimates);
      PublishLandmarks(landmark_estimates, new_frame->timestamp);
    }
    {
      std::vector<Pose3Stamped> pose_estimates;
      std::map<int, Point3> landmark_estimates;

      backend_.InitializeIMU(frontend_.GetKeyframeTransforms(), pose_estimates, landmark_estimates);

      PublishPoses(pose_estimates);
      PublishLandmarks(landmark_estimates, new_frame->timestamp);
    }
  }
  else if (backend_.GetStatus() == kLandmarksInitialized &&
           frontend_.GetNewestKeyframeTransform().frame2->id > backend_.GetLastFrameId())
  {
    std::vector<Pose3Stamped> pose_estimates;
    std::map<int, Point3> landmark_estimates;
    backend_.AddKeyframe(frontend_.GetNewestKeyframeTransform(), frontend_.GetActiveHighParallaxTracks(),
                         pose_estimates, landmark_estimates);
    PublishPoses(pose_estimates);
    PublishLandmarks(landmark_estimates, new_frame->timestamp);
  }
  else if (backend_.GetStatus() == kLandmarksInitialized)
  {
    std::vector<Pose3Stamped> pose_estimates;
    std::map<int, Point3> landmark_estimates;
    backend_.AddFrame(new_frame, frontend_.GetActiveHighParallaxTracks(), pose_estimates, landmark_estimates);
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
