#include "controller.h"
#include "frame.h"
#include "ros_helpers.h"

#include <geometry_msgs/PoseStamped.h>
#include <pose3_stamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <global_params.h>

Controller::Controller(FeatureExtractor& frontend, LidarFrameManager& lidar_frame_manager, Smoother& backend,
                       NewSmoother& new_backend, IMUGroundTruthSmoother& imu_ground_truth_smoother,
                       ros::Publisher& path_publisher, ros::Publisher& pose_arr_publisher,
                       ros::Publisher& landmark_publisher)
  : frontend_(frontend)
  , lidar_frame_manager_(lidar_frame_manager)
  , backend_(backend)
  , new_backend_(new_backend)
  , imu_ground_truth_smoother_(imu_ground_truth_smoother)
  , path_publisher_(path_publisher)
  , pose_arr_publisher_(pose_arr_publisher)
  , landmark_publisher_(landmark_publisher)
{
}

void Controller::LidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
  lidar_frame_manager_.LidarCallback(cloud, msg->header.stamp.toSec());
}

void Controller::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  auto new_frame = frontend_.lkCallback(msg);
  std::cout << "frame " << new_frame->id << std::endl;

  if (new_backend_.IsInitialized())
  {
    if (new_frame->is_keyframe)
    {
      new_backend_.AddKeyframe(new_frame);
    }
  }

  if (!new_backend_.IsInitialized() && new_frame->HasDepth())
  {
    if (new_frame->stationary)
    {
      if (frontend_.GetFramesForIMUAttitudeInitialization(new_frame->id))
      {
        auto imu_init_frames = frontend_.GetFramesForIMUAttitudeInitialization(new_frame->id);
        // We wait until we have > 1.0 seconds of imu integration before using it for gravity alignment
        if (imu_init_frames && std::abs(imu_init_frames->first->timestamp - imu_init_frames->second->timestamp) > 1.0)
        {
          std::cout << "Initializing with gravity alignment" << std::endl;
          new_backend_.Initialize(new_frame, std::pair<double, double>{ imu_init_frames->first->timestamp,
                                                                        imu_init_frames->second->timestamp });
        }
      }
    }
    else
    {
      // Not stationary, no point in waiting for more stationary imu messages. Initialize immediately.
      std::cout << "Initializing without gravity alignment" << std::endl;
      new_backend_.Initialize(new_frame);
    }
  }

  std::map<int, Pose3Stamped> pose_estimates;
  new_backend_.GetPoses(pose_estimates);
  std::vector<Pose3Stamped> pose_estimates_vector;
  for (const auto & pose_estimate : pose_estimates)
  {
    pose_estimates_vector.push_back(pose_estimate.second);
  }
  PublishPoses(pose_estimates_vector);

  std::map<int, LandmarkResult> landmark_estimates;
  new_backend_.GetLandmarks(landmark_estimates);
  PublishLandmarks(landmark_estimates, new_frame->timestamp);

  /*

  if (imu_ground_truth_smoother_.IsInitialized())
  {
    if (new_frame->is_keyframe)
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
  else
  {
    std::vector<Pose3Stamped> pose_estimates;
    imu_ground_truth_smoother_.Initialize(new_frame->timestamp, pose_estimates);
    PublishPoses(pose_estimates);
  }
   */

  /*
  if (backend_.GetStatus() == kUninitialized && frontend_.ReadyForInitialization())
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
  else if (backend_.GetStatus() != kUninitialized &&
           frontend_.GetNewestKeyframeTransform().frame2->id > backend_.GetLastFrameId())
  {
    std::vector<Pose3Stamped> pose_estimates;
    std::map<int, Point3> landmark_estimates;
    backend_.AddKeyframe(frontend_.GetNewestKeyframeTransform(), frontend_.GetActiveHighParallaxTracks(),
                         pose_estimates, landmark_estimates);
    PublishPoses(pose_estimates);
    PublishLandmarks(landmark_estimates, new_frame->timestamp);
  }
  else if (backend_.GetStatus() == kNominal && false)
  {
    std::vector<Pose3Stamped> pose_estimates;
    std::map<int, Point3> landmark_estimates;
    backend_.AddFrame(new_frame, frontend_.GetActiveHighParallaxTracks(), pose_estimates, landmark_estimates);
    PublishPoses(pose_estimates);
    PublishLandmarks(landmark_estimates, new_frame->timestamp);
  }
   */
}

void Controller::PublishPoses(const std::vector<Pose3Stamped>& poses)
{
  ros_helpers::PublishPoses(poses, path_publisher_, pose_arr_publisher_);
}

void Controller::PublishLandmarks(const std::map<int, LandmarkResult>& landmarks, double timestamp)
{
  ros_helpers::PublishLandmarks(landmarks, timestamp, landmark_publisher_);
}
