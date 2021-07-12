#include "controller.h"
#include "frame.h"
#include "ros_helpers.h"
#include "debug_value_publisher.h"

#include <geometry_msgs/PoseStamped.h>
#include <pose3_stamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_broadcaster.h>

#include <global_params.h>
#include <chrono>
#include <utility>

Controller::Controller(std::shared_ptr<FeatureExtractor> frontend, LidarFrameManager& lidar_frame_manager, NewSmoother& new_backend,
                       std::shared_ptr<BetweenTransformProvider> between_transform_provider,
                       std::shared_ptr<TimeOffsetProvider> lidar_time_offset_provider, ros::Publisher& path_publisher,
                       ros::Publisher& pose_arr_publisher, ros::Publisher& landmark_publisher)
  : frontend_(std::move(frontend))
  , lidar_frame_manager_(lidar_frame_manager)
  , new_backend_(new_backend)
  , between_transform_provider_(std::move(between_transform_provider))
  , lidar_time_offset_provider_(std::move(lidar_time_offset_provider))
  , path_publisher_(path_publisher)
  , pose_arr_publisher_(pose_arr_publisher)
  , landmark_publisher_(landmark_publisher)
  , full_trajectory_manager_(FullTrajectoryManager())
{
  backend_thread_ = std::thread(&Controller::BackendSpinner, this);
}

Controller::~Controller()
{
  back_cv_.notify_all();
  backend_thread_.join();
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
  static int i = 0;
  auto time_before = std::chrono::system_clock::now();
  auto new_frame = frontend_->lkCallback(msg);
  auto time_after = std::chrono::system_clock::now();

  auto micros = std::chrono::duration_cast<std::chrono::microseconds>(time_after - time_before);
  double millis = static_cast<double>(micros.count()) / 1000.;
  DebugValuePublisher::PublishFrontendDuration(millis);
  std::cout << "Frontend frame " << new_frame->id << " (took: " << millis << " ms)" << std::endl;

  if (i++ % GlobalParams::FrameInterval() != 0)
  {
    return;  // Return before feeding into backend. I.e. frontend has twice the rate of the backend
  }

  SetLatestFrameForBackend(new_frame);  // Will block if backend is still processing the previous frame.
}

void Controller::PublishPoses(const std::vector<Pose3Stamped>& poses)
{
  ros_helpers::PublishPoseArray(poses, pose_arr_publisher_);
}

void Controller::UpdateAndPublishFullTrajectory(const std::map<int, Pose3Stamped>& new_poses)
{
  full_trajectory_manager_.UpdatePoses(new_poses);
  ros_helpers::PublishPath(full_trajectory_manager_.GetTrajectoryAsVector(), path_publisher_);
}

void Controller::PublishLatestLidarTransform(const Pose3Stamped& pose_stamped)
{
  static tf2_ros::TransformBroadcaster br;
  ros_helpers::PublishTransform(pose_stamped, "world", "os1_lidar", br);
}

void Controller::PublishLandmarks(const std::map<int, LandmarkResult>& landmarks, double timestamp)
{
  ros_helpers::PublishLandmarks(landmarks, timestamp, landmark_publisher_);
}

void Controller::SetLatestFrameForBackend(std::shared_ptr<Frame> frame)
{
  std::lock_guard<std::mutex> lock(back_cv_m_);  // Blocks execution until backend is done with its latest frame
  latest_frame_ = std::move(frame);
  back_cv_.notify_one();  // Notifies backend of the new frame to process
}

void Controller::BackendSpinner()
{
  std::unique_lock<std::mutex> lk(back_cv_m_);
  std::cout << "Backend ready and spinning" << std::endl;
  while (!ros::isShuttingDown())
  {
    // We wait on the condition variable for a new latest frame to be provided.
    // When it is, the back_cv_m_ lock will be acquired atomically which blocks the frontend from processing
    // any further than the n+1th frame.
    back_cv_.wait(lk, [this] { return latest_frame_.is_initialized() || ros::isShuttingDown(); });

    ProcessWithBackend(*latest_frame_);
    latest_frame_ = boost::none;  // Set latest frame to none since we have processed it
  }
}

void Controller::ProcessWithBackend(const shared_ptr<Frame>& frame)
{
  auto time_before = std::chrono::system_clock::now();
  if (new_backend_.IsInitialized())
  {
    new_backend_.AddKeyframe(frame, frame->is_keyframe);
  }

  bool must_wait_for_transform = GlobalParams::LoamBetweenFactorsEnabled() &&
                                 !between_transform_provider_->CanTransform(
                                     frame->timestamp - lidar_time_offset_provider_->GetOffset(frame->timestamp));

  if (!new_backend_.IsInitialized() && frame->HasDepth() && !must_wait_for_transform)
  {
    if (GlobalParams::DoInitialGravityAlignment() && frame->stationary)
    {
      if (frontend_->GetFramesForIMUAttitudeInitialization(frame->id))
      {
        auto imu_init_frames = frontend_->GetFramesForIMUAttitudeInitialization(frame->id);
        // We wait until we have > 1.0 seconds of imu integration before using it for gravity alignment
        if (imu_init_frames && std::abs(imu_init_frames->first->timestamp - imu_init_frames->second->timestamp) > 1.0)
        {
          std::cout << "Initializing with gravity alignment" << std::endl;
          new_backend_.Initialize(frame, std::pair<double, double>{ imu_init_frames->first->timestamp,
                                                                    imu_init_frames->second->timestamp });
        }
      }
    }
    else
    {
      // Not stationary, or not aligning, no point in waiting for more stationary imu messages. Initialize immediately.
      std::cout << "Initializing without gravity alignment" << std::endl;
      new_backend_.Initialize(frame);
    }
  }
  auto time_after_backend = std::chrono::system_clock::now();

  std::map<int, Pose3Stamped> pose_estimates;
  new_backend_.GetPoses(pose_estimates);
  std::vector<Pose3Stamped> pose_estimates_vector;
  for (const auto& pose_estimate : pose_estimates)
  {
    pose_estimates_vector.push_back(pose_estimate.second);
  }
  PublishPoses(pose_estimates_vector);
  if (!pose_estimates.empty())
  {
    auto latest_lidar_pose = new_backend_.GetLatestLidarPose();
    if (latest_lidar_pose)
    {
      PublishLatestLidarTransform(*latest_lidar_pose);
    }
  }
  UpdateAndPublishFullTrajectory(pose_estimates);

  std::map<int, LandmarkResult> landmark_estimates;
  new_backend_.GetLandmarks(landmark_estimates);
  PublishLandmarks(landmark_estimates, frame->timestamp);

  auto time_final = std::chrono::system_clock::now();

  auto micros_backend = std::chrono::duration_cast<std::chrono::microseconds>(time_after_backend - time_before);
  auto micros_extras = std::chrono::duration_cast<std::chrono::microseconds>(time_final - time_after_backend);
  auto micros_total = std::chrono::duration_cast<std::chrono::microseconds>(time_final - time_before);

  double millis_backend = static_cast<double>(micros_backend.count()) / 1000.;
  double millis_extras = static_cast<double>(micros_extras.count()) / 1000.;
  double millis_total = static_cast<double>(micros_total.count()) / 1000.;
  std::cout << "Time spent " << millis_total << " ms (back: " << millis_backend << ", extras: " << millis_extras << ")"
            << std::endl;
}
