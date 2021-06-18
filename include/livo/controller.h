#ifndef ORB_TEST_CONTROLLER_H
#define ORB_TEST_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "feature_extractor.h"
#include "new_smoother.h"
#include "point3.h"
#include "pose3_stamped.h"
#include "imu_ground_truth_smoother.h"
#include "lidar_frame_manager.h"
#include "landmark_result.h"
#include "between_transform_provider.h"

#include <mutex>
#include <condition_variable>
#include <memory>
#include <thread>
#include <boost/optional.hpp>

class Controller
{
private:
  FeatureExtractor& frontend_;
  LidarFrameManager& lidar_frame_manager_;
  NewSmoother& new_backend_;
  IMUGroundTruthSmoother& imu_ground_truth_smoother_;
  ros::Publisher path_publisher_;
  ros::Publisher landmark_publisher_;
  ros::Publisher pose_arr_publisher_;
  std::shared_ptr<BetweenTransformProvider> between_transform_provider_;
  std::shared_ptr<TimeOffsetProvider> lidar_time_offset_provider_;

  std::mutex back_cv_m_;
  std::condition_variable back_cv_;
  boost::optional<std::shared_ptr<Frame>> latest_frame_;  // Protected by back_cv_m_
  std::thread backend_thread_;                            // Thread for backend. Frontend thread is the current thread.

public:
  explicit Controller(FeatureExtractor& frontend, LidarFrameManager& lidar_frame_manager, NewSmoother& new_backend,
                      IMUGroundTruthSmoother& imu_ground_truth_smoother,
                      std::shared_ptr<BetweenTransformProvider> between_transform_provider,
                      std::shared_ptr<TimeOffsetProvider> lidar_time_offset_provider,
                      ros::Publisher& path_publisher, ros::Publisher& pose_arr_publisher,
                      ros::Publisher& landmark_publisher);

  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

  void SetLatestFrameForBackend(std::shared_ptr<Frame> frame);
  void BackendSpinner();
  void ProcessWithBackend(const std::shared_ptr<Frame>& frame);

  void PublishPoses(const std::vector<Pose3Stamped>& poses);
  void PublishLatestLidarTransform(const Pose3Stamped& pose_stamped);
  void PublishLandmarks(const std::map<int, LandmarkResult>& landmarks, double timestamp);
  void LidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  virtual ~Controller();
};

#endif
