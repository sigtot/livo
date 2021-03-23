#include "controller.h"
#include "frame.h"
#include "ros_conversions.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pose3_stamped.h>

#include <chrono>
#include <thread>
#include <global_params.h>

Controller::Controller(FeatureExtractor& frontend, Smoother& backend, ros::Publisher& posePublisher,
                       ros::Publisher& landmarkPublisher)
  : frontend(frontend), backend(backend), pose_publisher_(posePublisher), landmark_publisher_(landmarkPublisher)
{
}

void Controller::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  auto new_frame = frontend.lkCallback(msg);
  std::cout << "frame " << new_frame->id << std::endl;
  if (frontend.ReadyForInitialization()) {
    std::cout << "R_H good. Ready for initialization!" << std::endl;
  }

  /*
  if (frontend.GetFrameCount() == 501) {
    std::vector<Pose3Stamped> pose_estimates;
    auto old_tracks = frontend.GetOldTracks();
    auto tracks = frontend.GetActiveTracks();
    tracks.insert(tracks.begin(), old_tracks.begin(), old_tracks.end());

    backend.InitIMUOnly(frontend.GetFrames(), tracks, pose_estimates);

    for (auto& pose_stamped : pose_estimates)
    {
      nav_msgs::Odometry odometry_msg;
      odometry_msg.header.stamp = ros::Time(pose_stamped.stamp);
      odometry_msg.pose.pose = ToPoseMsg(pose_stamped.pose);
      odometry_msg.header.frame_id = "world";
      pose_publisher_.publish(odometry_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    exit(0);
  }
   */

  if (frontend.CanPerformStationaryIMUInitialization()) {
    std::vector<Pose3Stamped> pose_estimates;
    backend.InitIMU(frontend.GetFrames(), pose_estimates);
    for (auto& pose_stamped : pose_estimates)
    {
      nav_msgs::Odometry odometry_msg;
      odometry_msg.header.stamp = ros::Time(pose_stamped.stamp);
      odometry_msg.pose.pose = ToPoseMsg(pose_stamped.pose);
      odometry_msg.header.frame_id = "world";
      pose_publisher_.publish(odometry_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  if (backend.GetStatus() == kIMUInitialized && frontend.ReadyForInitialization())
  {
    std::vector<Pose3Stamped> pose_estimates;
    std::vector<Point3> landmark_estimates;
    auto tracks = frontend.GetActiveTracks();
    for (auto & track : frontend.GetOldTracks()) {
      if (!track->key_features.empty())
      {
        tracks.push_back(track);
      }
    }

    backend.InitializeLandmarks(frontend.GetGoodKeyframeTransforms(), tracks, pose_estimates, landmark_estimates);
    for (auto& pose_stamped : pose_estimates)
    {
      nav_msgs::Odometry odometry_msg;
      odometry_msg.header.stamp = ros::Time(pose_stamped.stamp);
      odometry_msg.pose.pose = ToPoseMsg(pose_stamped.pose);
      odometry_msg.header.frame_id = "world";
      pose_publisher_.publish(odometry_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    visualization_msgs::MarkerArray markerArray;
    std::cout << "got " << landmark_estimates.size() << " landmarks from smoother" << std::endl;
    for (int i = 0; i < landmark_estimates.size(); ++i)
    {
      auto landmark = landmark_estimates[i];

      visualization_msgs::Marker marker;
      marker.pose.position = ToPointMsg(landmark);

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;

      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.id = i;
      marker.ns = "landmarks";
      marker.header.stamp = ros::Time(new_frame->timestamp);
      marker.header.frame_id = "world";
      markerArray.markers.push_back(marker);
    }
    landmark_publisher_.publish(markerArray);
  }
  else if (backend.GetStatus() == kLandmarksInitialized)
  {
    std::vector<Point3> landmark_estimates;
    auto pose_stamped = backend.Update(new_frame, frontend.GetActiveTracks(), landmark_estimates);
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.stamp = ros::Time(pose_stamped.stamp);
    odometry_msg.pose.pose = ToPoseMsg(pose_stamped.pose);
    odometry_msg.header.frame_id = "world";
    pose_publisher_.publish(odometry_msg);

    visualization_msgs::MarkerArray markerArray;
    for (int i = 0; i < landmark_estimates.size(); ++i)
    {
      auto landmark = landmark_estimates[i];

      visualization_msgs::Marker marker;
      marker.pose.position = ToPointMsg(landmark);

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;

      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.id = i;
      marker.ns = "landmarks";
      marker.header.stamp = ros::Time(new_frame->timestamp);
      marker.header.frame_id = "world";
      markerArray.markers.push_back(marker);
    }
    landmark_publisher_.publish(markerArray);
  }
  /*
  shared_ptr<Frame> new_frame = frontend.imageCallback(msg);
  frontend.PublishLandmarkTracksImage();
  int landmark_count_before = frontend.GetLandmarkCount();
  frontend.CullLandmarks(5, .6);
  frontend.CullLandmarks(10, .4);
  frontend.CullLandmarks(GlobalParams::LandmarkCullingFrameCount(),
                         GlobalParams::LandmarkCullingObservationPercentage());
  int landmark_count_after = frontend.GetLandmarkCount();
  std::cout << "Landmark count before and after cull: " << landmark_count_before << " -> " << landmark_count_after
            << "(" << landmark_count_before - landmark_count_after << " culled)" << std::endl;

  if (frontend.GetFrameCount() == 20)
  {
    std::vector<std::shared_ptr<Landmark>> non_culled_landmarks;
    for (const auto& landmark : frontend.GetLandmarks())
    {
      if (landmark.second->keypoint_observations.size() > 10)
      {
        non_culled_landmarks.push_back(landmark.second);
      }
    }
    // TryProjectDebug(non_culled_landmarks, 10.0, frontend.GetFrames()[0]->timestamp, landmark_publisher_);
    // TrySendGtsamSFMPoses(frontend.GetFrames()[0]->timestamp, pose_publisher_);
  }

  if (frontend.GetFrameCount() > 100)
  {
    std::vector<Pose3Stamped> pose_estimates;
    std::vector<Point3> landmark_estimates;
    Smoother::SmoothBatch(frontend.GetFrames(), frontend.GetLandmarks(), pose_estimates, landmark_estimates);
    std::cout << "poses:" << std::endl;
    for (auto& pose_stamped : pose_estimates)
    {
      std::cout << "[" << pose_stamped.pose.point.x << ", " << pose_stamped.pose.point.y << ", "
                << pose_stamped.pose.point.z << "]" << std::endl;
      nav_msgs::Odometry odometry_msg;
      odometry_msg.header.stamp = ros::Time(pose_stamped.stamp);
      odometry_msg.pose.pose = ToPoseMsg(pose_stamped.pose);
      odometry_msg.header.frame_id = "world";
      pose_publisher_.publish(odometry_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "landmarks:" << std::endl;
    visualization_msgs::MarkerArray markerArray;
    for (int i = 0; i < landmark_estimates.size(); ++i)
    {
      auto landmark = landmark_estimates[i];
      std::cout << "[" << landmark.x << ", " << landmark.y << ", " << landmark.z << "]" << std::endl;

      visualization_msgs::Marker marker;
      marker.pose.position = ToPointMsg(landmark);

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;

      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;

      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.id = i;
      marker.ns = "landmarks";
      marker.header.stamp = ros::Time(new_frame->timestamp);
      marker.header.frame_id = "world";
      markerArray.markers.push_back(marker);
    }
    landmark_publisher_.publish(markerArray);
    exit(0);
  }
   */
}
