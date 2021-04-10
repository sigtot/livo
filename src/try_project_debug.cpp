#include "try_project_debug.h"

#include "gtsam_conversions.h"
#include "newer_college_ground_truth.h"
#include "ros_conversions.h"
#include "global_params.h"

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/Point2.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <thread>

typedef gtsam::PinholePose<gtsam::Cal3_S2> Camera;

void TryProjectDebug(const std::vector<std::shared_ptr<Landmark>>& landmarks, double depth, double timestamp,
                     ros::Publisher& landmark_publisher)
{
  gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(GlobalParams::CamFx(), GlobalParams::CamFy(), 0.0,
                                                  GlobalParams::CamU0(), GlobalParams::CamV0()));

  int i = 99999;
  visualization_msgs::MarkerArray marker_array;
  auto pose = GroundTruth::At(timestamp);
  auto gtsam_pose = ToGtsamPose(pose);
  Camera camera(gtsam_pose, K);
  for (const auto& landmark : landmarks)
  {
    auto first_observation_keypoint = landmark->keypoint_observations[0]->keypoint.pt;
    gtsam::Point2 gtsam_point2(first_observation_keypoint.x, first_observation_keypoint.y);
    auto gtsam_point3 = camera.backproject(gtsam_point2, depth);

    auto point3 = ToPoint(gtsam_point3);
    std::cout << "[" << point3.x << ", " << point3.y << ", " << point3.z << "]" << std::endl;

    visualization_msgs::Marker marker;
    marker.pose.position = ToPointMsg(point3);

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.id = i;
    marker.ns = "landmarks";
    marker.header.stamp = ros::Time(timestamp);
    marker.header.frame_id = "world";
    marker_array.markers.push_back(marker);

    ++i;
  }
  landmark_publisher.publish(marker_array);
}

void TrySendGtsamSFMPoses(double timestamp, ros::Publisher& pose_publisher)
{
  const gtsam::Pose3& init = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI / 2, 0, -M_PI / 2), gtsam::Point3(30, 0, 0));
  const gtsam::Pose3& delta =
      gtsam::Pose3(gtsam::Rot3::Ypr(0, -M_PI / 4, 0), gtsam::Point3(sin(M_PI / 4) * 30, 0, 30 * (1 - sin(M_PI / 4))));
  int steps = 8;

  // Create the set of ground-truth poses
  // Default values give a circular trajectory, radius 30 at pi/4 intervals, always facing the circle center
  std::vector<gtsam::Pose3> poses;
  int i = 1;
  poses.push_back(init);
  for (; i < steps; ++i)
  {
    auto gtsam_pose = poses[i - 1].compose(delta);
    poses.push_back(gtsam_pose);
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.stamp = ros::Time(timestamp);
    odometry_msg.header.frame_id = "world";
    odometry_msg.pose.pose = ToPoseMsg(ToPose(gtsam_pose));
    pose_publisher.publish(odometry_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
  }
}
