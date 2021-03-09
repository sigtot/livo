#include "try_project_debug.h"

#include "gtsam_conversions.h"
#include "newer_college_ground_truth.h"
#include "ros_conversions.h"
#include "global_params.h"

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/Point2.h>
#include <visualization_msgs/MarkerArray.h>

typedef gtsam::PinholePose<gtsam::Cal3_S2> Camera;

void TryProjectDebug(const std::vector<std::shared_ptr<Landmark>>& landmarks, double depth, double timestamp,
                     ros::Publisher& landmark_publisher)
{
  gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(GlobalParams::CamFx(), GlobalParams::CamFy(), 0.0,
                                                  GlobalParams::CamU0(), GlobalParams::CamV0()));

  int i = 99999;
  visualization_msgs::MarkerArray marker_array;
  auto pose = NewerCollegeGroundTruth::At(timestamp);
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
    marker.header.frame_id = "map";
    marker_array.markers.push_back(marker);

    ++i;
  }
  landmark_publisher.publish(marker_array);
}