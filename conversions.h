#ifndef ORB_TEST_CONVERSIONS_H
#define ORB_TEST_CONVERSIONS_H

#include <gtsam/geometry/Pose3.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

using namespace gtsam;
using namespace std;

geometry_msgs::Pose toPoseMsg(const Pose3 &pose) {
    geometry_msgs::Point pointMsg;
    pointMsg.x = pose.x();
    pointMsg.y = pose.y();
    pointMsg.z = pose.z();

    geometry_msgs::Quaternion quatMsg;
    quatMsg.w = pose.rotation().toQuaternion().w();
    quatMsg.x = pose.rotation().toQuaternion().x();
    quatMsg.y = pose.rotation().toQuaternion().y();
    quatMsg.z = pose.rotation().toQuaternion().z();

    geometry_msgs::Pose poseMsg;
    poseMsg.position = pointMsg;
    poseMsg.orientation = quatMsg;
    return poseMsg;
}

#endif //ORB_TEST_CONVERSIONS_H
