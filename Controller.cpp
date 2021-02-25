#include "Controller.h"
#include "conversions.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

Controller::Controller(FeatureExtractor &frontend,
                       Smoother &backend,
                       ros::Publisher &posePublisher,
                       ros::Publisher &landmarkPublisher)
        : frontend(frontend),
          backend(backend),
          posePublisher(posePublisher),
          landmarkPublisher(landmarkPublisher) {}

void Controller::imageCallback(const sensor_msgs::Image::ConstPtr &msg) {
    static int callbackCount = 0;
    shared_ptr<Frame> newFrame = frontend.imageCallback(msg);
    if (callbackCount == 1) {
        auto twoFirstFrames = frontend.getFirstTwoFrames();
        backend.initializeFirstTwoPoses(twoFirstFrames.first, twoFirstFrames.second);
    }

    if (callbackCount > 1) {
        // TODO: Fails with IndeterminateLinearSystemException near variable 2. Figure out why.
        backend.update(newFrame);

        /*
        // This doesn't fail tho
        auto newestPose = backend.updateBatch(newFrame);
        nav_msgs::Odometry outMsg;
        outMsg.pose.pose = toPoseMsg(newestPose);
        outMsg.header.stamp = ros::Time(newFrame->timeStamp);
        outMsg.header.frame_id = "map";
        posePublisher.publish(outMsg);

        auto points = backend.getLandmarkEstimates();
        visualization_msgs::MarkerArray markerArray;
        for (int i = 1; i < points.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.pose.position.x = points[i].x();
            marker.pose.position.y = points[i].y();
            marker.pose.position.z = points[i].z();

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

            marker.action = visualization_msgs::Marker::MODIFY;

            marker.type = visualization_msgs::Marker::CUBE;

            marker.id = i;
            marker.ns = "landmarks";

            marker.header.stamp = ros::Time(newFrame->timeStamp);
            marker.header.frame_id = "map";

            markerArray.markers.push_back(marker);
        }
        landmarkPublisher.publish(markerArray);
         */
    }
    callbackCount++;
}
