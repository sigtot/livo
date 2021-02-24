#include "Controller.h"
#include "conversions.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;

Controller::Controller(FeatureExtractor &frontend,
                       Smoother &backend,
                       ros::Publisher &posePublisher)
        : frontend(frontend),
          backend(backend),
          posePublisher(posePublisher) {}

void Controller::imageCallback(const sensor_msgs::Image::ConstPtr &msg) {
    static int callbackCount = 0;
    shared_ptr<Frame> newFrame = frontend.imageCallback(msg);
    if (callbackCount == 1) {
        auto twoFirstFrames = frontend.getFirstTwoFrames();
        backend.initializeFirstTwoPoses(twoFirstFrames.first, twoFirstFrames.second);
    }

    if (callbackCount > 1) {
        // TODO: Fails with IndeterminateLinearSystemException near variable 2. Figure out why.
        //backend.update(newFrame);

        // This doesn't fail tho
        auto newestPose = backend.updateBatch(newFrame);
        nav_msgs::Odometry outMsg;
        outMsg.pose.pose = toPoseMsg(newestPose);
        outMsg.header.stamp = ros::Time(newFrame->timeStamp);
        outMsg.header.frame_id = "map";
        posePublisher.publish(outMsg);
    }
    callbackCount++;
}
