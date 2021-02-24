#include "Controller.h"

using namespace std;

Controller::Controller(FeatureExtractor &frontend, Smoother &backend) : frontend(frontend), backend(backend) {}

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
        backend.updateBatch(newFrame);
    }
    callbackCount++;
}
