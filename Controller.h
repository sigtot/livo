#ifndef ORB_TEST_CONTROLLER_H
#define ORB_TEST_CONTROLLER_H


#include "FeatureExtractor.h"
#include "Smoother.h"

class Controller {
private:
    FeatureExtractor &frontend;
    Smoother &backend;

public:
    explicit Controller(FeatureExtractor &frontend, Smoother &backend);

    void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
};


#endif
