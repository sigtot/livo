#ifndef ORB_TEST_FEATUREEXTRACTOR_H
#define ORB_TEST_FEATUREEXTRACTOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

class FeatureExtractor {
private:
    ros::Publisher pub;
public:
    FeatureExtractor(const ros::Publisher &pub);

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
};


#endif //ORB_TEST_FEATUREEXTRACTOR_H
