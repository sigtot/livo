#include "FeatureExtractor.h"

FeatureExtractor::FeatureExtractor(const ros::Publisher &pub) : pub(pub) {}

void FeatureExtractor::imageCallback(const sensor_msgs::Image::ConstPtr &img) {
    pub.publish(img);
    ROS_INFO("Published image");
}
