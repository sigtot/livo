#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "FeatureExtractor.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "orb_test");
    ros::NodeHandle nh;

    auto matchesPub = nh.advertise<sensor_msgs::Image>("/matches_image", 1000);
    auto tracksPub = nh.advertise<sensor_msgs::Image>("/tracks_image", 1000);
    FeatureExtractor featureExtractor(matchesPub, tracksPub, 20);
    auto sub = nh.subscribe("/camera/image_mono", 1000, &FeatureExtractor::imageCallback, &featureExtractor);

    ROS_INFO("Starting up");

    ros::spin();
    return 0;
}
