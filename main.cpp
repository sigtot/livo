#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include "FeatureExtractor.h"
#include "Controller.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "orb_test");
    ros::NodeHandle nh;

    auto matchesPub = nh.advertise<sensor_msgs::Image>("/matches_image", 1000);
    auto tracksPub = nh.advertise<sensor_msgs::Image>("/tracks_image", 1000);
    auto posePub = nh.advertise<nav_msgs::Odometry>("/pose", 1000);
    auto landmarksPub = nh.advertise<visualization_msgs::MarkerArray>("/landmarks", 1000);
    FeatureExtractor featureExtractor(matchesPub, tracksPub, 20);
    Smoother smoother;
    Controller controller(featureExtractor, smoother, posePub, landmarksPub);
    auto sub = nh.subscribe("/camera/infra1/image_rect_raw", 1000, &Controller::imageCallback, &controller);

    ROS_INFO("Starting up");

    ros::spin();
    return 0;
}
